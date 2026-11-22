"""Optional local YOLOv8/11 ONNX proposals. Never download or deserialize .pt."""
from pathlib import Path
import math
import cv2
import numpy as np

class OnnxDetector:
    def __init__(self,path,class_map,confidence=.35):
        p=Path(path).expanduser().resolve()
        if p.suffix.lower()!='.onnx' or not p.is_file():
            raise ValueError('검증한 로컬 YOLO ONNX 파일이 필요합니다 (.pt 자동 로딩 없음)')
        if not isinstance(class_map,dict) or not class_map or any(type(k) is not int or k<0 or v not in ('adult','child','human') for k,v in class_map.items()):
            raise ValueError('명시적인 class index → adult/child/human 매핑이 필요합니다')
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        self.net=ort.InferenceSession(str(p), sess_options=options, providers=['CPUExecutionProvider'])
        inputs=self.net.get_inputs()
        if len(inputs)!=1 or inputs[0].shape!=[1,3,640,640]:raise ValueError('정적 1×3×640×640 모델이 필요합니다')
        self.input=inputs[0].name;self.class_map=dict(class_map);self.confidence=confidence
    def detect(self,frame):
        h,w=frame.shape[:2];scale=min(640/w,640/h)
        rw,rh=round(w*scale),round(h*scale);left,top=(640-rw)//2,(640-rh)//2
        canvas=np.full((640,640,3),114,np.uint8);canvas[top:top+rh,left:left+rw]=cv2.resize(frame,(rw,rh))
        blob=cv2.dnn.blobFromImage(canvas,1/255,(640,640),swapRB=True)
        rows=np.asarray(self.net.run(None,{self.input:blob})[0])
        if rows.ndim!=3 or rows.shape[0]!=1:raise ValueError('지원하지 않는 YOLO 출력')
        rows=rows[0]
        if rows.shape[0]<rows.shape[1]:rows=rows.T
        if rows.shape[1]!=4+len(self.class_map) or set(self.class_map)!=set(range(len(self.class_map))):raise ValueError('YOLOv8/11 xywh+class 출력이 필요합니다')
        finite_rows=rows[np.isfinite(rows).all(axis=1)]
        class_ids=finite_rows[:,4:].argmax(axis=1)
        confidence=finite_rows[np.arange(len(finite_rows)),4+class_ids].astype(float)
        allowed_classes=[index for index,label in self.class_map.items() if label in ('adult','child')]
        keep=(confidence>=self.confidence)&np.isin(class_ids,allowed_classes)
        boxes=[];scores=[];classes=[]
        for r,cls,score in zip(finite_rows[keep],class_ids[keep],confidence[keep]):
            x,y,bw,bh=map(float,r[:4]);x1=max(0,(x-bw/2-left)/scale);y1=max(0,(y-bh/2-top)/scale)
            x2=min(w,(x+bw/2-left)/scale);y2=min(h,(y+bh/2-top)/scale)
            if x2-x1<32 or y2-y1<64:continue
            boxes.append([x1,y1,x2-x1,y2-y1]);scores.append(score);classes.append(cls)
        ids=cv2.dnn.NMSBoxes(boxes,scores,self.confidence,.45)
        return [{'box':[boxes[i][0],boxes[i][1],boxes[i][0]+boxes[i][2],boxes[i][1]+boxes[i][3]],'score':scores[i],'label':self.class_map[classes[i]]} for i in np.asarray(ids).reshape(-1)[:20]]

def proposals(detector,frame):
    """Validate injected/native detector output before exposing selectable IDs."""
    h,w=frame.shape[:2];out=[]
    for item in detector.detect(frame):
        b=item.get('box');score=item.get('score',0)
        if not isinstance(b,(list,tuple)) or len(b)!=4 or not all(isinstance(v,(int,float,np.number)) and math.isfinite(v) for v in b):continue
        x1,y1,x2,y2=map(float,b)
        if not 0<=x1<x2<=w or not 0<=y1<y2<=h or x2-x1<32 or y2-y1<64:continue
        if not isinstance(score,(int,float,np.number)) or not math.isfinite(score) or not 0<=score<=1:continue
        out.append({'box':[x1/w,y1/h,x2/w,y2/h],'score':float(score),'label':str(item.get('label','object'))[:40]})
        if len(out)==20:break
    return out


class ShoppingDetector(OnnxDetector):
    """Also inspect a closer view so small distant figures can be selected."""
    def zoom(self, frame):
        boxes = []
        h, w = frame.shape[:2]
        x, y = w//8, h//8
        crop = frame[y:h-y, x:w-x]
        ch, cw = crop.shape[:2]
        for item in super().detect(cv2.resize(crop, (640, 640))):
            a, b, c, d = item['box']
            if a < 2 or b < 2 or c > 638 or d > 638:
                continue
            box = [x+a*cw/640, y+b*ch/640, x+c*cw/640, y+d*ch/640]
            if box[2]-box[0] >= 32 and box[3]-box[1] >= 64:
                boxes.append({**item, 'box': box})
        return boxes

    def full_frame(self, frame):
        return super().detect(frame)

    def detect(self, frame):
        boxes = super().detect(frame) + self.zoom(frame)
        xywh = [[a,b,c-a,d-b] for a,b,c,d in (item['box'] for item in boxes)]
        ids = cv2.dnn.NMSBoxes(xywh, [b['score'] for b in boxes], self.confidence, .4)
        return [boxes[i] for i in np.asarray(ids).reshape(-1)[:20]]


class SelectedProposals:
    """Quick image candidates between YOLO passes; only DINO can confirm identity."""
    def __init__(self, frame, box):
        h, w = frame.shape[:2]
        a,b,c,d = [round(v*n) for v,n in zip(box,(w,h,w,h))]
        self.crop = cv2.cvtColor(frame[b:d,a:c], cv2.COLOR_BGR2GRAY)
        self.colors = self.histogram(frame[b:d,a:c])
        self.sh = d-b
        self.sw = c-a

    @staticmethod
    def histogram(crop):
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv],[0,1,2],None,[12,4,4],[0,180,0,256,0,256])
        return cv2.normalize(hist,hist,alpha=1,norm_type=cv2.NORM_L1)

    def detect(self, frame):
        h, w = frame.shape[:2]
        gray = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (w//2,h//2))
        items = []
        min_correlation = .60
        for scale in (.32,.40,.50,.64,.8,1.,1.25,1.56,1.95,2.44,3.05,3.81):
            bw, bh = round(self.sw*scale/2), round(self.sh*scale/2)
            if bw<16 or bh<32 or bw>=gray.shape[1] or bh>=gray.shape[0]:
                continue
            sample = cv2.resize(self.crop,(bw,bh))
            x, y = bw//6, bh//8
            body = sample[y:bh-2*y, x:bw-x]
            if np.std(body)<12:
                continue
            scores = cv2.matchTemplate(gray, body, cv2.TM_CCOEFF_NORMED)
            for _ in range(3):
                _, score, _, (u,v) = cv2.minMaxLoc(scores)
                if score <= min_correlation:
                    break
                a,b = (u-x)*2, (v-y)*2
                if a>=0 and b>=0 and a+2*bw<=w and b+2*bh<=h:
                    colors = self.histogram(frame[b:b+2*bh,a:a+2*bw])
                    if cv2.compareHist(self.colors, colors, cv2.HISTCMP_BHATTACHARYYA) < .55:
                        items.append(([a,b,2*bw,2*bh], score))
                scores[max(0,v-bh//2):v+bh//2+1,max(0,u-bw//2):u+bw//2+1] = -1.
        ids = cv2.dnn.NMSBoxes([i[0] for i in items],[i[1] for i in items],min_correlation,.35)
        return [dict(box=[a,b,a+bw,b+bh], score=score) for index in np.asarray(ids).reshape(-1)[:6]
                for (a,b,bw,bh),score in [items[index]]]
