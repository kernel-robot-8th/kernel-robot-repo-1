import { useEffect, useRef, useState } from 'react';
import { shopping, shoppingImage, type ShoppingStatus } from './api';
import { automaticShopper } from './flow';
import { Icon } from './icons';

export function Shopping({onBack, onFinish, onGate}: {onBack: () => void; onFinish: () => void; onGate: () => void}) {
  const [session] = useState(() => crypto.randomUUID());
  const [status, setStatus] = useState<ShoppingStatus | null>(null);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const [image, setImage] = useState('');
  const [offline, setOffline] = useState(false);
  const hasImage = Boolean(image);
  const [loaded, setLoaded] = useState('');
  const [picture, setPicture] = useState<{url: string; path: string} | null>(null);
  const pictureUrl = useRef('');
  useEffect(() => {
    if (!image || offline) return;
    const controller = new AbortController();
    void shoppingImage(session, image, AbortSignal.any([controller.signal, AbortSignal.timeout(1200)])).then(blob => {
      if (!blob || controller.signal.aborted) return;
      const url = URL.createObjectURL(blob);
      if (pictureUrl.current) URL.revokeObjectURL(pictureUrl.current);
      pictureUrl.current = url;
      setPicture({url, path: image});
    }).catch(() => {});
    return () => controller.abort();
  }, [session, image, offline]);
  useEffect(() => {
    if (hasImage && !offline) return;
    if (pictureUrl.current) URL.revokeObjectURL(pictureUrl.current);
    pictureUrl.current = '';
    setPicture(null);
    setLoaded('');
  }, [hasImage, offline]);
  useEffect(() => () => {
    if (pictureUrl.current) URL.revokeObjectURL(pictureUrl.current);
    pictureUrl.current = '';
  }, []);
  const [portrait, setPortrait] = useState('');
  const [started, setStarted] = useState(false);
  const [manual, setManual] = useState(false);
  const [cursor, setCursor] = useState([.5,.5]);
  const [rect, setRect] = useState<number[] | null>(null);
  const drag = useRef<{frame: string; x: number; y: number} | null>(null);
  const video = useRef<HTMLDivElement>(null);
  const audio = useRef<AudioContext | null>(null);
  const chime = Boolean(!offline && status?.armed && status.notice === 'COME_CLOSER');
  useEffect(() => {
    if (!chime || !audio.current) return;
    const ctx = audio.current;
    const volume = ctx.createGain();
    volume.connect(ctx.destination);
    function ring() {
      if (ctx.state !== 'running' || document.hidden) return;
      const now = ctx.currentTime;
      const tone = ctx.createOscillator();
      const envelope = ctx.createGain();
      tone.frequency.setValueAtTime(523.25, now);
      tone.frequency.setValueAtTime(659.25, now + .16);
      envelope.gain.setValueAtTime(0, now);
      envelope.gain.linearRampToValueAtTime(.10, now + .02);
      envelope.gain.linearRampToValueAtTime(0, now + .14);
      envelope.gain.linearRampToValueAtTime(.10, now + .18);
      envelope.gain.linearRampToValueAtTime(0, now + .38);
      tone.connect(envelope).connect(volume);
      tone.onended = () => { tone.disconnect(); envelope.disconnect(); };
      tone.start(now); tone.stop(now + .39);
    }
    ring();
    const timer = window.setInterval(ring, 3000);
    return () => { window.clearInterval(timer); volume.disconnect(); };
  }, [chime]);
  useEffect(() => () => {
    if (audio.current) void audio.current.close().catch(() => {});
    audio.current = null;
  }, []);
  const active = useRef(false), revision = useRef(0), mounted = useRef(true), pending = useRef(false);
  useEffect(() => {
    let closed = false;
    mounted.current = true;
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      const before = revision.current;
      try {
        const s = await shopping(session, active.current && !document.hidden ? 'heartbeat' : 'status');
        if (!closed && before === revision.current && !pending.current) {
          if (active.current && !s.ownsCamera) {
            active.current = false;
            setPortrait(''); setStarted(false); setRect(null); drag.current = null;
          }
          setStatus(s); setOffline(false);
          setImage(s.ownsCamera ? '/shopping/image?' + (s.frame ? 'frame='+s.frame : 't='+Date.now()) : '');
        }
      } catch {
        if (!closed && before === revision.current) setOffline(true);
      }
      if (!closed) timer = setTimeout(poll, 240);
    };
    void poll();
    return () => { closed = true; mounted.current = false; clearTimeout(timer); };
  }, [session]);

  async function command(action: string, args: Record<string,unknown> = {}, then?: () => void) {
    if (pending.current && action !== 'stop') return;
    if (action === 'start' && typeof AudioContext !== 'undefined') {
      // Unlock the pad speaker during the passenger's tap, before the network request.
      try {
        audio.current ??= new AudioContext();
        void audio.current.resume().catch(() => {});
      } catch { /* The visual guidance still works without audio output. */ }
    }
    if (action === 'capture' || action === 'select') { setRect(null); drag.current = null; }
    let chosen = '';
    if (action === 'select') {
      const img = video.current?.querySelector('img');
      const bounds = Array.isArray(args.box) ? args.box : status?.boxes[Number(args.index)];
      if (img?.complete && img.naturalWidth && bounds) {
        const [a,b,c,d] = bounds;
        const canvas = document.createElement('canvas');
        canvas.width = 84; canvas.height = Math.max(1, Math.min(240, 84*(d-b)*img.naturalHeight/((c-a)*img.naturalWidth)));
        canvas.getContext('2d')?.drawImage(img,a*img.naturalWidth,b*img.naturalHeight,
          (c-a)*img.naturalWidth,(d-b)*img.naturalHeight,0,0,canvas.width,canvas.height);
        chosen = canvas.toDataURL('image/jpeg');
      }
    }
    const rev = ++revision.current;
    pending.current = true; setBusy(true); setError('');
    try {
      const s = await shopping(session, action, {...args, order: rev});
      if (!mounted.current || revision.current !== rev) return;
      active.current = s.ownsCamera;
      if (action === 'select') setPortrait(chosen);
      if (action === 'capture') { setPortrait(''); setStarted(false); setManual(false); }
      if (action === 'start') setStarted(true);
      setStatus(s); setOffline(false);
      setImage(s.ownsCamera ? '/shopping/image?' + (s.frame ? 'frame='+s.frame : 't='+Date.now()) : '');
      then?.();
    } catch (err) {
      if (mounted.current && revision.current === rev) setError(err instanceof Error ? err.message : '연결을 확인해 주세요.');
    } finally {
      if (revision.current === rev) {
        pending.current = false;
        if (mounted.current) setBusy(false);
      }
    }
  }
  function leave(go: () => void) {
    if (active.current) void command('end', {}, go);
    else go();
  }
  const s = status;
  const frozen = !!s?.frame && !offline && loaded.endsWith('frame='+s.frame);
  const autoSelected = useRef<string | null>(null);
  useEffect(() => {
    if (!s?.frame || !frozen || manual || busy || pending.current || document.hidden ||
        s.detecting || autoSelected.current === s.frame) return;
    const index = automaticShopper(s.boxes);
    if (index === null) return;
    autoSelected.current = s.frame;
    void command('select', {frame: s.frame, index});
  });
  function pointer(e: React.PointerEvent) {
    const bounds = video.current!.getBoundingClientRect();
    const clip = (v: number) => Math.min(1, Math.max(0, v));
    return [clip((e.clientX-bounds.left)/bounds.width), clip((e.clientY-bounds.top)/bounds.height)];
  }
  function corners(x:number, y:number) {
    const first = drag.current!;
    return [Math.min(x,first.x), Math.min(y,first.y), Math.max(x,first.x), Math.max(y,first.y)];
  }
  const picking = !!s?.frame;
  const drawing = picking && (manual || !s?.detecting && !s?.boxes.length);
  const waiting = s?.armed && ['SEARCHING','AMBIGUOUS','VERIFYING'].includes(s.state);
  const notice = offline ? undefined : s?.notice;
  const title = notice === 'COME_CLOSER' ? '카트 앞으로 조금 더 가까이 와 주세요' :
    notice === 'BLOCKED' ? '카트가 지나갈 길을 비켜 주세요' : notice === 'UNAVAILABLE' ? '카트 상태를 확인하고 있어요' : offline ? '카트를 연결하고 있어요' : !s?.selected ?
    (picking && s?.detecting && !manual ? '사진에서 대상을 찾고 있어요' :
     picking ? '함께 갈 사람을 골라 주세요' : '카트 앞에 서 주세요') :
    s.armed ? (s.state==='CLOSE' ? '조금 앞서 걸어 주세요' : waiting ? '다시 찾고 있어요' :
      s.state==='CORNER' ? '모퉁이를 확인하고 있어요' : '함께 이동하고 있어요') :
      started ? '다시 출발할까요?' : '이 모습이 맞나요?';
  const lead = notice === 'COME_CLOSER' ? '카메라 앞에 잠시 서 주세요. 같은 사람을 확인하면 다시 따라갈게요.' :
    notice === 'BLOCKED' ? (s?.armed ? '잠시 멈췄어요. 길과 대상을 확인한 뒤 다시 시도해요.' : '통로를 비우고 카트 앞에 서서 다시 시작을 눌러 주세요.') : notice === 'UNAVAILABLE' ? s?.reason : offline ? '연결되면 이곳에 카메라 영상이 나타나요.' :
    !s?.selected ? (drawing ? '화면에서 본인의 모습을 손가락으로 감싸듯 사각형을 그려 주세요.' :
      s?.detecting ? '가운데 모습이 뚜렷하면 자동으로 선택돼요.' :
      picking ? '사진 속 내 번호를 누르세요. 번호가 없으면 직접 선택할 수 있어요.' : '화면 가운데에 서서 내 모습 찾기를 눌러 주세요.') :
    s.armed ? (s.state==='CLOSE' ? '카트가 기다리고 있어요. 간격이 생기면 다시 따라갈게요.' : waiting ?
      '카트 앞에 다시 모습을 보여 주세요. 같은 사람을 확인하면 이어서 따라가요.' :
      '앞에서 천천히 걸어 주세요. 언제든 아래 버튼으로 멈출 수 있어요.') :
      (s.reason ? '카트가 출발을 준비하고 있어요. 잠시 기다려 주세요.' :
       s.canStart ? '준비되면 시작을 누르고 앞에서 천천히 걸어 주세요.' : '카트 쪽을 보고 잠시 서 주세요.');
  return <section className="shopping" aria-label="쇼핑 모드">
    <div className="shopping-camera"><div className="shopping-camera-head"><strong>카트 앞 카메라{s?.simulation ? ' · 시뮬레이션' : ''}</strong>
      <span className={picking ? 'still' : ''}>{picking ? '선택용 사진' : offline || !loaded ? '연결 대기' : '실시간'}</span></div>
    <div className="shopping-video" ref={video}>
      {(!loaded || offline) && <span className="camera-empty"><Icon name="follow" />{!offline && s && !s.ownsCamera ? '카메라를 켜 주세요' : '카메라를 기다리는 중'}</span>}
      {image && !offline && picture && <img src={picture.url} alt="카트 앞 카메라" onLoad={() => setLoaded(picture.path)} draggable={false} />}
      {s?.frame && frozen && !manual && s.boxes.map(([a,b,c,d], index) =>
        <button key={`${s.frame}-${index}`} className="shop-box" disabled={busy} aria-label={'대상 '+(index+1)+' 선택'}
          style={{left:a*100+'%', top:b*100+'%', width:(c-a)*100+'%', height:(d-b)*100+'%'}}
          onClick={() => command('select',{frame:s.frame,index})}><span>{index+1}</span></button>)}
      {s?.frame && frozen && drawing &&
        <div className="shop-drag" role="group" tabIndex={0}
        aria-label="직접 선택. 방향키로 이동하고 엔터로 사각형의 두 모서리를 지정하세요."
        onKeyDown={e => {
          if (busy) return;
          if (e.key.startsWith('Arrow')) {
            e.preventDefault();
            const x=Math.max(0,Math.min(1,cursor[0]+(e.key==='ArrowLeft'?-.025:e.key==='ArrowRight'?.025:0)));
            const y=Math.max(0,Math.min(1,cursor[1]+(e.key==='ArrowUp'?-.025:e.key==='ArrowDown'?.025:0)));
            setCursor([x,y]); if (drag.current) setRect(corners(x,y));
          } else if(e.key==='Enter') {
            e.preventDefault();
            if (drag.current) { const box=corners(cursor[0],cursor[1]); drag.current=null; void command('select',{frame:s.frame,box}); }
            else drag.current={frame:s.frame!,x:cursor[0],y:cursor[1]};
          } else if(e.key==='Escape') { drag.current=null; setRect(null); }
        }} onPointerDown={e => {
          if (busy || !e.isPrimary) return;
          const [x,y] = pointer(e); drag.current = {frame:s.frame!,x,y};
          e.currentTarget.setPointerCapture(e.pointerId); setRect([x,y,x,y]);
        }} onPointerMove={e => { if (drag.current && e.isPrimary) { const [x,y]=pointer(e); setRect(corners(x,y)); } }}
        onPointerCancel={() => { drag.current=null; setRect(null); }}
        onPointerUp={e => {
          const first=drag.current; if (!first || first.frame!==s.frame || !e.isPrimary) return;
          const [x,y]=pointer(e); const box=corners(x,y); drag.current=null; setRect(null);
          void command('select',{frame:first.frame,box});
        }}>
          <i className="shop-cursor" style={{left:cursor[0]*100+'%',top:cursor[1]*100+'%'}}/>
          {rect && <span className="shop-box" style={{left:rect[0]*100+'%',top:rect[1]*100+'%',
            width:(rect[2]-rect[0])*100+'%',height:(rect[3]-rect[1])*100+'%'}}/>}
        </div>}
      {s?.box && !s.frame && <span className="shop-box shop-track" style={{left:s.box[0]*100+'%',top:s.box[1]*100+'%',
        width:(s.box[2]-s.box[0])*100+'%',height:(s.box[3]-s.box[1])*100+'%'}}/>}
    </div>
    </div>
    <div className="shop-controls">
      <h1 aria-live="polite" aria-atomic="true">{title}</h1>
      <p className="lead" role="status">{lead}</p>
      {s?.selected && portrait && <div className="shop-person"><img src={portrait} alt="선택한 모습"/><span>함께 이동할 사람</span></div>}
      {picking && !drawing && frozen && <div className="shop-numbers" aria-label="사진 속 사람 선택">
        {s!.boxes.map((_,i) => <button key={i} className="btn btn-primary" disabled={busy}
          onClick={() => command('select',{frame:s!.frame,index:i})}>{i+1}번 선택</button>)}
      </div>}
      {error && <p className="shop-error" role="alert">{/crop|box|pixel|선택해/.test(error)
        ? '몸통이 안에 들어오도록 조금 더 크게 그려 주세요.' : error}</p>}
      <div className="actions">
        {s && !s.ownsCamera && <button className="btn btn-primary btn-block" disabled={busy || offline} onClick={() => command('preview')}>카메라 켜기</button>}
        {s?.ownsCamera && !s.armed && <button className={'btn '+(!picking && !s?.selected ? 'btn-primary btn-block' : 'btn-secondary')} disabled={busy || offline} onClick={() => command('capture')}>{s?.frame || s?.selected ? '다시 찍기' : '내 모습 찾기'}</button>}
        {s?.frame && s.boxes.length > 0 && <button className="btn btn-secondary" disabled={busy}
          onClick={() => { setManual(!manual); setRect(null); }}>{manual ? '번호로 선택' : '직접 선택'}</button>}
        {s?.selected && !s.armed && <button className="btn btn-secondary" disabled={busy || offline}
          onClick={() => command('capture', {}, () => setManual(true))}>직접 선택</button>}
        {s?.selected && !s.armed && <button className="btn btn-primary" disabled={busy || offline || !s.canStart}
          onClick={() => command('start')}><Icon name="play" />{started ? '다시 시작' : '따라가기 시작'}</button>}
        {(s?.armed || busy && s?.selected) && <button className="btn btn-stop btn-block" onClick={() => command('stop')}><Icon name="pause" />잠시 멈추기</button>}
        {!s?.armed && (!s?.selected || s.stopped) && <>
          <button className="btn btn-secondary" disabled={busy} onClick={() => leave(onBack)}>이전</button>
          {s?.selected && <><button className="btn btn-secondary" disabled={busy || offline} onClick={() => leave(onGate)}>게이트 안내로 전환</button>
          <button className="btn btn-secondary" disabled={busy || offline} onClick={() => leave(onFinish)}>쇼핑 종료</button></>}
        </>}
      </div>
    </div>
  </section>;
}
