"""로봇(Pinky)에서 실행: Picamera2 영상을 MPJPEG로 stdout 에 낸다(카메라만, 모터·ROS 없음).

노트북 도구가 SSH 로 이 파일 내용을 보내 실행한다(로봇에 설치할 것 없음):
  ssh pinky@<IP> python3 -u - --fps 10 --quality 80 < robot_cam_stream.py
각 프레임 머리에 로봇 시각(X-Robot-Time, epoch s)을 붙인다. 표준 라이브러리 + picamera2 + cv2 만 쓴다.
"""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--quality', type=int, default=80)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--no-rotate', action='store_true', help='180도 회전 안 함(기본은 콘솔과 같이 회전)')
    # 노출 고정(0 = 자동). 짧게 하면 회전할 때 번짐이 줄어든다(11-20: 1 rad/s 회전 중 카드 인식 끊김)
    ap.add_argument('--exposure-us', type=int, default=0)
    ap.add_argument('--gain', type=float, default=0.0)
    a = ap.parse_args()
    if not (0.5 <= a.fps <= 30 and 30 <= a.quality <= 95 and 160 <= a.width <= 1280
            and 120 <= a.height <= 960):
        sys.exit('bad --fps/--quality/--width/--height')
    if a.exposure_us and not 100 <= a.exposure_us <= 33000 or a.gain and not 1.0 <= a.gain <= 16.0:
        sys.exit('bad --exposure-us (100..33000) or --gain (1..16)')
    import cv2
    from picamera2 import Picamera2
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={'size': (a.width, a.height), 'format': 'RGB888'}))
    cam.start()
    if a.exposure_us:
        ctl = {'AeEnable': False, 'ExposureTime': int(a.exposure_us)}
        if a.gain:
            ctl['AnalogueGain'] = float(a.gain)
        cam.set_controls(ctl)
    out = sys.stdout.buffer
    period = 1.0 / a.fps
    nxt = time.monotonic()
    try:
        while True:
            frame = cam.capture_array()          # RGB888 = 메모리상 BGR(OpenCV 와 같음)
            stamp = time.time()
            if not a.no_rotate:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            if ok:
                data = jpg.tobytes()
                out.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                          b'X-Robot-Time: %.6f\r\nContent-Length: %d\r\n\r\n' % (stamp, len(data)))
                out.write(data)
                out.write(b'\r\n')
                out.flush()
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 1.0:     # 많이 밀리면 따라잡지 않고 새로 맞춤
                nxt = time.monotonic()
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        cam.stop()
        cam.close()


if __name__ == '__main__':
    main()
