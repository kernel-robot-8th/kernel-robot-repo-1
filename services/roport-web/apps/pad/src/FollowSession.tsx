import { useEffect, useRef, useState, type PointerEvent } from 'react';
import * as api from './api';
import { followView } from './flow';
import { newerFollowStatus, validBox, type Box, type FollowAction, type FollowOperation, type FollowStatus } from './follow';
import { Icon } from './icons';
import { newVerificationRequestId } from './verify';

export function FollowSession({ receiptId, onClose }: { receiptId: string; onClose: () => void }) {
  const [status, setStatus] = useState<FollowStatus | null>(null);
  const [offline, setOffline] = useState(false);
  const [message, setMessage] = useState('');
  const [pending, setPending] = useState<FollowAction | null>(null);
  const [retry, setRetry] = useState<FollowAction | null>(null);
  const [awaiting, setAwaiting] = useState<FollowAction | null>(null);
  const [picture, setPicture] = useState<{ url: string; frame: string } | null>(null);
  const [manual, setManual] = useState(false);
  const [rect, setRect] = useState<Box | null>(null);
  const [cursor, setCursor] = useState<[number, number]>([.25, .25]);
  const drag = useRef<{ frame: string; x: number; y: number } | null>(null);
  const video = useRef<HTMLDivElement>(null);
  const mounted = useRef(false);
  const sending = useRef<FollowAction | null>(null);

  useEffect(() => {
    mounted.current = true;
    const abort = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const next = await api.followStatus(receiptId, AbortSignal.any([abort.signal, AbortSignal.timeout(8000)]));
        if (abort.signal.aborted) return;
        setStatus(current => newerFollowStatus(current, next));
        setOffline(false);
      } catch {
        if (!abort.signal.aborted) setOffline(true);
      } finally {
        if (!abort.signal.aborted) timer = setTimeout(poll, 1500);
      }
    }
    void poll();
    return () => { mounted.current = false; abort.abort(); clearTimeout(timer); };
  }, [receiptId]);

  useEffect(() => {
    if (!status) return;
    if (awaiting && (status.revision > awaiting.expectedRevision || status.lastRequest?.requestId === awaiting.requestId)) setAwaiting(null);
    if (retry && status.lastRequest?.requestId === retry.requestId) {
      setRetry(null); setMessage('요청 처리 상태를 확인했습니다. 실제 카트 상태를 표시합니다.');
    }
  }, [status, awaiting, retry]);

  const registration = status?.registration;
  const frame = registration?.frameRef;
  const path = registration?.imagePath;
  const registered = registration?.registered;
  const expired = Boolean(registration && Date.parse(registration.expiresAt) <= Date.now());
  useEffect(() => {
    setPicture(null); setRect(null); setManual(false); drag.current = null;
    if (!frame || !path || registered || expired) return;
    const abort = new AbortController();
    let objectUrl = '';
    void api.followImage(receiptId, frame, path, abort.signal).then(blob => {
      if (abort.signal.aborted) return;
      objectUrl = URL.createObjectURL(blob);
      setPicture({ url: objectUrl, frame });
    }).catch(() => { if (!abort.signal.aborted) setMessage('사진을 불러오지 못했습니다. 다시 찍어 주세요.'); });
    return () => { abort.abort(); if (objectUrl) URL.revokeObjectURL(objectUrl); };
  }, [receiptId, frame, path, registered, expired]);

  async function submit(action: FollowAction) {
    // An explicit stop can supersede a request still waiting for its HTTP response.
    if (sending.current && (action.operation !== 'pause' || sending.current.operation === 'pause')) return;
    sending.current = action; setPending(action); setRetry(null); setAwaiting(null); setMessage('요청을 보내고 있습니다.');
    try {
      const accepted = await api.followAction(receiptId, action);
      if (!mounted.current || sending.current !== action) return;
      if (accepted) setAwaiting(action);
      setMessage(accepted ? '요청을 접수했습니다. 실제 카트 상태를 확인하고 있습니다.' : '현재 상태에서는 요청할 수 없습니다. 상태가 갱신된 뒤 다시 확인해 주세요.');
    } catch {
      if (!mounted.current || sending.current !== action) return;
      setRetry(action); setMessage('요청 결과를 받지 못했습니다. 같은 요청의 결과를 다시 확인할 수 있습니다.');
    } finally {
      if (sending.current === action) {
        sending.current = null;
        if (mounted.current) setPending(null);
      }
    }
  }

  const allowed = (operation: FollowOperation) => Boolean(status && !offline && !status.stale && status.allowedActions.includes(operation));
  const disabled = (operation: FollowOperation) => !allowed(operation) || Boolean(operation === 'pause' ? (pending || retry || awaiting)?.operation === 'pause' : pending || retry || awaiting);
  function command(operation: FollowOperation, fields: Partial<Pick<FollowAction, 'frameRef' | 'candidateId' | 'box'>> = {}) {
    if (!status || disabled(operation)) return;
    if (operation === 'select' && (!registration || Date.parse(registration.expiresAt) <= Date.now())) {
      setMessage('사진의 선택 시간이 지났습니다. 다시 찍어 주세요.'); return;
    }
    void submit({ requestId: newVerificationRequestId(), expectedRevision: status.revision, operation, ...fields });
  }
  function selectBox(box: Box, selectedFrame: string) {
    setRect(null); drag.current = null;
    if (!validBox(box) || box[2] - box[0] < .03 || box[3] - box[1] < .03) {
      setMessage('본인의 몸통을 포함하도록 조금 더 크게 선택해 주세요.'); return;
    }
    command('select', { frameRef: selectedFrame, box });
  }
  function point(event: PointerEvent): [number, number] {
    const bounds = video.current!.getBoundingClientRect();
    return [Math.max(0, Math.min(1, (event.clientX - bounds.left) / bounds.width)), Math.max(0, Math.min(1, (event.clientY - bounds.top) / bounds.height))];
  }
  function corners(x: number, y: number): Box {
    const first = drag.current!;
    return [Math.min(x, first.x), Math.min(y, first.y), Math.max(x, first.x), Math.max(y, first.y)];
  }
  const view = followView(status?.stage || '', status?.status || '', status?.mode || 'none');
  const selectable = Boolean(frame && registration && picture?.frame === frame && Date.parse(registration.expiresAt) > Date.now() && !disabled('select'));
  const terminal = status && (status.status === 'COMPLETE' || status.status === 'COMPLETED' || status.status === 'CANCELED' || status.stage === 'done');

  return <section className="shopping follow-session" aria-label="카트 서비스 상태">
    {status?.mode === 'appearance' && !registered ? <div className="shopping-camera">
      <div className="shopping-camera-head"><strong>등록용 사진</strong><span className="still">{picture ? '직접 선택' : '촬영 대기'}</span></div>
      <div className="shopping-video" ref={video}>
        {picture ? <img src={picture.url} alt="대상 등록을 위해 촬영한 카트 앞 모습" draggable={false} /> : <span className="camera-empty"><Icon name="follow" />사진 촬영을 기다리고 있습니다</span>}
        {selectable && !manual && registration?.candidates?.map((candidate, index) => {
          const [a,b,c,d] = candidate.box;
          return <button key={candidate.candidateId} className="shop-box" aria-label={`대상 ${index + 1} 선택`}
            style={{ left: `${a*100}%`, top: `${b*100}%`, width: `${(c-a)*100}%`, height: `${(d-b)*100}%` }}
            onClick={() => command('select', { frameRef: frame, candidateId: candidate.candidateId })}><span>{index + 1}</span></button>;
        })}
        {selectable && manual && frame && <div className="shop-drag" role="group" tabIndex={0}
          aria-label="직접 선택. 방향키로 이동하고 엔터로 사각형의 두 모서리를 지정하세요."
          onKeyDown={e => {
            if (e.key.startsWith('Arrow')) {
              e.preventDefault();
              const x = Math.max(0, Math.min(1, cursor[0] + (e.key === 'ArrowLeft' ? -.025 : e.key === 'ArrowRight' ? .025 : 0)));
              const y = Math.max(0, Math.min(1, cursor[1] + (e.key === 'ArrowUp' ? -.025 : e.key === 'ArrowDown' ? .025 : 0)));
              setCursor([x,y]); if (drag.current) setRect(corners(x,y));
            } else if (e.key === 'Enter') {
              e.preventDefault();
              if (drag.current) selectBox(corners(...cursor), frame);
              else drag.current = { frame, x: cursor[0], y: cursor[1] };
            } else if (e.key === 'Escape') { drag.current = null; setRect(null); }
          }}
          onPointerDown={e => { if (!e.isPrimary) return; const [x,y] = point(e); drag.current = { frame, x, y }; setRect([x,y,x,y]); e.currentTarget.setPointerCapture(e.pointerId); }}
          onPointerMove={e => { if (drag.current && e.isPrimary) setRect(corners(...point(e))); }}
          onPointerCancel={() => { drag.current = null; setRect(null); }}
          onPointerUp={e => { if (e.isPrimary && drag.current?.frame === frame) selectBox(corners(...point(e)), frame); }}>
          <i className="shop-cursor" style={{ left: `${cursor[0]*100}%`, top: `${cursor[1]*100}%` }} />
          {rect && <span className="shop-box" style={{ left: `${rect[0]*100}%`, top: `${rect[1]*100}%`, width: `${(rect[2]-rect[0])*100}%`, height: `${(rect[3]-rect[1])*100}%` }} />}
        </div>}
      </div>
    </div> : <div className="split-visual"><div className="status-icon"><Icon name={terminal ? 'check' : 'follow'} /></div></div>}
    <div className="shop-controls">
      <h1 aria-live="polite">{view.title}</h1>
      <p className="lead">{view.lead}</p>
      {(offline || status?.stale) && <p className="shop-error" role="alert">최신 상태를 확인하지 못했습니다. 마지막 상태를 표시합니다. 카트 움직임은 안내 직원에게 확인해 주세요.</p>}
      {registration?.registered && <p>추종 대상이 등록되었습니다.</p>}
      {selectable && !manual && <div className="shop-numbers" aria-label="사진 속 사람 선택">
        {registration?.candidates?.map((candidate, index) => <button key={candidate.candidateId} className="btn btn-primary"
          onClick={() => command('select', { frameRef: frame, candidateId: candidate.candidateId })}>{index + 1}번 선택</button>)}
      </div>}
      {message && <p className="shop-error" role="status">{message}</p>}
      <div className="actions">
        {status?.allowedActions.includes('capture') && <button className="btn btn-primary" disabled={disabled('capture')} onClick={() => command('capture')}>{frame ? '다시 찍기' : '내 모습 찾기'}</button>}
        {selectable && <button className="btn btn-secondary" onClick={() => { setManual(!manual); setRect(null); drag.current = null; }}>{manual ? '번호로 선택' : '직접 선택'}</button>}
        {status?.allowedActions.includes('release') && <button className="btn btn-secondary" disabled={disabled('release')} onClick={() => command('release')}>대상 등록 해제</button>}
        {status?.allowedActions.includes('pause') && <button className="btn btn-stop btn-block" disabled={disabled('pause') || pending?.operation === 'pause'} onClick={() => command('pause')}><Icon name="pause" />잠시 멈추기</button>}
        {status?.allowedActions.includes('resume') && <button className="btn btn-primary" disabled={disabled('resume')} onClick={() => command('resume')}>추종 재개</button>}
        {status?.allowedActions.includes('return') && <button className="btn btn-secondary" disabled={disabled('return')} onClick={() => command('return')}>물품을 모두 내렸습니다 · 복귀 요청</button>}
        {retry && <button className="btn btn-secondary" disabled={Boolean(pending)} onClick={() => submit(retry)}>같은 요청 결과 다시 확인</button>}
        {terminal && <button className="btn btn-secondary" onClick={onClose}>화면 닫기</button>}
      </div>
    </div>
  </section>;
}
