import { useEffect, useReducer, useRef, useState, type ReactNode } from 'react';
import * as api from './api';
import { bigButton, initialState, movingPhase, reduce, type MovingPhase, type Notice, type Step } from './flow';
import { Icon, PhoneQr, type IconName } from './icons';
import type { ServiceMode, VerifiedPickup, VerifyFailure, VerifyResult } from './types';

const FAILURE_TEXT: Record<VerifyFailure | 'ERROR', string> = {
  NOT_FOUND: '예약을 찾을 수 없습니다. 예약번호를 다시 확인해 주세요.',
  WRONG_CART: '이 카트에 배정된 예약이 아닙니다. 안내 직원에게 문의해 주세요.',
  ALREADY_RECEIVED: '이미 수령이 완료된 예약입니다.',
  ERROR: '서버와 통신하지 못했습니다. 잠시 후 다시 시도해 주세요.',
};

const NOTICE_TEXT: Record<Exclude<Notice, null>, string> = {
  ERROR: FAILURE_TEXT.ERROR,
  SERVICE_REJECTED: '요청이 거절되었습니다. 안내 직원에게 문의해 주세요.',
  COMMAND_REJECTED: '요청이 거절되었습니다. 카트에서 떨어져 안내 직원에게 문의해 주세요.',
  RECEIPT_NOT_SAVED: '수령 확인이 저장되지 않았습니다. 다시 눌러 주세요.',
};

const MODE_LABEL: Record<ServiceMode, string> = { GATE_GUIDE: '게이트 안내', FOLLOW: '짐 운반·추종' };

function formatTime(iso: string): string {
  return new Date(iso).toLocaleTimeString('ko-KR', { timeZone: 'Asia/Seoul', hour: '2-digit', minute: '2-digit', hour12: false });
}

function minutesUntil(iso: string, now: number): number {
  return Math.round((new Date(iso).getTime() - now) / 60_000);
}

function pickupOf(step: Step): VerifiedPickup | null {
  return 'pickup' in step ? step.pickup : null;
}

// 진행 표시줄: 입국장 수령만 서비스 단계가 있다.
function progressOf(step: Step, arrival: boolean): { labels: string[]; current: number } {
  const labels = arrival ? ['예약 확인', '물품 확인', '서비스 이용', '수령 완료'] : ['예약 확인', '물품 확인', '수령 완료'];
  const order: Partial<Record<Step['name'], number>> = arrival
    ? { items: 1, chooseService: 2, moving: 2, receipt: 3, returning: 4 }
    : { items: 1, receipt: 2, returning: 3 };
  return { labels, current: order[step.name] ?? 0 };
}

// 이동 화면 문구. 왼쪽 상태 표시와 오른쪽 제목이 같은 단계 값에서 나온다.
function movingText(phase: MovingPhase, mode: ServiceMode, gate: string): { label: string; title: string; lead: string; icon: IconName; warn: boolean } {
  switch (phase) {
    case 'START_PENDING':
      return { label: '출발 확인 중', title: '출발 요청을 보냈습니다', lead: '카트가 움직이기 시작하면 화면이 바뀝니다. 카트 앞을 비워 주세요.', icon: mode === 'FOLLOW' ? 'follow' : 'gate', warn: false };
    case 'MOVING':
      return mode === 'FOLLOW'
        ? { label: `${MODE_LABEL[mode]} 중`, title: '카드를 들고 천천히 걸어 주세요', lead: '마커 카드가 카트 쪽을 향하도록 들어 주세요. 추종을 끝내려면 먼저 중지해 주세요.', icon: 'follow', warn: false }
        : { label: `${MODE_LABEL[mode]} 중`, title: `${gate}으로 안내합니다`, lead: '카트를 따라 이동해 주세요.', icon: 'gate', warn: false };
    case 'PAUSE_PENDING':
      return { label: '중지 확인 중', title: '중지 요청을 보냈습니다', lead: '카트가 완전히 멈출 때까지 가까이 가지 마세요.', icon: 'pause', warn: true };
    case 'PAUSED':
      return { label: '일시 중지', title: '카트가 멈췄습니다', lead: mode === 'FOLLOW' ? '다시 출발하려면 재개를, 추종을 끝내려면 추종 종료를 눌러 주세요.' : '다시 출발하려면 재개를 눌러 주세요.', icon: 'pause', warn: true };
    case 'RESUME_PENDING':
      return { label: '재개 확인 중', title: '재개 요청을 보냈습니다', lead: '카트가 다시 움직이기 시작하면 화면이 바뀝니다. 카트 앞을 비워 주세요.', icon: 'play', warn: true };
    case 'ARRIVED':
      return { label: '도착', title: `${gate}에 도착했습니다`, lead: '카트에서 물품을 모두 꺼낸 뒤 아래 버튼을 눌러 주세요.', icon: 'check', warn: false };
  }
}

export function App() {
  const [state, dispatch] = useReducer(reduce, initialState);
  const { step, notice, lastPickup } = state;
  const busy = state.request !== null;
  const [now, setNow] = useState(() => Date.now());
  const nextToken = useRef(0);
  // 같은 렌더 안의 연속 터치로 요청이 두 번 나가지 않게 reduce의 요청 상태를 즉시 따라가는 값
  const inFlight = useRef<{ token: number; stop: boolean } | null>(null);

  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 30_000);
    return () => clearInterval(timer);
  }, []);

  const pickup = pickupOf(step);
  const go = (s: Step) => {
    inFlight.current = null;
    dispatch({ type: 'go', step: s });
  };

  // 요청을 보내고 결과를 토큰과 함께 넘긴다. 그사이 화면이 바뀌었으면 reduce가 결과를 버린다.
  // 중지(stop)만 다른 요청을 기다리는 중에도 보낼 수 있다(reduce의 send 규칙과 같다).
  async function send<T>(call: () => Promise<T>, done: (token: number, res: T | null) => void, waitStep?: Step, stop = false) {
    const cur = inFlight.current;
    if (cur && (!stop || cur.stop)) return;
    const token = ++nextToken.current;
    inFlight.current = { token, stop };
    dispatch({ type: 'send', token, step: waitStep, stop });
    let res: T | null = null;
    try {
      res = await call();
    } catch {
      res = null;
    }
    if (inFlight.current?.token === token) inFlight.current = null;
    done(token, res);
  }

  const verifyWith = (call: () => Promise<VerifyResult>) =>
    send(call, (token, result) => dispatch({ type: 'verified', token, result }), { name: 'verifying' });

  const startService = (target: VerifiedPickup, mode: ServiceMode) =>
    send(() => api.requestService(target.pickupId, mode), (token, res) => dispatch({ type: 'serviceAnswered', token, pickup: target, mode, res }));

  const sendCommand = (target: VerifiedPickup, command: 'PAUSE' | 'RESUME') =>
    send(
      () => (command === 'PAUSE' ? api.requestPause : api.requestResume)(target.pickupId),
      (token, res) => dispatch({ type: 'commandAnswered', token, command, res }),
      undefined,
      command === 'PAUSE',
    );

  const confirm = (target: VerifiedPickup) =>
    send(() => api.confirmReceipt(target.pickupId), (token, res) => dispatch({ type: 'receiptAnswered', token, res }));

  function reset() {
    api.resetDemo();
    inFlight.current = null;
    dispatch({ type: 'reset' });
  }

  const progressPickup = pickup ?? (step.name === 'returning' ? lastPickup : null);
  const progress = progressPickup ? progressOf(step, api.isArrivalHall(progressPickup.location)) : null;

  return (
    <div className="pad">
      <header className="topbar">
        <div className="brand">
          <span className="logo">RO<span className="logo-accent">:PORT</span></span>
          <span className="cart-id">{api.CART_LABEL}</span>
          {api.IS_DEMO && <span className="badge-demo" title="데모 데이터로 동작하며 로봇 명령을 보내지 않습니다">데모</span>}
        </div>
        {pickup && (
          <dl className="flight" aria-label="탑승 정보">
            <div><dt>항공편</dt><dd>{pickup.flight.flightNo}</dd></div>
            <div className="flight-gate"><dt>탑승 게이트</dt><dd>{api.gateLabel(pickup.flight.gateCode)}</dd></div>
            <div>
              <dt>탑승 시작</dt>
              <dd>{formatTime(pickup.flight.boardingStart)} <small>{minutesUntil(pickup.flight.boardingStart, now)}분 후</small></dd>
            </div>
            <div><dt>탑승 마감</dt><dd>{formatTime(pickup.flight.boardingEnd)}</dd></div>
          </dl>
        )}
      </header>

      {progress && (
        <ol className="progress" aria-label="진행 단계">
          {progress.labels.map((label, i) => (
            <li key={label} className={i < progress.current ? 'done' : i === progress.current ? 'current' : ''}>
              <span className="num">{i < progress.current ? <Icon name="check" /> : i + 1}</span>
              {label}
            </li>
          ))}
        </ol>
      )}

      <main className="stage">
        {step.name === 'idle' && (
          <Split visual={<PhoneQr />}>
            <h1>예약 QR을<br />카메라에 보여주세요</h1>
            <ol className="guide">
              <li><span>휴대폰에서 고객 웹의 <b>예약 화면</b>을 엽니다.</span></li>
              <li><span>QR을 카트의 <b>카메라</b>에 가까이 비춥니다.</span></li>
              <li><span>확인되면 이 화면에 주문하신 물품이 표시됩니다.</span></li>
            </ol>
            <button className="btn btn-secondary" onClick={() => go({ name: 'keypad' })}>
              <Icon name="keyboard" /> QR이 인식되지 않으면 예약번호 입력
            </button>
          </Split>
        )}

        {step.name === 'keypad' && (
          <Keypad busy={busy} onCancel={() => go({ name: 'idle' })} onSubmit={(no) => verifyWith(() => api.verifyReservationNo(no))} />
        )}

        {step.name === 'verifying' && (
          <Split visual={<div className="spinner" aria-hidden="true" />}>
            <h1 role="status">예약을 확인하고 있습니다</h1>
            <p className="lead">잠시만 기다려 주세요.</p>
          </Split>
        )}

        {step.name === 'verifyFailed' && (
          <Split visual={<StatusIcon name="alert" tone="warn" />}>
            <h1>예약을 확인하지 못했습니다</h1>
            <p className="lead" role="alert">{FAILURE_TEXT[step.reason]}</p>
            <div className="actions">
              <button className="btn btn-primary" onClick={() => go({ name: 'keypad' })}>예약번호 입력</button>
              <button className="btn btn-secondary" onClick={() => go({ name: 'idle' })}>처음으로</button>
            </div>
          </Split>
        )}

        {step.name === 'items' && (
          <Split visual={<><StatusIcon name="bag" /><p className="visual-note">포장 <b>{step.pickup.packageCount}개</b>가 카트에 실려 있습니다</p></>}>
            <h1>{step.pickup.customerName}님의 주문 물품</h1>
            <table className="items">
              <thead><tr><th>상품</th><th>수량</th></tr></thead>
              <tbody>
                {step.pickup.items.map((item) => (
                  <tr key={item.itemId}><td>{item.productName}</td><td>{item.quantity}개</td></tr>
                ))}
              </tbody>
            </table>
            <p className="meta">예약번호 {step.pickup.reservationNo} · 주문번호 {step.pickup.orderNo} · {step.pickup.location.name} 수령</p>
            {api.isArrivalHall(step.pickup.location) ? (
              <button className="btn btn-primary btn-block" onClick={() => go({ name: 'chooseService', pickup: step.pickup })}>
                물품을 확인했습니다
              </button>
            ) : (
              <button className="btn btn-primary btn-block" onClick={() => go({ name: 'receipt', pickup: step.pickup })}>
                물품을 꺼냈습니다
              </button>
            )}
          </Split>
        )}

        {step.name === 'chooseService' && (
          <div className="choose">
            <h1>이동 방법을 선택해 주세요</h1>
            <p className="lead">짐은 카트에 실은 채로 함께 이동합니다.</p>
            <div className="choices">
              <button className="choice" disabled={busy} onClick={() => startService(step.pickup, 'GATE_GUIDE')}>
                <span className="choice-icon"><Icon name="gate" /></span>
                <span className="choice-text">
                  <b>게이트 안내</b>
                  <span>카트가 앞에서 {api.gateLabel(step.pickup.flight.gateCode)}까지 길을 안내합니다.</span>
                </span>
                <span className="choice-arrow"><Icon name="arrow" /></span>
              </button>
              <button className="choice" disabled={busy} onClick={() => startService(step.pickup, 'FOLLOW')}>
                <span className="choice-icon"><Icon name="follow" /></span>
                <span className="choice-text">
                  <b>짐 운반·추종</b>
                  <span>마커 카드를 들고 걸으면 카트가 뒤따라옵니다. 면세점을 둘러보는 동안에도 쓸 수 있습니다.</span>
                </span>
                <span className="choice-arrow"><Icon name="arrow" /></span>
              </button>
            </div>
          </div>
        )}

        {step.name === 'moving' && (() => {
          const phase = movingPhase(step);
          const gate = api.gateLabel(step.pickup.flight.gateCode);
          const text = movingText(phase, step.mode, gate);
          // 재개 요청을 보내는 동안에는 카트가 움직이기 시작할 수 있어 큰 버튼을 중지로 바꾼다.
          const big = bigButton(phase) === 'RESUME' && busy ? 'PAUSE' : bigButton(phase);
          return (
            <Split
              visual={
                <>
                  <StatusIcon name={text.icon} tone={text.warn ? 'warn' : undefined} />
                  <p className={`state-label ${text.warn ? 'warn' : ''}`}>{text.label}</p>
                </>
              }
            >
              <h1 aria-live="polite">{text.title}</h1>
              <p className="lead">{text.lead}</p>
              {big ? (
                <div className="actions">
                  <button
                    className={`btn btn-big ${big === 'RESUME' ? 'btn-primary' : 'btn-stop'}`}
                    disabled={big === 'PAUSE' ? state.stopping : busy}
                    onClick={() => sendCommand(step.pickup, big)}
                  >
                    <Icon name={big === 'RESUME' ? 'play' : 'pause'} />
                    {big === 'RESUME' ? '재개' : '중지'}
                  </button>
                  {step.mode === 'FOLLOW' && (
                    <>
                      <button className="btn btn-secondary" disabled={busy || phase !== 'MOVING'} onClick={() => startService(step.pickup, 'GATE_GUIDE')}>
                        게이트 안내로 전환
                      </button>
                      {/* 카트가 멈춘 것이 확인된 뒤에만 추종을 끝낸다. */}
                      <button className="btn btn-secondary" disabled={busy || phase !== 'PAUSED'} onClick={() => go({ name: 'receipt', pickup: step.pickup })}>
                        추종 종료
                      </button>
                    </>
                  )}
                </div>
              ) : (
                <button className="btn btn-primary btn-block" onClick={() => go({ name: 'receipt', pickup: step.pickup })}>
                  물품을 꺼냈습니다
                </button>
              )}
            </Split>
          );
        })()}

        {step.name === 'receipt' && (
          <Split visual={<StatusIcon name="bag" />}>
            <h1>물품을 모두 받으셨나요?</h1>
            <p className="lead">수령을 확인하면 카트는 대기 장소로 돌아갑니다.</p>
            <button className="btn btn-primary btn-block" disabled={busy} onClick={() => confirm(step.pickup)}>
              {busy ? '저장 중…' : '수령 확인'}
            </button>
          </Split>
        )}

        {step.name === 'returning' && (
          <div className="finale">
            <StatusIcon name="check" tone="done" />
            <h1>수령이 완료되었습니다</h1>
            <p className="lead">이용해 주셔서 감사합니다. 카트는 대기 장소로 돌아갑니다.</p>
            <p className="tagline">로봇이 만드는 더 편리한 여행의 시작, RO:PORT</p>
          </div>
        )}

        {notice && <p className="notice" role="alert">{NOTICE_TEXT[notice]}</p>}
      </main>

      {api.IS_DEMO && (
        <footer className="demo-bar" aria-label="데모 조작">
          <span className="demo-label">데모 조작</span>
          {step.name === 'idle' && (
            <>
              {api.DEMO_QR.map(([label, no]) => (
                <button key={no} className="btn-mini" onClick={() => verifyWith(() => api.verifyQr(no))}>QR: {label}</button>
              ))}
              <button className="btn-mini" onClick={() => verifyWith(() => api.verifyQr('unknown'))}>잘못된 QR</button>
            </>
          )}
          {step.name === 'moving' && !step.arrived && (
            <>
              <span className="demo-label">모의 카트 상태</span>
              <button className="btn-mini" onClick={() => dispatch({ type: 'robotState', motion: 'MOVING' })}>주행 중</button>
              <button className="btn-mini" onClick={() => dispatch({ type: 'robotState', motion: 'PAUSED' })}>정지됨</button>
              {step.mode === 'GATE_GUIDE' && step.motion === 'MOVING' && (
                <button className="btn-mini" onClick={() => dispatch({ type: 'arrived' })}>게이트 도착</button>
              )}
            </>
          )}
          {step.name === 'returning' && (
            <button className="btn-mini" onClick={() => go({ name: 'idle' })}>복귀 완료</button>
          )}
          <button className="btn-mini" onClick={reset}>처음부터</button>
        </footer>
      )}
    </div>
  );
}

// 왼쪽 그림 영역 + 오른쪽 내용 영역
function Split({ visual, children }: { visual: ReactNode; children: ReactNode }) {
  return (
    <div className="split">
      <aside className="split-visual">{visual}</aside>
      <section className="split-body">{children}</section>
    </div>
  );
}

function StatusIcon({ name, tone }: { name: IconName; tone?: 'warn' | 'done' }) {
  return <div className={`status-icon ${tone ?? ''}`}><Icon name={name} /></div>;
}

// 예약번호는 서버 문자 집합(영문 대문자·숫자, 0·O·1·I 제외)만 받는다.
function Keypad({ busy, onSubmit, onCancel }: { busy: boolean; onSubmit: (no: string) => void; onCancel: () => void }) {
  const [value, setValue] = useState('');
  const full = value.length === api.RESERVATION_LENGTH;

  function append(chars: string) {
    const allowed = [...chars.toUpperCase()].filter((c) => api.RESERVATION_ALPHABET.includes(c)).join('');
    setValue((v) => (v + allowed).slice(0, api.RESERVATION_LENGTH));
  }

  return (
    <form className="keypad-screen" onSubmit={(e) => { e.preventDefault(); if (full) onSubmit(value); }}>
      <h1>예약번호 8자리를 입력해 주세요</h1>
      <p className="lead">고객 웹의 예약 조회 화면에서 확인할 수 있습니다.</p>
      <label className="slots">
        <input
          className="slots-input"
          aria-label="예약번호"
          autoCapitalize="characters"
          autoComplete="off"
          value={value}
          onChange={(e) => { setValue(''); append(e.target.value); }}
          autoFocus
        />
        {Array.from({ length: api.RESERVATION_LENGTH }, (_, i) => (
          <span key={i} className={`slot ${i === value.length ? 'active' : ''}`} aria-hidden="true">{value[i] ?? ''}</span>
        ))}
      </label>
      <div className="keypad">
        {[...api.RESERVATION_ALPHABET].map((c) => (
          <button key={c} type="button" className="key" disabled={busy || full} onClick={() => append(c)}>{c}</button>
        ))}
      </div>
      <div className="actions">
        <button type="button" className="btn btn-secondary" onClick={onCancel}>취소</button>
        <button type="button" className="btn btn-secondary" disabled={!value} onClick={() => setValue((v) => v.slice(0, -1))}>지우기</button>
        <button type="submit" className="btn btn-primary" disabled={busy || !full}>확인</button>
      </div>
    </form>
  );
}
