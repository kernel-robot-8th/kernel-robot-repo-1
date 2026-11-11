import { useEffect, useState, type ReactNode } from 'react';
import * as api from './api';
import { Icon, PhoneQr, type IconName } from './icons';
import type { ServiceMode, VerifiedPickup, VerifyFailure, VerifyResult } from './types';

// 패드 화면 단계. 대기 → 인증 → 물품 확인 → (입국장이면 서비스 선택·이동) → 수령 확인 → 복귀
type Step =
  | { name: 'idle' }
  | { name: 'keypad' }
  | { name: 'verifying' }
  | { name: 'verifyFailed'; reason: VerifyFailure | 'ERROR' }
  | { name: 'items'; pickup: VerifiedPickup }
  | { name: 'chooseService'; pickup: VerifiedPickup }
  | { name: 'moving'; pickup: VerifiedPickup; mode: ServiceMode; paused: boolean; arrived: boolean }
  | { name: 'receipt'; pickup: VerifiedPickup }
  | { name: 'returning' };

const FAILURE_TEXT: Record<VerifyFailure | 'ERROR', string> = {
  NOT_FOUND: '예약을 찾을 수 없습니다. 예약번호를 다시 확인해 주세요.',
  WRONG_CART: '이 카트에 배정된 예약이 아닙니다. 안내 직원에게 문의해 주세요.',
  ALREADY_RECEIVED: '이미 수령이 완료된 예약입니다.',
  ERROR: '서버와 통신하지 못했습니다. 잠시 후 다시 시도해 주세요.',
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

export function App() {
  const [step, setStep] = useState<Step>({ name: 'idle' });
  const [busy, setBusy] = useState(false);
  const [notice, setNotice] = useState<string | null>(null);
  const [now, setNow] = useState(() => Date.now());
  const [lastPickup, setLastPickup] = useState<VerifiedPickup | null>(null);

  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 30_000);
    return () => clearInterval(timer);
  }, []);

  const pickup = pickupOf(step);
  useEffect(() => {
    if (pickup) setLastPickup(pickup);
  }, [pickup]);

  async function run<T>(task: () => Promise<T>): Promise<T | null> {
    if (busy) return null;
    setBusy(true);
    setNotice(null);
    try {
      return await task();
    } catch {
      setNotice(FAILURE_TEXT.ERROR);
      return null;
    } finally {
      setBusy(false);
    }
  }

  async function verifyWith(call: () => Promise<VerifyResult>) {
    if (busy) return;
    setStep({ name: 'verifying' });
    const result = await run(call);
    if (!result) setStep({ name: 'verifyFailed', reason: 'ERROR' });
    else if (result.ok) setStep({ name: 'items', pickup: result.pickup });
    else setStep({ name: 'verifyFailed', reason: result.reason });
  }

  async function startService(target: VerifiedPickup, mode: ServiceMode) {
    const res = await run(() => api.requestService(target.pickupId, mode));
    if (res?.accepted) setStep({ name: 'moving', pickup: target, mode, paused: false, arrived: false });
    else if (res) setNotice('요청이 거절되었습니다. 안내 직원에게 문의해 주세요.');
  }

  async function togglePause(current: Extract<Step, { name: 'moving' }>) {
    const call = current.paused ? api.requestResume : api.requestPause;
    const res = await run(() => call(current.pickup.pickupId));
    if (res?.accepted) {
      setStep({ ...current, paused: !current.paused });
      setNotice(`${current.paused ? '재개' : '중지'} 요청이 접수되었습니다. 카트 상태가 바뀔 때까지 기다려 주세요.`);
    }
  }

  async function confirm(target: VerifiedPickup) {
    const res = await run(() => api.confirmReceipt(target.pickupId));
    if (res?.accepted) setStep({ name: 'returning' });
    else if (res) setNotice('수령 확인이 저장되지 않았습니다. 다시 눌러 주세요.');
  }

  function reset() {
    api.resetDemo();
    setNotice(null);
    setLastPickup(null);
    setStep({ name: 'idle' });
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
            <button className="btn btn-secondary" onClick={() => setStep({ name: 'keypad' })}>
              <Icon name="keyboard" /> QR이 인식되지 않으면 예약번호 입력
            </button>
          </Split>
        )}

        {step.name === 'keypad' && (
          <Keypad busy={busy} onCancel={() => setStep({ name: 'idle' })} onSubmit={(no) => verifyWith(() => api.verifyReservationNo(no))} />
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
              <button className="btn btn-primary" onClick={() => setStep({ name: 'keypad' })}>예약번호 입력</button>
              <button className="btn btn-secondary" onClick={() => setStep({ name: 'idle' })}>처음으로</button>
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
              <button className="btn btn-primary btn-block" onClick={() => setStep({ name: 'chooseService', pickup: step.pickup })}>
                물품을 확인했습니다
              </button>
            ) : (
              <button className="btn btn-primary btn-block" onClick={() => setStep({ name: 'receipt', pickup: step.pickup })}>
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

        {step.name === 'moving' && (
          <Split
            visual={
              <>
                <StatusIcon
                  name={step.arrived ? 'check' : step.paused ? 'pause' : step.mode === 'FOLLOW' ? 'follow' : 'gate'}
                  tone={step.paused ? 'warn' : undefined}
                />
                <p className={`state-label ${step.paused ? 'warn' : ''}`}>
                  {step.arrived ? '도착' : step.paused ? '일시 중지' : `${MODE_LABEL[step.mode]} 중`}
                </p>
              </>
            }
          >
            <h1>
              {step.arrived ? `${api.gateLabel(step.pickup.flight.gateCode)}에 도착했습니다`
                : step.paused ? '카트를 멈추고 있습니다'
                : step.mode === 'FOLLOW' ? '카드를 들고 천천히 걸어 주세요'
                : `${api.gateLabel(step.pickup.flight.gateCode)}으로 안내합니다`}
            </h1>
            <p className="lead">
              {step.arrived ? '카트에서 물품을 모두 꺼낸 뒤 아래 버튼을 눌러 주세요.'
                : step.paused ? '카트가 완전히 멈출 때까지 가까이 가지 마세요.'
                : step.mode === 'FOLLOW' ? '마커 카드가 카트 쪽을 향하도록 들어 주세요.'
                : '카트를 따라 이동해 주세요.'}
            </p>
            {step.arrived ? (
              <button className="btn btn-primary btn-block" onClick={() => setStep({ name: 'receipt', pickup: step.pickup })}>
                물품을 꺼냈습니다
              </button>
            ) : (
              <div className="actions">
                <button className={`btn btn-big ${step.paused ? 'btn-primary' : 'btn-stop'}`} disabled={busy} onClick={() => togglePause(step)}>
                  <Icon name={step.paused ? 'play' : 'pause'} />
                  {step.paused ? '재개' : '중지'}
                </button>
                {step.mode === 'FOLLOW' && (
                  <>
                    <button className="btn btn-secondary" disabled={busy} onClick={() => startService(step.pickup, 'GATE_GUIDE')}>
                      게이트 안내로 전환
                    </button>
                    <button className="btn btn-secondary" onClick={() => setStep({ name: 'receipt', pickup: step.pickup })}>
                      추종 종료
                    </button>
                  </>
                )}
              </div>
            )}
          </Split>
        )}

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

        {notice && <p className="notice" role="status">{notice}</p>}
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
          {step.name === 'moving' && step.mode === 'GATE_GUIDE' && !step.arrived && (
            <button className="btn-mini" onClick={() => setStep({ ...step, arrived: true })}>게이트 도착</button>
          )}
          {step.name === 'returning' && (
            <button className="btn-mini" onClick={() => setStep({ name: 'idle' })}>복귀 완료</button>
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
