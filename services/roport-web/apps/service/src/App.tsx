import { useCallback, useEffect, useRef, useState, type FormEvent } from 'react';
import { toDataURL } from 'qrcode';
import { api, ApiError, fmt, loadErrorText as errText, loginErrorText, isActivePickupExists, qrErrorText, reserveErrorText, shouldReloadTimes, statusLabel, timesErrorText, upcomingFlights } from './api';
import type { Flight, Location, Member, Order, Pickup } from './api';
import { boardingNotice, mmss, progressIndex, PROGRESS_STEPS, qrDeadline, qrSecondsLeft, splitLocations, when } from './view';

type View =
  | { name: 'orders' }
  | { name: 'reserve'; order: Order }
  | { name: 'pickups' }
  | { name: 'ticket'; pickup: Pickup; justCreated?: boolean }
  | { name: 'status'; pickup: Pickup };
type Load<T> = { s: 'loading' } | { s: 'error'; msg: string } | { s: 'ok'; data: T };

function Logo() {
  return <span className="logo">RO<span className="logo-accent">:</span><span className="logo-port">PORT</span></span>;
}

export function App() {
  const [member, setMember] = useState<Member | null | undefined>(undefined); // undefined: 확인 중
  const [view, setView] = useState<View>({ name: 'orders' });
  const [sessionErr, setSessionErr] = useState('');

  useEffect(() => {
    api.me().then(setMember, (e) => {
      if (!(e instanceof ApiError && e.status === 401)) setSessionErr(errText(e));
      setMember(null);
    });
  }, []);

  // 어느 화면이든 401이면 로그인 화면으로
  const onError = useCallback((e: unknown) => {
    if (e instanceof ApiError && e.status === 401) {
      setSessionErr('세션이 만료되었습니다. 다시 로그인해 주세요.');
      setMember(null);
      return true;
    }
    return false;
  }, []);

  if (member === undefined) return <main className="app"><p className="status-line" role="status">확인 중…</p></main>;
  if (member === null) {
    return <Login err={sessionErr} onDone={(m) => { setSessionErr(''); setView({ name: 'orders' }); setMember(m); }} />;
  }

  const toPickups = () => setView({ name: 'pickups' });

  // 상세 화면(예약·QR·진행 상태)은 뒤로 가기 머리글을, 목록 화면은 로고·탭 머리글을 쓴다.
  if (view.name === 'reserve') {
    return (
      <main className="app">
        <SubHeader title="수령 예약" sub={`주문 ${view.order.orderNo} · ${view.order.items.length}개 상품`} onBack={() => setView({ name: 'orders' })} />
        <Reserve order={view.order} onError={onError} onCancel={() => setView({ name: 'orders' })} onDone={(p) => setView({ name: 'ticket', pickup: p, justCreated: true })} onShowPickups={toPickups} />
      </main>
    );
  }
  if (view.name === 'ticket') {
    return (
      <main className="app">
        <SubHeader title="예약 QR" sub={`주문 ${view.pickup.orderNo}`} onBack={toPickups} />
        <Ticket pickup={view.pickup} justCreated={view.justCreated} onError={onError} onStatus={() => setView({ name: 'status', pickup: view.pickup })} />
      </main>
    );
  }
  if (view.name === 'status') {
    return (
      <main className="app">
        <SubHeader title="진행 상태" sub={`예약 ${view.pickup.reservationNo} · ${view.pickup.location.name}`} onBack={toPickups} />
        <Status pickup={view.pickup} onTicket={() => setView({ name: 'ticket', pickup: view.pickup })} />
      </main>
    );
  }

  return (
    <main className="app">
      <header className="topbar">
        <Logo />
        <button type="button" className="btn-text" onClick={() => api.logout().catch(() => {}).then(() => setMember(null))}>로그아웃</button>
      </header>
      <nav className="tabs" aria-label="메뉴">
        <button type="button" aria-current={view.name === 'orders' ? 'page' : undefined} onClick={() => setView({ name: 'orders' })}>주문 목록</button>
        <button type="button" aria-current={view.name === 'pickups' ? 'page' : undefined} onClick={toPickups}>내 예약</button>
      </nav>
      <div className="content">
        <p className="dev-note">{member.displayName}님 · 개발 전용 로그인 (공개 배포 금지)</p>
        {view.name === 'orders' && <Orders member={member} onError={onError} onReserve={(order) => setView({ name: 'reserve', order })} onShowPickups={toPickups} />}
        {view.name === 'pickups' && <Pickups onError={onError} onTicket={(p) => setView({ name: 'ticket', pickup: p })} onStatus={(p) => setView({ name: 'status', pickup: p })} />}
      </div>
    </main>
  );
}

function SubHeader({ title, sub, onBack }: { title: string; sub: string; onBack: () => void }) {
  return (
    <header className="subbar">
      <button type="button" className="back" onClick={onBack} aria-label="뒤로">
        <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M15 5l-7 7 7 7" /></svg>
      </button>
      <div>
        <h1>{title}</h1>
        <p>{sub}</p>
      </div>
    </header>
  );
}

function Login({ err, onDone }: { err: string; onDone: (m: Member) => void }) {
  const [loginId, setLoginId] = useState('');
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState(err);

  async function submit(e: FormEvent) {
    e.preventDefault();
    if (busy || !loginId.trim()) return;
    setBusy(true);
    setMsg('');
    try {
      onDone(await api.login(loginId.trim()));
    } catch (x) {
      setMsg(loginErrorText(x));
      setBusy(false);
    }
  }

  return (
    <main className="app">
      <div className="login-hero">
        <h1><Logo /></h1>
        <p>면세품 픽업 예약</p>
      </div>
      <div className="content">
        <h2 className="headline">구매하신 면세품을<br />원하는 곳에서 받으세요</h2>
        <p className="muted">게이트 또는 입국장에서 스마트카트가 기다립니다.</p>
        <form className="form" onSubmit={submit}>
          <label className="field">
            <span>로그인 ID</span>
            <input value={loginId} onChange={(e) => setLoginId(e.target.value)} autoComplete="username" required />
          </label>
          <button type="submit" className="btn btn-primary btn-block" disabled={busy}>{busy ? '로그인 중…' : '로그인'}</button>
        </form>
        {msg && <p className="alert" role="alert">{msg}</p>}
        <p className="note">개발 전용 로그인입니다. 로그인 ID만 확인하며 공개 배포용이 아닙니다.</p>
      </div>
    </main>
  );
}

function useLoad<T>(fn: () => Promise<T>, onError: (e: unknown) => boolean): [Load<T>, () => void] {
  const [state, setState] = useState<Load<T>>({ s: 'loading' });
  const [n, setN] = useState(0);
  useEffect(() => {
    let live = true;
    setState({ s: 'loading' });
    fn().then(
      (data) => { if (live) setState({ s: 'ok', data }); },
      (e) => { if (live && !onError(e)) setState({ s: 'error', msg: errText(e) }); },
    );
    return () => { live = false; };
    // fn은 매 렌더 새로 만들어질 수 있어 재시도 카운터(n)로만 다시 불러온다.
  }, [n]); // eslint-disable-line
  return [state, () => setN((x) => x + 1)];
}

// 불러오는 중·실패 상태 공통 표시
function LoadState({ st, reload }: { st: Load<unknown>; reload: () => void }) {
  if (st.s === 'loading') return <p className="status-line" role="status">불러오는 중…</p>;
  if (st.s === 'error') return <p className="alert" role="alert">{st.msg} <button type="button" className="btn-text" onClick={reload}>다시 시도</button></p>;
  return null;
}

function Orders({ member, onError, onReserve, onShowPickups }: { member: Member; onError: (e: unknown) => boolean; onReserve: (o: Order) => void; onShowPickups: () => void }) {
  const [st, reload] = useLoad(api.orders, onError);
  return (
    <section aria-labelledby="orders-h">
      <h2 id="orders-h" className="headline">{member.displayName}님의 주문</h2>
      <LoadState st={st} reload={reload} />
      {st.s === 'ok' && st.data.length === 0 && <p className="empty">예약할 수 있는 주문이 없습니다.</p>}
      {st.s === 'ok' && (
        <ul className="cards">
          {st.data.map((o) => (
            <li key={o.orderId} className="card">
              <div className="card-head">
                <strong>{o.orderNo}</strong>
                <span className="muted small">{fmt(o.orderedAt)} 구매</span>
              </div>
              <ul className="item-list">
                {o.items.map((i) => <li key={i.itemId}><span>{i.productName}</span><b>{i.quantity}개</b></li>)}
              </ul>
              {o.activePickup ? (
                <div className="reserved">
                  <div>
                    <span className="muted small">예약번호</span>
                    <strong className="mono">{o.activePickup.reservationNo}</strong>
                  </div>
                  <span className="tag">{statusLabel(o.activePickup.status)}</span>
                  <button type="button" className="btn-text" onClick={onShowPickups}>내 예약에서 보기</button>
                </div>
              ) : (
                <button type="button" className="btn btn-primary btn-block" onClick={() => onReserve(o)}>수령 예약하기</button>
              )}
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}

// 수령 위치 간략 지도: 게이트 줄, 통로, 나머지 위치 줄. 라디오 버튼이라 키보드(화살표)로도 고를 수 있다.
function LocationMap({ locations, value, onChange }: { locations: Location[]; value: string; onChange: (id: string) => void }) {
  const { gates, others } = splitLocations(locations);
  const tile = (l: Location) => (
    <label key={l.locationId} className={`map-tile${value === l.locationId ? ' on' : ''}`}>
      <input type="radio" name="location" value={l.locationId} checked={value === l.locationId} onChange={() => onChange(l.locationId)} required />
      <span>{l.name}</span>
    </label>
  );
  return (
    <div className="map">
      {gates.length > 0 && <div className="map-row">{gates.map(tile)}</div>}
      {gates.length > 0 && others.length > 0 && <div className="map-corridor" aria-hidden="true" />}
      {others.length > 0 && <div className="map-row">{others.map(tile)}</div>}
      <span className="map-caption" aria-hidden="true">간략 지도</span>
    </div>
  );
}

function Reserve({ order, onError, onCancel, onDone, onShowPickups }: { order: Order; onError: (e: unknown) => boolean; onCancel: () => void; onDone: (p: Pickup) => void; onShowPickups: () => void }) {
  const [opts, reloadOpts] = useLoad<{ locations: Location[]; flights: Flight[] }>(
    () => Promise.all([api.locations(), api.flights()]).then(([locations, flights]) => ({ locations, flights: upcomingFlights(flights) })),
    onError,
  );
  const [locationId, setLocationId] = useState('');
  const [flightId, setFlightId] = useState('');
  const [pickupAt, setPickupAt] = useState('');
  const [times, setTimes] = useState<Load<string[]> | null>(null);
  const [timesN, setTimesN] = useState(0); // 시간 목록 재조회 카운터
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState('');
  const [showList, setShowList] = useState(false); // 이미 접수된 예약 → 예약 조회 안내 버튼

  const change = (fn: () => void) => { fn(); setErr(''); setShowList(false); };

  useEffect(() => {
    setPickupAt('');
    if (!locationId || !flightId) { setTimes(null); return; }
    let live = true;
    setTimes({ s: 'loading' });
    api.times(order.orderId, flightId, locationId).then(
      (r) => { if (live) setTimes({ s: 'ok', data: r.times }); },
      (e) => { if (live && !onError(e)) setTimes({ s: 'error', msg: timesErrorText(e) }); },
    );
    return () => { live = false; };
  }, [order.orderId, flightId, locationId, onError, timesN]);

  async function submit(e: FormEvent) {
    e.preventDefault();
    if (busy || !locationId || !flightId || !pickupAt) return;
    setBusy(true);
    setErr('');
    setShowList(false);
    try {
      onDone(await api.reserve({ orderId: order.orderId, locationId, flightId, pickupAt }));
    } catch (x) {
      if (!onError(x)) { setErr(reserveErrorText(x)); setShowList(isActivePickupExists(x)); }
      if (shouldReloadTimes(x)) setTimesN((n) => n + 1);
      setBusy(false);
    }
  }

  const locName = opts.s === 'ok' ? opts.data.locations.find((l) => l.locationId === locationId)?.name : undefined;

  return (
    <section className="content" aria-label={`수령 예약 · ${order.orderNo}`}>
      <LoadState st={opts} reload={reloadOpts} />
      {opts.s === 'ok' && (
        <form className="form" onSubmit={submit}>
          <fieldset className="step">
            <legend>1. 수령 위치</legend>
            {opts.data.locations.length === 0 ? <p className="empty">예약할 수 있는 수령 위치가 없습니다.</p> : (
              <LocationMap locations={opts.data.locations} value={locationId} onChange={(id) => change(() => setLocationId(id))} />
            )}
          </fieldset>

          <div className="step">
            <label className="field">
              <span className="step-title">2. 탑승 항공편</span>
              <select value={flightId} onChange={(e) => change(() => setFlightId(e.target.value))} required>
                <option value="">선택</option>
                {opts.data.flights.map((f) => <option key={f.flightId} value={f.flightId}>{f.flightNo} · 게이트 {f.gateCode} · 탑승 {when(f.boardingStart)}</option>)}
              </select>
            </label>
            {opts.data.flights.length === 0 && <p className="empty">탑승 전인 항공편이 없습니다.</p>}
          </div>

          <fieldset className="step" disabled={times?.s !== 'ok'}>
            <legend>3. 수령 시간</legend>
            {!times && <p className="muted small">위치와 항공편을 먼저 선택해 주세요.</p>}
            {times?.s === 'loading' && <p className="status-line" role="status">가능한 시간을 불러오는 중…</p>}
            {times?.s === 'error' && <p className="alert" role="alert">{times.msg}</p>}
            {times?.s === 'ok' && times.data.length === 0 && <p className="empty">선택한 조건에서 예약 가능한 시간이 없습니다.</p>}
            {times?.s === 'ok' && times.data.length > 0 && (
              <div className="chips">
                {times.data.map((t) => (
                  <label key={t} className={`chip${pickupAt === t ? ' on' : ''}`} title={fmt(t)}>
                    <input type="radio" name="pickupAt" value={t} checked={pickupAt === t} onChange={() => change(() => setPickupAt(t))} required />
                    <span>{when(t)}</span>
                  </label>
                ))}
              </div>
            )}
          </fieldset>

          {err && <p className="alert" role="alert">{err}{showList && <>{' '}<button type="button" className="btn-text" onClick={onShowPickups}>예약 조회로 이동</button></>}</p>}

          <div className="sticky-actions">
            <button type="submit" className="btn btn-primary btn-block" disabled={busy || !pickupAt}>
              {busy ? '접수 중…' : pickupAt && locName ? `${locName} · ${when(pickupAt)} 예약하기` : '예약하기'}
            </button>
            <button type="button" className="btn btn-secondary btn-block" onClick={onCancel} disabled={busy}>취소</button>
          </div>
        </form>
      )}
    </section>
  );
}

function Pickups({ onError, onTicket, onStatus }: { onError: (e: unknown) => boolean; onTicket: (p: Pickup) => void; onStatus: (p: Pickup) => void }) {
  const [st, reload] = useLoad(api.pickups, onError);
  return (
    <section aria-labelledby="pickups-h">
      <h2 id="pickups-h" className="headline">내 예약</h2>
      <LoadState st={st} reload={reload} />
      {st.s === 'ok' && st.data.length === 0 && <p className="empty">예약 내역이 없습니다.</p>}
      {st.s === 'ok' && (
        <ul className="cards">
          {st.data.map((p) => (
            <li key={p.pickupId} className="card">
              <div className="card-head">
                <strong className="mono">{p.reservationNo}</strong>
                <span className="tag">{statusLabel(p.status)}</span>
              </div>
              <dl className="facts">
                <div><dt>수령 위치</dt><dd>{p.location.name}</dd></div>
                <div><dt>수령 희망</dt><dd>{when(p.pickupAt)}</dd></div>
                <div><dt>항공편</dt><dd>{p.flight.flightNo}</dd></div>
                <div><dt>주문</dt><dd>{p.orderNo}</dd></div>
              </dl>
              <div className="row-actions">
                <button type="button" className="btn btn-primary" onClick={() => onTicket(p)}>예약 QR</button>
                <button type="button" className="btn btn-secondary" onClick={() => onStatus(p)}>진행 상태</button>
              </div>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}

type QrState =
  | { st: 'loading' }
  | { st: 'ready'; image: string; deadline: number }
  | { st: 'expired' }
  | { st: 'error'; msg: string };

// 예약 QR(SR-18-1). 화면에 들어올 때 서버에서 새 QR을 받아 브라우저에서 그린다(토큰을 외부로 보내지 않음).
// 발급할 때마다 이전 QR이 폐기되므로 자동으로 다시 받지 않고, 만료되면 고객이 "새 QR 받기"를 누른다.
// QR이 인식되지 않을 때를 위해 카트 패드에 입력할 예약번호를 함께 크게 보여 준다(SR-18-2-E).
function Ticket({ pickup: p, justCreated, onError, onStatus }: { pickup: Pickup; justCreated?: boolean; onError: (e: unknown) => boolean; onStatus: () => void }) {
  const [qr, setQr] = useState<QrState>({ st: 'loading' });
  const [now, setNow] = useState(() => performance.now()); // 단조 시계(ms): 휴대폰 시계를 바꿔도 영향 없음
  const seq = useRef(0);

  const issue = useCallback(async () => {
    const mine = ++seq.current; // 늦게 온 이전 발급 결과가 새 QR을 덮지 않게 한다
    setQr({ st: 'loading' });
    try {
      const requestedAt = performance.now(); // 응답 지연이 유효 기간을 늘리지 않게 요청 시점부터 센다
      const t = await api.qrToken(p.pickupId);
      const deadline = qrDeadline(t.issuedAt, t.expiresAt, requestedAt);
      // 카메라 인식에 필요한 사방 4모듈 흰 여백(QR 표준 quiet zone). 이미지 테두리·배경은 여백으로 치지 않는다.
      const image = await toDataURL(t.token, { errorCorrectionLevel: 'M', margin: 4, width: 320 });
      if (mine !== seq.current) return;
      setNow(performance.now());
      setQr({ st: 'ready', image, deadline });
    } catch (e) {
      if (mine !== seq.current || onError(e)) return;
      setQr({ st: 'error', msg: qrErrorText(e) });
    }
  }, [p.pickupId, onError]);

  useEffect(() => {
    void issue();
    return () => { seq.current++; }; // 화면을 떠나면 진행 중인 발급 결과를 버린다
  }, [issue]);

  const left = qr.st === 'ready' ? qrSecondsLeft(qr.deadline, now) : 0;
  useEffect(() => {
    if (qr.st !== 'ready') return;
    const id = setInterval(() => setNow(performance.now()), 1000);
    return () => clearInterval(id);
  }, [qr.st]);
  useEffect(() => {
    if (qr.st === 'ready' && left === 0) setQr({ st: 'expired' }); // 만료된 QR은 카트가 거절하므로 가린다
  }, [qr.st, left]);

  return (
    <section className="content">
      {justCreated && (
        <div className="lead-block" role="status">
          <h2>예약이 접수되었습니다.</h2>
          <p>현장 배정 전이며 수령이 확정된 것은 아닙니다. 수령 위치에서 카트에 아래 QR을 보여 주세요. 인식되지 않으면 카트 패드에 예약번호를 입력해 주세요.</p>
        </div>
      )}
      <article className="ticket">
        <header className="ticket-head">
          <span>면세품 픽업</span>
          <span>주문 {p.orderNo}</span>
        </header>
        <div className="ticket-body">
          {qr.st === 'ready' ? (
            <>
              <img className="qr-img" src={qr.image} alt={`예약 ${p.reservationNo} QR 코드`} width={192} height={192} />
              <p className="muted small" aria-live="polite">{mmss(left)} 후 만료 · 만료되면 새 QR을 받아 주세요</p>
            </>
          ) : (
            <div className="qr-slot" role={qr.st === 'error' ? 'alert' : 'status'}>
              {qr.st === 'loading' && <span>QR 받는 중…</span>}
              {qr.st === 'expired' && <><span>QR이 만료되었습니다</span><small>새 QR을 받으면 이전 QR은 쓸 수 없습니다</small></>}
              {qr.st === 'error' && <><span>QR을 받지 못했습니다</span><small>{qr.msg}</small></>}
              {qr.st !== 'loading' && <button type="button" className="btn btn-primary btn-small" onClick={() => void issue()}>새 QR 받기</button>}
            </div>
          )}
          <p className="muted small">예약번호</p>
          <p className="resv-no mono">{p.reservationNo}</p>
        </div>
        <dl className="facts ticket-facts">
          <div><dt>수령 위치</dt><dd>{p.location.name}</dd></div>
          <div><dt>수령 시간</dt><dd>{when(p.pickupAt)}</dd></div>
          <div><dt>항공편</dt><dd>{p.flight.flightNo}</dd></div>
          <div><dt>탑승 시작</dt><dd>{when(p.flight.boardingStart)}</dd></div>
        </dl>
      </article>
      <div className="foot-row">
        <span className="muted small">{statusLabel(p.status)}</span>
        <button type="button" className="btn-text" onClick={onStatus}>진행 상태 보기</button>
      </div>
    </section>
  );
}

// 진행 상태(SR-06). 서버 계약에는 접수 상태만 있어 이후 단계는 "예정"으로 그리고, 관제 연동 전임을 밝힌다.
function Status({ pickup: p, onTicket }: { pickup: Pickup; onTicket: () => void }) {
  const cur = progressIndex(p.status);
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const timer = setInterval(() => setNow(new Date()), 30_000); // 탑승 알림 남은 시간 갱신
    return () => clearInterval(timer);
  }, []);
  const notice = boardingNotice(p.flight, now);
  const steps = PROGRESS_STEPS.map((label, i) => {
    const state = i < cur ? 'done' : i === cur ? 'done current' : 'todo';
    return (
      <li key={label} className={state}>
        <span className="dot" aria-hidden="true" />
        <span className="step-label">{i === 3 ? `${p.location.name} 도착` : label}</span>
        <span className="step-time">{i === 0 && cur >= 0 ? when(p.createdAt) : i > cur ? '예정' : ''}</span>
      </li>
    );
  });
  return (
    <section className="content">
      <div className="hero-card">
        <p className="hero-kicker">예약 {p.reservationNo}</p>
        <h2>{cur >= 0 ? '예약이 접수되었습니다' : '상태를 확인할 수 없습니다'}</h2>
        <p>{statusLabel(p.status)}</p>
      </div>
      <ol className="timeline" aria-label="진행 단계">{steps}</ol>
      <p className="note">카트 배정 이후 단계는 관제 연동 후 실시간으로 표시됩니다. 지금은 연결 전입니다.</p>
      <div className={`boarding ${notice.tone}`} role="status">
        <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M10.5 21l2-1.5V14l8 2.5V14.5l-8-5V4a1.5 1.5 0 0 0-3 0v5.5l-8 5V16.5l8-2.5v5.5l-2 1.5" /></svg>
        <div>
          <b>{p.flight.flightNo} 탑승 {when(p.flight.boardingStart)} · 게이트 {p.flight.gateCode}</b>
          <p>{notice.text}</p>
        </div>
      </div>
      <button type="button" className="btn btn-secondary btn-block" onClick={onTicket}>예약 QR 다시 보기</button>
    </section>
  );
}
