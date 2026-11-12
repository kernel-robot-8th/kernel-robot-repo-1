import { useCallback, useEffect, useState, type FormEvent } from 'react';
import { api, ApiError, fmt, loadErrorText as errText, loginErrorText, isActivePickupExists, reserveErrorText, shouldReloadTimes, statusLabel, timesErrorText, upcomingFlights } from './api';
import type { Flight, Location, Member, Order, Pickup } from './api';

type View = { name: 'orders' } | { name: 'reserve'; order: Order } | { name: 'pickups'; justCreated?: string };
type Load<T> = { s: 'loading' } | { s: 'error'; msg: string } | { s: 'ok'; data: T };

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

  if (member === undefined) return <main><p role="status">확인 중…</p></main>;
  if (member === null) {
    return <Login err={sessionErr} onDone={(m) => { setSessionErr(''); setView({ name: 'orders' }); setMember(m); }} />;
  }

  return (
    <main>
      <header>
        <h1>RO:PORT 고객 서비스</h1>
        <p>{member.displayName}님 · 개발 전용 로그인 (공개 배포 금지)</p>
        <nav aria-label="메뉴">
          <button type="button" onClick={() => setView({ name: 'orders' })}>주문 목록</button>{' '}
          <button type="button" onClick={() => setView({ name: 'pickups' })}>예약 조회</button>{' '}
          <button type="button" onClick={() => api.logout().catch(() => {}).then(() => setMember(null))}>로그아웃</button>
        </nav>
      </header>
      {view.name === 'orders' && <Orders onError={onError} onReserve={(order) => setView({ name: 'reserve', order })} />}
      {view.name === 'reserve' && (
        <Reserve order={view.order} onError={onError} onCancel={() => setView({ name: 'orders' })} onDone={(p) => setView({ name: 'pickups', justCreated: p.pickupId })} onShowPickups={() => setView({ name: 'pickups' })} />
      )}
      {view.name === 'pickups' && <Pickups onError={onError} highlight={view.justCreated} />}
    </main>
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
    <main>
      <h1>RO:PORT 고객 서비스</h1>
      <p>개발 전용 로그인입니다. 로그인 ID만 확인하며 공개 배포용이 아닙니다.</p>
      <form onSubmit={submit}>
        <label>로그인 ID <input value={loginId} onChange={(e) => setLoginId(e.target.value)} autoComplete="username" required /></label>{' '}
        <button type="submit" disabled={busy}>{busy ? '로그인 중…' : '로그인'}</button>
      </form>
      {msg && <p role="alert">{msg}</p>}
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

function Orders({ onError, onReserve }: { onError: (e: unknown) => boolean; onReserve: (o: Order) => void }) {
  const [st, reload] = useLoad(api.orders, onError);
  return (
    <section aria-labelledby="orders-h">
      <h2 id="orders-h">주문 목록</h2>
      {st.s === 'loading' && <p role="status">불러오는 중…</p>}
      {st.s === 'error' && <p role="alert">{st.msg} <button type="button" onClick={reload}>다시 시도</button></p>}
      {st.s === 'ok' && st.data.length === 0 && <p>예약할 수 있는 주문이 없습니다.</p>}
      {st.s === 'ok' && (
        <ul>
          {st.data.map((o) => (
            <li key={o.orderId}>
              <strong>{o.orderNo}</strong> · {fmt(o.orderedAt)}
              <ul>{o.items.map((i) => <li key={i.itemId}>{i.productName} × {i.quantity}</li>)}</ul>
              {o.activePickup ? (
                <p>예약번호 {o.activePickup.reservationNo} ({statusLabel(o.activePickup.status)})</p>
              ) : (
                <button type="button" onClick={() => onReserve(o)}>수령 예약</button>
              )}
            </li>
          ))}
        </ul>
      )}
    </section>
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

  return (
    <section aria-labelledby="reserve-h">
      <h2 id="reserve-h">수령 예약 · {order.orderNo}</h2>
      {opts.s === 'loading' && <p role="status">불러오는 중…</p>}
      {opts.s === 'error' && <p role="alert">{opts.msg} <button type="button" onClick={reloadOpts}>다시 시도</button></p>}
      {opts.s === 'ok' && (
        <form onSubmit={submit}>
          <p>
            <label>수령 위치{' '}
              <select value={locationId} onChange={(e) => change(() => setLocationId(e.target.value))} required>
                <option value="">선택</option>
                {opts.data.locations.map((l) => <option key={l.locationId} value={l.locationId}>{l.name}</option>)}
              </select>
            </label>
          </p>
          <p>
            <label>항공편{' '}
              <select value={flightId} onChange={(e) => change(() => setFlightId(e.target.value))} required>
                <option value="">선택</option>
                {opts.data.flights.map((f) => <option key={f.flightId} value={f.flightId}>{f.flightNo} · 출발 {fmt(f.departureAt)} · 게이트 {f.gateCode}</option>)}
              </select>
            </label>
          </p>
          <p>
            <label>수령 시간{' '}
              <select value={pickupAt} onChange={(e) => change(() => setPickupAt(e.target.value))} required disabled={times?.s !== 'ok'}>
                <option value="">선택</option>
                {times?.s === 'ok' && times.data.map((t) => <option key={t} value={t}>{fmt(t)}</option>)}
              </select>
            </label>
          </p>
          {times?.s === 'loading' && <p role="status">가능한 시간을 불러오는 중…</p>}
          {times?.s === 'error' && <p role="alert">{times.msg}</p>}
          {times?.s === 'ok' && times.data.length === 0 && <p>선택한 조건에서 예약 가능한 시간이 없습니다.</p>}
          <button type="submit" disabled={busy || !pickupAt}>{busy ? '접수 중…' : '예약하기'}</button>{' '}
          <button type="button" onClick={onCancel} disabled={busy}>취소</button>
          {err && <p role="alert">{err}{showList && <>{' '}<button type="button" onClick={onShowPickups}>예약 조회로 이동</button></>}</p>}
        </form>
      )}
    </section>
  );
}

function Pickups({ onError, highlight }: { onError: (e: unknown) => boolean; highlight?: string }) {
  const [st, reload] = useLoad(api.pickups, onError);
  return (
    <section aria-labelledby="pickups-h">
      <h2 id="pickups-h">예약 조회</h2>
      {highlight && <p role="status">예약 접수됨 – 현장 배정 전입니다. 수령이 확정된 것이 아닙니다.</p>}
      {st.s === 'loading' && <p role="status">불러오는 중…</p>}
      {st.s === 'error' && <p role="alert">{st.msg} <button type="button" onClick={reload}>다시 시도</button></p>}
      {st.s === 'ok' && st.data.length === 0 && <p>예약 내역이 없습니다.</p>}
      {st.s === 'ok' && (
        <ul>
          {st.data.map((p) => (
            <li key={p.pickupId}>
              <strong>예약번호 {p.reservationNo}</strong> · 주문 {p.orderNo}
              <br />상태: {statusLabel(p.status)}
              <br />위치 {p.location.name} · 항공편 {p.flight.flightNo} · 수령 희망 {fmt(p.pickupAt)}
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
