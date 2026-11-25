import { useEffect, useRef } from 'react';
import { Icon } from './icons';

// 탑승 시간 알림 창. 카트·관제의 알림 보고(flow 의 boardingAlert)가 있을 때만 그려진다.
// 시간 계산은 하지 않는다. 「예」「아니오」의 동작은 부르는 쪽이 기존 경로로 넘긴다.
// 창이 떠 있는 동안 뒤의 버튼은 가려진다. onPause 를 주면 창 안에 같은 「잠시 멈추기」를 그려 창을 닫지 않고 멈출 수 있다.
// onSwitch 가 없으면(게이트 안내 전환 경로가 없는 화면) 「확인」 버튼 하나만 그린다.
// 편명·탑승 시각·게이트는 예약 확인 응답에서 온다. 새로고침으로 잃었으면 빼고 안내한다.
type Props = {
  flightNo?: string;
  boardingStartText?: string;
  gateText?: string;
  busy?: boolean; // 「예」를 막는 조건. 기존 「게이트 안내로 전환」 버튼과 같다(요청 대기 중이거나 주행 중이 아닐 때)
  onSwitch?: () => void; // 기존 startService(pickup, 'GATE_GUIDE')
  onPause?: () => void; // 뒤 화면의 「잠시 멈추기」와 같은 요청. 중복 방지는 부르는 쪽이 한다
  pauseDisabled?: boolean;
  onDismiss: () => void; // 창만 닫는다
};

export function BoardingAlert({ flightNo, boardingStartText, gateText, busy = false, onSwitch, onPause, pauseDisabled = false, onDismiss }: Props) {
  const root = useRef<HTMLDivElement>(null);
  const dismissBtn = useRef<HTMLButtonElement>(null);
  const switchBtn = useRef<HTMLButtonElement>(null);

  // 열릴 때 명령을 보내지 않는 「아니오」에 포커스를 두고, 닫히면 원래 포커스로 돌려준다.
  useEffect(() => {
    const before = document.activeElement as HTMLElement | null;
    dismissBtn.current?.focus();
    return () => before?.focus?.();
  }, []);

  // 「예」가 막히면(전환 요청 대기 등) 그 버튼에 있던 포커스가 body 로 빠져 Esc·Tab 이 창에 닿지 않는다.
  // 그때는 명령을 보내지 않는 「아니오」로 포커스를 옮긴다. 브라우저가 포커스를 빼기 전에 이 효과가 먼저 돌 수 있어
  // 「예」에 포커스가 남아 있는 경우도 함께 본다.
  useEffect(() => {
    if (!busy) return;
    const active = document.activeElement;
    if (active === switchBtn.current || !root.current?.contains(active)) dismissBtn.current?.focus();
  }, [busy]);

  // Esc = 아니오, Tab 은 창 안에서만 돈다.
  function onKeyDown(e: React.KeyboardEvent) {
    if (e.key === 'Escape') {
      e.preventDefault();
      onDismiss();
      return;
    }
    if (e.key !== 'Tab' || !root.current) return;
    const items = [...root.current.querySelectorAll<HTMLButtonElement>('button:not(:disabled)')];
    if (items.length === 0) return;
    const first = items[0];
    const last = items[items.length - 1];
    if (e.shiftKey && document.activeElement === first) {
      e.preventDefault();
      last.focus();
    } else if (!e.shiftKey && document.activeElement === last) {
      e.preventDefault();
      first.focus();
    }
  }

  return (
    <div className="boarding" ref={root} role="alertdialog" aria-modal="true" aria-labelledby="boarding-title" aria-describedby="boarding-desc" onKeyDown={onKeyDown}>
      <div className="boarding-card">
        <div className="status-icon warn"><Icon name="alert" /></div>
        <p className="state-label warn">탑승 시간 알림</p>
        <h1 id="boarding-title">탑승 시작 15분 전입니다</h1>
        <p className="lead" id="boarding-desc">
          {flightNo && boardingStartText && gateText
            ? `${flightNo}편 탑승이 ${boardingStartText}에 시작됩니다(${gateText}).`
            : '탑승 시각과 게이트를 확인해 주세요.'}{' '}
          {!onSwitch
            ? '탑승 게이트로 이동해 주세요. 카트는 계속 따라갑니다.'
            : busy
              ? '게이트 안내 전환은 카트가 주행 중일 때 할 수 있습니다.'
              : '게이트 안내로 전환할까요? 전환하면 카트가 추종을 멈추고 게이트까지 앞에서 안내합니다.'}
        </p>
        <div className="actions">
          {onSwitch && <button className="btn btn-primary btn-big" ref={switchBtn} disabled={busy} onClick={onSwitch}>예, 게이트 안내로 전환</button>}
          <button className={onSwitch ? 'btn btn-secondary' : 'btn btn-primary btn-big'} ref={dismissBtn} onClick={onDismiss}>{onSwitch ? '아니오, 계속 따라오게 할게요' : '확인'}</button>
          {onPause && <button className="btn btn-stop" disabled={pauseDisabled} onClick={onPause}><Icon name="pause" />잠시 멈추기</button>}
        </div>
      </div>
    </div>
  );
}
