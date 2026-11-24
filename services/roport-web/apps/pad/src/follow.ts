export type FollowMode = 'none' | 'marker' | 'appearance';
export const FOLLOW_ACTIONS = ['capture', 'select', 'release', 'pause', 'resume', 'return'] as const;
export type FollowOperation = typeof FOLLOW_ACTIONS[number];
export type Box = [number, number, number, number];
export interface FollowRegistration {
  frameRef?: string;
  imagePath?: string;
  expiresAt: string;
  perceptionBootId: string;
  candidates?: { candidateId: string; box: Box }[];
  registered?: boolean;
}
export interface FollowStatus {
  receiptId: string;
  mode: FollowMode;
  stage: string;
  status: string;
  reason: string | null;
  updatedAt: string | null;
  revision: number;
  allowedActions: FollowOperation[];
  stale: boolean;
  boardingAlert: boolean; // 서버가 보낸 탑승 시간 알림. 필드가 없거나 true 가 아니면 false
  registration?: FollowRegistration;
  lastRequest?: { requestId: string; status: string; reason?: string | null };
}
export interface FollowAction {
  requestId: string;
  expectedRevision: number;
  operation: FollowOperation;
  frameRef?: string;
  candidateId?: string;
  box?: Box;
}
export interface ReceiptIntent {
  verificationId: string;
  confirmationRequestId: string;
  followMode: FollowMode;
  followCard?: number;
}

const record = (value: unknown): value is Record<string, unknown> => Boolean(value && typeof value === 'object' && !Array.isArray(value));
export const followMode = (value: unknown): value is FollowMode => value === 'none' || value === 'marker' || value === 'appearance';
export const validBox = (value: unknown): value is Box => Array.isArray(value) && value.length === 4 &&
  value.every(n => typeof n === 'number' && Number.isFinite(n) && n >= 0 && n <= 1) && value[0] < value[2] && value[1] < value[3];
const text = (value: unknown): value is string => typeof value === 'string' && value.length > 0;

export function receiptIntent(verificationId: string, mode: FollowMode, card?: number): ReceiptIntent {
  if (!verificationId || !followMode(mode) || (mode === 'marker' && (!Number.isInteger(card) || card! < 35 || card! > 49))) {
    throw new Error('수령할 예약과 추종 카드 번호를 확인해 주세요.');
  }
  return { verificationId, confirmationRequestId: `receipt-${verificationId}`, followMode: mode, ...(mode === 'marker' ? { followCard: card } : {}) };
}

export function parseReceiptIntent(value: unknown): ReceiptIntent | null {
  if (!record(value) || !text(value.verificationId) || !followMode(value.followMode)) return null;
  try {
    const intent = receiptIntent(value.verificationId, value.followMode, typeof value.followCard === 'number' ? value.followCard : undefined);
    return value.confirmationRequestId === intent.confirmationRequestId ? intent : null;
  } catch { return null; }
}

export function framePath(receiptId: string, frameRef: string): string {
  return `/cart-api/follow/${encodeURIComponent(receiptId)}/frames/${encodeURIComponent(frameRef)}`;
}

export function parseFollowStatus(value: unknown, receiptId: string): FollowStatus | null {
  if (!record(value) || value.receiptId !== receiptId || !followMode(value.mode) || !text(value.stage) ||
      !text(value.status) || !Number.isSafeInteger(value.revision) || (value.revision as number) < 0 ||
      typeof value.stale !== 'boolean' || !Array.isArray(value.allowedActions) ||
      value.allowedActions.some(a => !FOLLOW_ACTIONS.includes(a as FollowOperation))) return null;
  let registration: FollowRegistration | undefined;
  if (value.registration != null) {
    const r = value.registration;
    if (!record(r) || !text(r.perceptionBootId) || !text(r.expiresAt) || !Number.isFinite(Date.parse(r.expiresAt)) ||
        (r.registered != null && typeof r.registered !== 'boolean')) return null;
    if (r.registered === true) {
      registration = { registered: true, expiresAt: r.expiresAt, perceptionBootId: r.perceptionBootId };
    } else {
      if (!text(r.frameRef) || r.imagePath !== framePath(receiptId, r.frameRef) || !Array.isArray(r.candidates) ||
          r.candidates.some(c => !record(c) || !text(c.candidateId) || !validBox(c.box))) return null;
      registration = r as unknown as FollowRegistration;
    }
  }
  const last = value.lastRequest;
  if (last != null && (!record(last) || !text(last.requestId) || !text(last.status))) return null;
  return {
    receiptId, mode: value.mode, stage: value.stage, status: value.status,
    reason: typeof value.reason === 'string' ? value.reason : null,
    updatedAt: typeof value.updatedAt === 'string' ? value.updatedAt : null,
    revision: value.revision as number, stale: value.stale, boardingAlert: value.boardingAlert === true,
    allowedActions: value.allowedActions as FollowOperation[], registration,
    ...(last != null ? { lastRequest: last as unknown as FollowStatus['lastRequest'] } : {}),
  };
}

// A delayed response cannot undo a newer state or restore an expired selection.
export function newerFollowStatus(current: FollowStatus | null, incoming: FollowStatus): FollowStatus {
  return current && incoming.revision < current.revision ? current : incoming;
}

// ---- 탑승 시간 알림(서버가 알린다) ----
// 알림 시점은 관제가 정한다. 패드는 상태 조회의 boardingAlert 가 true 인 동안만 창을 띄우고 스스로 시간을 재지 않는다.

// 창을 띄울 수 있는 상태: 최신 조회 결과가 「로봇이 실제 추종 활성을 보고한 단계」일 때만 true.
// 접수(follow_start)·정지·복귀·HOLD·취소, 조회 실패(offline), 서버가 오래됐다고 알린 상태(stale)는 아니다.
export function followingNow(status: Pick<FollowStatus, 'stage' | 'status' | 'stale'> | null, offline = false): boolean {
  return Boolean(status && !offline && !status.stale && status.stage === 'following' && status.status !== 'HOLD' && status.status !== 'CANCELED');
}
