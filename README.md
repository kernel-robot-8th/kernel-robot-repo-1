# roport-web

RO:PORT · GATE CREW — 고객 서비스 웹·어드민 웹

## 기술 스택

Node.js 24 / React 19 / TypeScript 7 / Vite 8 / npm workspaces. apps/service와 apps/admin을 별도 빌드합니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](https://github.com/kernel-robot-8th/kernel-robot-repo-1/blob/main/docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

```bash
npm ci
npm run typecheck
npm run build
npm run dev --workspace @roport/service
# 별도 터미널
npm run dev --workspace @roport/admin -- --port 5174
```

각 앱의 `dist/`를 별도로 배포할 수 있습니다. 현재 기능은 준비 화면뿐입니다.
브라우저에 전달하는 환경변수에 비밀키를 넣지 않습니다.

## 상세 설계 기준

- [레포별 기술 스택·통신·CI 계획](https://github.com/kernel-robot-8th/kernel-robot-repo-1/blob/main/docs/technology-stack.md)

문서의 현재 구현과 후속 개발 기준을 구분합니다. 실장비·GPU 확인 전 의존성 업그레이드를 확정하지 않습니다.
