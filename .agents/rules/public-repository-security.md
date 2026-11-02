# 공개 저장소 보안 규칙

코드뿐 아니라 Git 이력, PR·이슈 본문, Actions 로그·아티팩트와 첨부 이미지도 공개 범위다. 자동 검사 통과만으로 공개 가능하다고 판단하지 않는다.

## 올리지 않는 정보

- 실제 비밀번호, API/PAT 토큰, SSH·TLS 개인키, 쿠키·세션, DB 접속 비밀값, Wi-Fi 비밀번호, MQTT/EMQX·클라우드 인증정보. 환경변수나 별도 비밀 저장소에서 주입한다.
- `.env` 변형 파일, 로컬 인증 파일, DB 덤프, 패킷 캡처, ROS bag, 실제 카메라 원본·녹음, 학습 데이터·가중치. 공개 예제 모델·이미지와 상위 프로젝트의 테스트 자산은 출처·라이선스·내용을 확인한다.
- 얼굴·음성·연락처·고객/주문 식별자, 비공개 회의록·Confluence/Jira 본문, 공개가 승인되지 않은 현장 지도. 원문 대신 필요한 내용을 비식별화한다.
- 실제 운영 주소·SSID·MAC·장비 일련번호는 필요성을 검토해 예제에서 제거한다. 사설 IP 자체를 비밀키로 취급하지 않으며, 예제 주소는 `localhost` 또는 `example.com`, 값은 `REPLACE_ME` 같은 명확한 자리표시자를 사용한다.

`.env.example`에도 실제 값을 넣지 않는다. 프런트엔드 번들·설치 패키지에 포함되는 환경변수는 사용자에게 노출되므로 비밀 보관 수단이 아니다.

## 개발 PC 준비

저장소 루트에서 Node.js 24와 Gitleaks 8.30.1을 준비한다. 아래 설치기는 Linux x86_64 전용이며 공식 배포 파일의 SHA-256을 확인한다. 다른 OS는 [동일 버전 공식 배포](https://github.com/gitleaks/gitleaks/releases/tag/v8.30.1)를 사용한다.

```bash
sh scripts/install-gitleaks.sh
export PATH="$HOME/.local/bin:$PATH"
npm ci
npm run test:security
```

`npm ci`가 기존 Husky에 커밋 전 staged 검사와 push 전 로컬에 확보된 모든 ref의 이력 검사를 연결한다. Gitleaks가 없으면 검사는 실패한다. 새 clone·다른 PC에서도 설치가 필요하며, shallow clone은 전체 이력을 먼저 가져온다.

## 공유 전 순서

1. 필요한 파일만 stage하고 `git diff --cached --stat`, 파일 목록, 실제 diff를 로컬에서 확인한다. 의심 값은 채팅이나 공개 로그로 출력하지 않는다.
2. `sh scripts/check-secrets.sh --pre-commit --staged`로 올릴 변경을 검사한다. `.gitignore`는 이미 추적 중인 파일이나 강제 추가를 막지 못한다.
3. 커밋 후 `npm run check:secrets`로 로컬에 확보된 모든 ref의 이력을 검사한다. 현재 브랜치 외에 push할 브랜치·태그와 중간 커밋에서 삭제한 비밀값도 검사 대상이다. CI는 전체 이력을 받아 PR의 병합 검증 커밋 또는 push된 HEAD와 가져온 ref를 검사한다.
4. PR·이슈·스크린샷·로그는 별도로 검토한다. 코드 검사기는 이미지 속 개인정보나 문서의 공개 권한을 판단하지 못한다. 원본 캡처·검사 상세 보고서는 저장소 밖에 둔다.

검사 우회(`--no-verify`, `HUSKY=0`, `gitleaks:allow`, 광범위한 경로 제외)를 해결책으로 사용하지 않는다. 오탐은 근거와 최소 예외 범위를 리뷰받는다. 저장소 전체·벤더 전체를 검사에서 제외하지 않는다.

## GitHub와 CI

- 관리자는 저장소의 Secret scanning과 Push protection을 활성화하고 유지한다. [GitHub 문서](https://docs.github.com/en/code-security/concepts/secret-security/push-protection)에 따른 지원 토큰 탐지에는 한계가 있으며 모든 비밀번호·개인정보를 차단하지 않는다.
- `Secret scan` CI는 토큰을 마스킹하고 읽기 권한만 사용한다. 공개 검사 보고서·SARIF 업로드는 하지 않는다. 실패한 검사를 무시하고 병합하지 않는다.
- 새 CI는 최소 권한과 고정된 Action SHA를 사용한다. 비신뢰 PR 코드를 비밀값·쓰기 권한이 있는 `pull_request_target` 작업에서 실행하지 않는다. 자격증명을 출력하거나 다운로드한 스크립트를 검증 없이 실행하지 않는다.

## 이미 공개된 경우

1. 추가 공유를 중단하고 실제 값 대신 파일·커밋·영향 범위를 비공개로 정리한다. 공개 이슈·PR에 비밀값을 재현하지 않는다.
2. 자격증명 소유자가 해당 키를 폐기·교체한다. 단순 파일 삭제나 PR 닫기로 노출이 취소되지 않는다.
3. 이력·로그·아티팩트 정리는 팀과 범위를 정한 후 수행한다. 승인 없이 force push·공유 이력 삭제를 하지 않는다. [GitHub 제거 절차](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/removing-sensitive-data-from-a-repository)를 따른다.
