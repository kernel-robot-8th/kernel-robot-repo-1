"""관제 실행부 프로세스 계층 (골격, 아직 구현 없음).

통합관제 Tech Spec v4 기준으로 현장 관제 PC에서 실행부는 한 개만 실행한다.
API가 운영 DB에 저장한 요청을 가져와 검사하고 domain 규칙을 호출한 뒤,
승인한 명령과 송신 대기(outbox) 기록을 한 트랜잭션으로 저장한다.
전송은 adapters(MQTT·서비스 WSS)가 맡는다. 요청 저장, 장비 접수, 실제 완료를 구분한다.

domain·contracts·adapters를 import할 수 있지만 adapters.ui·adapters.api는
import하지 않는다(tests/test_layers.py가 강제). 관련 Jira: FFT1-106.
"""
