"""관제 실행부 프로세스 계층.

Fleet은 역할·배차·통행·복구를 판단하고 MQTT와 서비스 WSS를 연결한다.
API의 조작은 Unix socket으로 받고, 운영 상태·서비스 요청은 PostgreSQL에 저장한다.
별도 순차 주행 시험은 파일 journal을 사용한다. 요청 접수와 실제 완료를 구분한다.

Import 경계는 tests/test_layers.py를 따른다. adapters.api는 import하지 않는다.
"""
