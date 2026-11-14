"""관제 단일 실행부: 요청 기반 FLEET와 구간 시험 TRIAL.

실행부가 MQTT·서비스 WSS·운영 저장소를 소유하며 API는 로컬 소켓으로 요청한다.
adapters.api·UI를 import하지 않는다(tests/test_layers.py).
"""
