# 팀 LAN 전용 배포용. 호스트 게시는 deploy/compose.yaml이 LAN IP로 제한한다.
FROM python:3.12-slim
RUN pip install --no-cache-dir uv==0.13.5 && useradd --system --create-home roport
WORKDIR /app
COPY --chown=roport:roport . .
USER roport
RUN uv sync --locked --no-dev
EXPOSE 8000
HEALTHCHECK --interval=15s --timeout=3s --retries=5 CMD python -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8000/healthz', timeout=2)"
CMD ["uv", "run", "--no-sync", "uvicorn", "app:app", "--app-dir", "src", "--host", "0.0.0.0", "--port", "8000"]
