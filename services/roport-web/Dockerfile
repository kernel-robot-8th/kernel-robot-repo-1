# 정적 빌드를 nginx로 서빙한다. 고객용(service)은 8080, 관리자용(admin)은 8081.
FROM node:24-bookworm-slim AS build
WORKDIR /app
COPY . .
RUN npm ci && npm run build

# 비루트로 도는 이미지: compose의 cap_drop: ALL에서도 기동한다(공식 nginx는 setuid가 필요).
FROM nginxinc/nginx-unprivileged:1.27-alpine
COPY nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /app/apps/service/dist /usr/share/nginx/service
COPY --from=build /app/apps/admin/dist /usr/share/nginx/admin
EXPOSE 8080 8081
