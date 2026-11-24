import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
// 개발 서버에서만 카트 API를 로컬 서비스 서버로 넘긴다. 배포 nginx 전달은 아직 없다.
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/cart-api': process.env.ROPORT_SERVICE_HTTP_URL || 'http://127.0.0.1:8080',
      // 탑승 시간 알림의 로봇 표시(선택, VITE_PAD_BOARDING_ROBOT=1): 같은 PC 의 roport_follow tool --boarding-alert
      '/follow-tool': { target: process.env.ROPORT_FOLLOW_TOOL_URL || 'http://127.0.0.1:8090', rewrite: (path) => path.replace(/^\/follow-tool/, '') },
    },
  },
});
