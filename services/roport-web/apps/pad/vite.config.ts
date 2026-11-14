import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
// 개발 서버에서만 카트 API를 로컬 서비스 서버로 넘긴다. 배포 nginx 전달은 아직 없다.
export default defineConfig({
  plugins: [react()],
  server: { proxy: { '/cart-api': 'http://127.0.0.1:8080' } },
});
