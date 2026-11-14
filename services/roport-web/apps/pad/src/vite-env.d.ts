/// <reference types="vite/client" />

// 빌드 때 브라우저 코드에 들어가는 값이다. 비밀값을 넣지 않는다.
interface ImportMetaEnv {
  readonly VITE_PAD_API?: 'server';
  readonly VITE_PAD_CART_ID?: string;
}
