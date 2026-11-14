package com.roport.service.api;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.security.SecureRandom;
import java.util.Base64;
import java.util.HexFormat;

/** 예약 QR 토큰. 원문은 발급 응답으로 한 번만 내보내고 DB(qr_credentials.token_hash)에는 SHA-256 해시만 둔다. */
final class QrTokens {
    /** 형식 버전. 예약번호(8자리)와 구분되고, 형식을 바꾸면 접두사를 올린다. */
    static final String PREFIX = "RPQ1.";
    private static final SecureRandom RANDOM = new SecureRandom();

    private QrTokens() {}

    /** 256비트 무작위 값. 추측할 수 없어야 하므로 예약 ID·회원 ID에서 만들지 않는다. */
    static String newToken() {
        byte[] bytes = new byte[32];
        RANDOM.nextBytes(bytes);
        return PREFIX + Base64.getUrlEncoder().withoutPadding().encodeToString(bytes);
    }

    static String hash(String token) {
        try {
            byte[] digest = MessageDigest.getInstance("SHA-256").digest(token.getBytes(StandardCharsets.UTF_8));
            return HexFormat.of().formatHex(digest);
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("SHA-256을 사용할 수 없습니다", e);
        }
    }
}
