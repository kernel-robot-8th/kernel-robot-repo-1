package com.roport.service.api;

import com.roport.service.api.VerificationService.Method;
import com.roport.service.api.VerificationService.Outcome;
import com.roport.service.api.VerificationService.Verification;
import com.roport.service.api.VerificationService.VerifyRequest;
import java.util.Locale;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestAttribute;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/** 카트 패드 API(개발 전용 카트 인증, CartApiConfig). 새 확인은 201, 같은 요청 ID 재전송은 200. */
@RestController
@RequestMapping("/cart-api")
class CartController {
    record VerifyBody(String verificationRequestId, String method, String value) {}

    private final VerificationService service;

    CartController(VerificationService service) {
        this.service = service;
    }

    @PostMapping("/verifications")
    ResponseEntity<Verification> verify(@RequestAttribute(CartApiConfig.CART_ID) String cartId,
            @RequestBody VerifyBody body) {
        VerifyRequest req = validate(body);
        Outcome out = service.verify(cartId, req); // 같은 요청 ID의 동시 저장은 ON CONFLICT 후 재조회로 처리한다
        return ResponseEntity.status(out.created() ? HttpStatus.CREATED : HttpStatus.OK).body(out.verification());
    }

    private static VerifyRequest validate(VerifyBody b) {
        if (b == null || b.verificationRequestId() == null || !CartApiConfig.ID.matcher(b.verificationRequestId()).matches()
                || b.method() == null || b.value() == null) {
            throw ApiException.badRequest("verificationRequestId·method·value가 필요합니다.");
        }
        Method method;
        try {
            method = Method.valueOf(b.method());
        } catch (IllegalArgumentException e) {
            throw ApiException.badRequest("method는 QR 또는 RESERVATION_NO입니다.");
        }
        String value = b.value().trim();
        if (method == Method.RESERVATION_NO) {
            value = value.toUpperCase(Locale.ROOT);
            if (value.length() != 8 || !value.chars().allMatch(c -> PickupService.RESERVATION_ALPHABET.indexOf(c) >= 0)) {
                throw ApiException.badRequest("예약번호는 8자리(0·O·1·I 제외)입니다.");
            }
        } else if (!value.startsWith(QrTokens.PREFIX) || value.length() > 200) {
            throw ApiException.badRequest("QR 값 형식이 올바르지 않습니다.");
        }
        return new VerifyRequest(b.verificationRequestId(), method, value);
    }
}
