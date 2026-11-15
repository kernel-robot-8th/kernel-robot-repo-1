package com.roport.service.api;

import com.roport.service.api.QrService.QrToken;
import org.springframework.http.CacheControl;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.bind.annotation.SessionAttribute;

/** 고객 예약 QR 발급. 토큰 원문은 이 응답에만 있으므로 캐시하지 않게 한다. */
@RestController
@RequestMapping("/api")
class QrController {
    private final QrService service;

    QrController(QrService service) {
        this.service = service;
    }

    @PostMapping("/pickups/{pickupId}/qr-token")
    ResponseEntity<QrToken> issue(@SessionAttribute(WebConfig.MEMBER_ID) String memberId, @PathVariable String pickupId) {
        return ResponseEntity.status(HttpStatus.CREATED).cacheControl(CacheControl.noStore())
                .body(service.issue(memberId, pickupId));
    }
}
