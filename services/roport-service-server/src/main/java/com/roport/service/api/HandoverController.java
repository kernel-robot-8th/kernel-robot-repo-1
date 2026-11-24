package com.roport.service.api;

import java.util.Map;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestAttribute;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RestController;

@RestController
class HandoverController {
    record Confirmation(String verificationId, String confirmationRequestId, Integer followCard, String followMode) {}
    private final HandoverService service;
    HandoverController(HandoverService service) { this.service = service; }

    @PostMapping("/cart-api/receipts")
    Map<String, Object> confirm(@RequestAttribute(CartApiConfig.CART_ID) String cart, @RequestBody Confirmation request) {
        if (request == null || request.verificationId() == null || request.confirmationRequestId() == null
                || !CartApiConfig.ID.matcher(request.verificationId()).matches()
                || !CartApiConfig.ID.matcher(request.confirmationRequestId()).matches()) throw ApiException.badRequest("verificationId·confirmationRequestId가 필요합니다.");
        return service.confirm(cart, request.verificationId(), request.confirmationRequestId(), request.followCard(), request.followMode());
    }
}
