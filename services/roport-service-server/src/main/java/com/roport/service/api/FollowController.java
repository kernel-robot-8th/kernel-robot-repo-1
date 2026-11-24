package com.roport.service.api;

import java.util.Map;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.CacheControl;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestAttribute;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import tools.jackson.databind.JsonNode;

/** The existing cart boundary owns requests; only control reports make a robot ready. */
@RestController
class FollowController {
    private final FollowService service;
    FollowController(FollowService service) { this.service = service; }

    @GetMapping("/cart-api/follow/capabilities")
    ResponseEntity<Map<String, Object>> capabilities(@RequestAttribute(CartApiConfig.CART_ID) String cart,
            @RequestParam String verificationId) {
        return ResponseEntity.ok().cacheControl(CacheControl.noStore()).body(service.capabilities(cart, verificationId));
    }

    @GetMapping("/cart-api/follow/{receiptId}")
    ResponseEntity<Map<String, Object>> status(@RequestAttribute(CartApiConfig.CART_ID) String cart,
            @PathVariable String receiptId) {
        return ResponseEntity.ok().cacheControl(CacheControl.noStore()).body(service.status(cart, receiptId));
    }

    @PostMapping("/cart-api/follow/{receiptId}/actions")
    ResponseEntity<Map<String, Object>> action(@RequestAttribute(CartApiConfig.CART_ID) String cart,
            @PathVariable String receiptId, @RequestBody JsonNode body) {
        return ResponseEntity.accepted().cacheControl(CacheControl.noStore()).body(service.action(cart, receiptId, body));
    }

    @GetMapping("/cart-api/follow/{receiptId}/frames/{frameRef}")
    ResponseEntity<byte[]> frame(@RequestAttribute(CartApiConfig.CART_ID) String cart,
            @PathVariable String receiptId, @PathVariable String frameRef) {
        var frame = service.frame(cart, receiptId, frameRef);
        return ResponseEntity.ok().contentType(MediaType.parseMediaType(frame.contentType()))
                .header(HttpHeaders.CACHE_CONTROL, "no-store")
                .header("X-Content-Type-Options", "nosniff")
                .header("Content-Security-Policy", "default-src 'none'; sandbox")
                .body(frame.bytes());
    }
}
