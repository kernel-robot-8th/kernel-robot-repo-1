package com.roport.service.api;

import com.roport.service.api.PickupService.CreatePickup;
import com.roport.service.api.PickupService.Flight;
import com.roport.service.api.PickupService.Location;
import com.roport.service.api.PickupService.Order;
import com.roport.service.api.PickupService.Pickup;
import java.time.Instant;
import java.time.LocalDate;
import java.time.OffsetDateTime;
import java.time.format.DateTimeParseException;
import java.util.List;
import java.util.Map;
import org.springframework.dao.DuplicateKeyException;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.bind.annotation.SessionAttribute;

@RestController
@RequestMapping("/api")
class CustomerController {
    record PickupRequest(String orderId, String locationId, String flightId, String pickupAt) {}

    private final PickupService service;

    CustomerController(PickupService service) {
        this.service = service;
    }

    @GetMapping("/orders")
    List<Order> orders(@SessionAttribute(WebConfig.MEMBER_ID) String memberId) {
        return service.orders(memberId);
    }

    @GetMapping("/pickup-locations")
    List<Location> locations() {
        return service.locations();
    }

    @GetMapping("/flights")
    List<Flight> flights(@RequestParam(required = false) String serviceDate) {
        try {
            return service.flights(serviceDate == null ? null : LocalDate.parse(serviceDate));
        } catch (DateTimeParseException e) {
            throw ApiException.badRequest("serviceDate는 YYYY-MM-DD 형식이어야 합니다.");
        }
    }

    @GetMapping("/orders/{orderId}/pickup-times")
    Map<String, List<Instant>> pickupTimes(@SessionAttribute(WebConfig.MEMBER_ID) String memberId,
            @PathVariable String orderId, @RequestParam String flightId, @RequestParam String locationId) {
        return Map.of("times", service.pickupTimes(memberId, orderId, locationId, flightId));
    }

    @PostMapping("/pickups")
    ResponseEntity<Pickup> create(@SessionAttribute(WebConfig.MEMBER_ID) String memberId,
            @RequestBody PickupRequest body) {
        CreatePickup req = validate(body);
        Pickup result;
        try {
            result = service.create(memberId, req);
        } catch (DuplicateKeyException e) {
            // 활성 예약 부분 UNIQUE 충돌(주문 잠금 밖의 저장과 경쟁). 새 트랜잭션에서 다시 판정하면 409 ACTIVE_PICKUP_EXISTS.
            result = service.create(memberId, req);
        }
        return ResponseEntity.status(HttpStatus.CREATED).body(result);
    }

    @GetMapping("/pickups")
    List<Pickup> pickups(@SessionAttribute(WebConfig.MEMBER_ID) String memberId) {
        return service.pickups(memberId);
    }

    @GetMapping("/pickups/{pickupId}")
    Pickup pickup(@SessionAttribute(WebConfig.MEMBER_ID) String memberId, @PathVariable String pickupId) {
        return service.pickup(memberId, pickupId).orElseThrow(() -> ApiException.notFound("PICKUP_NOT_FOUND"));
    }

    private static CreatePickup validate(PickupRequest b) {
        if (b == null || !id(b.orderId()) || !id(b.locationId()) || !id(b.flightId())
                || b.pickupAt() == null) {
            throw ApiException.badRequest("orderId·locationId·flightId·pickupAt이 필요합니다.");
        }
        try {
            // offset 없는 시각은 거부한다(계약: offset 있는 RFC3339만).
            Instant at = OffsetDateTime.parse(b.pickupAt()).toInstant();
            return new CreatePickup(b.orderId(), b.locationId(), b.flightId(), at);
        } catch (DateTimeParseException e) {
            throw ApiException.badRequest("pickupAt은 offset이 있는 RFC3339 시각이어야 합니다.");
        }
    }

    private static boolean id(String s) {
        return s != null && !s.isBlank() && s.length() <= 100;
    }
}
