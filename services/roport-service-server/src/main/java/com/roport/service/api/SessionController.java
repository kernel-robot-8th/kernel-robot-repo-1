package com.roport.service.api;

import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpSession;
import java.util.Map;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.jdbc.core.simple.JdbcClient;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

/** 개발 전용 로그인(login_id만 확인, 비밀번호 없음). 공개 배포 금지. */
@RestController
@RequestMapping("/api/session")
class SessionController {
    record LoginRequest(String loginId) {}

    record SessionResponse(String memberId, String displayName) {}

    private final JdbcClient db;

    SessionController(JdbcClient db) {
        this.db = db;
    }

    @PostMapping
    SessionResponse login(@RequestBody LoginRequest body, HttpServletRequest req) {
        if (body == null || body.loginId() == null || body.loginId().isBlank() || body.loginId().length() > 100) {
            throw ApiException.badRequest("loginId가 필요합니다.");
        }
        SessionResponse member = db.sql("SELECT member_id, display_name FROM members WHERE login_id = ?")
                .param(body.loginId())
                .query((rs, n) -> new SessionResponse(rs.getString(1), rs.getString(2)))
                .optional()
                .orElseThrow(() -> new ApiException(HttpStatus.UNAUTHORIZED, "LOGIN_FAILED", "로그인할 수 없습니다."));
        HttpSession old = req.getSession(false);
        if (old != null) {
            old.invalidate(); // 세션 고정 방지
        }
        HttpSession session = req.getSession(true);
        session.setAttribute(WebConfig.MEMBER_ID, member.memberId());
        session.setAttribute(WebConfig.DISPLAY_NAME, member.displayName());
        return member;
    }

    @GetMapping
    SessionResponse current(HttpSession session) {
        return new SessionResponse((String) session.getAttribute(WebConfig.MEMBER_ID),
                (String) session.getAttribute(WebConfig.DISPLAY_NAME));
    }

    @DeleteMapping
    ResponseEntity<Map<String, Object>> logout(HttpSession session) {
        session.invalidate();
        return ResponseEntity.noContent().build();
    }
}
