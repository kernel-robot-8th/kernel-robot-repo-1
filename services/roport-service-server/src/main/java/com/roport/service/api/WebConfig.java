package com.roport.service.api;

import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.HttpSession;
import java.time.Clock;
import org.springframework.boot.web.server.servlet.CookieSameSiteSupplier;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.HttpStatus;
import org.springframework.web.method.HandlerMethod;
import org.springframework.web.servlet.HandlerInterceptor;
import org.springframework.web.servlet.config.annotation.InterceptorRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

/**
 * 개발 전용 세션 인증: login_id만으로 로그인하는 HttpSession 쿠키(HttpOnly, SameSite=Strict).
 * 비밀번호·Spring Security가 없으므로 공개 배포에 쓰면 안 된다. 로그인은 service.dev-login.enabled=true 일 때만 열린다.
 */
@Configuration
class WebConfig implements WebMvcConfigurer {
    static final String MEMBER_ID = "memberId";
    static final String DISPLAY_NAME = "displayName";

    @Bean
    Clock clock() {
        return Clock.systemUTC();
    }

    // 세션 쿠키는 서블릿 기본으로 HttpOnly다. SameSite만 추가한다.
    @Bean
    CookieSameSiteSupplier sessionCookieSameSite() {
        return CookieSameSiteSupplier.ofStrict().whenHasName("JSESSIONID");
    }

    @Override
    public void addInterceptors(InterceptorRegistry registry) {
        registry.addInterceptor(new HandlerInterceptor() {
            @Override
            public boolean preHandle(HttpServletRequest req, HttpServletResponse res, Object handler) {
                // 로그인 핸들러 자체만 면제한다. 개발용 로그인이 꺼져 SessionController가 없으면 면제도 없다.
                if ("POST".equals(req.getMethod()) && handler instanceof HandlerMethod h
                        && h.getBeanType() == SessionController.class) {
                    return true;
                }
                HttpSession session = req.getSession(false);
                if (session == null || session.getAttribute(MEMBER_ID) == null) {
                    throw new ApiException(HttpStatus.UNAUTHORIZED, "UNAUTHENTICATED", "로그인이 필요합니다.");
                }
                return true;
            }
        }).addPathPatterns("/api/**");
    }
}
