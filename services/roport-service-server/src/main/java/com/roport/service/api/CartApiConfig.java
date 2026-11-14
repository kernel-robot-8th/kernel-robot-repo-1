package com.roport.service.api;

import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import java.util.regex.Pattern;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.HttpStatus;
import org.springframework.web.servlet.HandlerInterceptor;
import org.springframework.web.servlet.config.annotation.InterceptorRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

/**
 * 개발 전용 카트 인증: 요청 헤더 X-Cart-Id만으로 카트를 식별한다(비밀값 없음, 위조 가능). 공개 배포 금지.
 * service.dev-cart.enabled=true 일 때만 열리고, 기본값(false)에서는 모든 /cart-api/** 가 401이다.
 * 고객 세션(/api/**)과 섞지 않는다. 운영용 기기 인증은 후속이다.
 */
@Configuration
class CartApiConfig implements WebMvcConfigurer {
    static final String CART_ID = "cartId";
    static final String HEADER = "X-Cart-Id";
    static final Pattern ID = Pattern.compile("[A-Za-z0-9_.-]{1,64}");

    private static final Logger log = LoggerFactory.getLogger(CartApiConfig.class);

    private final boolean enabled;

    CartApiConfig(@Value("${service.dev-cart.enabled:false}") boolean enabled) {
        this.enabled = enabled;
        if (enabled) {
            log.warn("개발 전용 카트 인증이 켜져 있습니다(X-Cart-Id만 확인, 비밀값 없음). 공개 배포 금지");
        }
    }

    @Override
    public void addInterceptors(InterceptorRegistry registry) {
        registry.addInterceptor(new HandlerInterceptor() {
            @Override
            public boolean preHandle(HttpServletRequest req, HttpServletResponse res, Object handler) {
                String cartId = req.getHeader(HEADER);
                if (!enabled || cartId == null || !ID.matcher(cartId).matches()) {
                    throw new ApiException(HttpStatus.UNAUTHORIZED, "CART_UNAUTHENTICATED", "카트 인증이 필요합니다.");
                }
                req.setAttribute(CART_ID, cartId);
                return true;
            }
        }).addPathPatterns("/cart-api/**");
    }
}
