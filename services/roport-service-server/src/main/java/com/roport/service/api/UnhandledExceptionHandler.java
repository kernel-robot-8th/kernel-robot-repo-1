package com.roport.service.api;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.core.Ordered;
import org.springframework.core.annotation.Order;
import org.springframework.http.HttpStatus;
import org.springframework.http.ProblemDetail;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;

/**
 * 처리되지 않은 예외 → 500 problem+json(code=INTERNAL_ERROR). 내부 정보는 로그에만 남긴다.
 * 최저 우선순위라 ApiExceptionHandler·Boot ProblemDetailsExceptionHandler(@Order(0))가 먼저 405 등을 처리한다.
 */
@RestControllerAdvice
@Order(Ordered.LOWEST_PRECEDENCE)
class UnhandledExceptionHandler {
    private static final Logger log = LoggerFactory.getLogger(UnhandledExceptionHandler.class);

    @ExceptionHandler(Exception.class)
    ResponseEntity<ProblemDetail> unhandled(Exception e) {
        log.error("unhandled exception", e);
        return ApiExceptionHandler.problem(HttpStatus.INTERNAL_SERVER_ERROR, "INTERNAL_ERROR", "서버 오류가 발생했습니다.");
    }
}
