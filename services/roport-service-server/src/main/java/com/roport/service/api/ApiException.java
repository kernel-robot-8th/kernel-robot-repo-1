package com.roport.service.api;

import org.springframework.http.HttpStatus;

/** 계약의 오류 code와 HTTP 상태. ApiExceptionHandler가 ProblemDetail로 바꾼다. */
public class ApiException extends RuntimeException {
    final HttpStatus status;
    final String code;

    public ApiException(HttpStatus status, String code, String detail) {
        super(detail);
        this.status = status;
        this.code = code;
    }

    static ApiException badRequest(String detail) {
        return new ApiException(HttpStatus.BAD_REQUEST, "VALIDATION_FAILED", detail);
    }

    static ApiException notFound(String code) {
        return new ApiException(HttpStatus.NOT_FOUND, code, "리소스를 찾을 수 없습니다.");
    }
}
