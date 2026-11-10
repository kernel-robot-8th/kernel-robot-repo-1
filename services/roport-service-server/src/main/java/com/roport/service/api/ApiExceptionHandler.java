package com.roport.service.api;

import java.net.URI;

import org.springframework.core.MethodParameter;
import org.springframework.core.Ordered;
import org.springframework.core.annotation.Order;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ProblemDetail;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.HttpMessageConverter;
import org.springframework.http.converter.HttpMessageNotReadableException;
import org.springframework.http.server.ServerHttpRequest;
import org.springframework.http.server.ServerHttpResponse;
import org.springframework.web.bind.MissingServletRequestParameterException;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;
import org.springframework.web.method.annotation.MethodArgumentTypeMismatchException;
import org.springframework.web.servlet.mvc.method.annotation.ResponseBodyAdvice;

/**
 * 오류 응답: application/problem+json {type,title,status,detail,code}.
 * problemdetails.enabled 의 기본 핸들러보다 먼저 400 계열을 처리해 code 를 항상 붙인다.
 */
@RestControllerAdvice
@Order(Ordered.HIGHEST_PRECEDENCE)
class ApiExceptionHandler implements ResponseBodyAdvice<Object> {
    static final URI ABOUT_BLANK = URI.create("about:blank");

    @Override
    public boolean supports(MethodParameter returnType, Class<? extends HttpMessageConverter<?>> converterType) {
        return true;
    }

    /** 프레임워크 기본 오류(405·415·404 등)의 ProblemDetail은 type·code가 없다. 계약 필드라 상태로 채운다. */
    @Override
    public Object beforeBodyWrite(Object body, MethodParameter returnType, MediaType contentType,
            Class<? extends HttpMessageConverter<?>> converterType, ServerHttpRequest req, ServerHttpResponse res) {
        if (body instanceof ProblemDetail p) {
            if (p.getType() == null) {
                p.setType(ABOUT_BLANK);
            }
            if (p.getProperties() == null || !p.getProperties().containsKey("code")) {
                p.setProperty("code", genericCode(p.getStatus()));
            }
        }
        return body;
    }

    static String genericCode(int status) {
        return switch (status) {
            case 404 -> "NOT_FOUND";
            case 405 -> "METHOD_NOT_ALLOWED";
            case 406 -> "NOT_ACCEPTABLE";
            case 409 -> "CONFLICT";
            case 415 -> "UNSUPPORTED_MEDIA_TYPE";
            default -> status >= 500 ? "INTERNAL_ERROR" : "BAD_REQUEST";
        };
    }

    @ExceptionHandler(ApiException.class)
    ResponseEntity<ProblemDetail> api(ApiException e) {
        return problem(e.status, e.code, e.getMessage());
    }

    @ExceptionHandler({HttpMessageNotReadableException.class, MissingServletRequestParameterException.class,
            MethodArgumentTypeMismatchException.class})
    ResponseEntity<ProblemDetail> badRequest(Exception e) {
        return problem(HttpStatus.BAD_REQUEST, "VALIDATION_FAILED", "요청 형식이 올바르지 않습니다.");
    }

    static ResponseEntity<ProblemDetail> problem(HttpStatus status, String code, String detail) {
        ProblemDetail body = ProblemDetail.forStatusAndDetail(status, detail);
        body.setType(ABOUT_BLANK); // 기본값이면 직렬화에서 빠지므로 계약 필드 type을 명시한다
        body.setTitle(status.getReasonPhrase());
        body.setProperty("code", code);
        return ResponseEntity.status(status).body(body);
    }
}
