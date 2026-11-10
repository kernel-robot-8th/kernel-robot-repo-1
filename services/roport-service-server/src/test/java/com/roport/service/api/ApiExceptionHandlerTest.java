package com.roport.service.api;

import static org.assertj.core.api.Assertions.assertThat;

import java.lang.reflect.Method;
import org.junit.jupiter.api.Test;
import org.springframework.http.HttpStatus;
import org.springframework.http.ProblemDetail;
import org.springframework.http.ResponseEntity;
import org.springframework.web.method.annotation.ExceptionHandlerMethodResolver;
import org.springframework.web.method.annotation.MethodArgumentTypeMismatchException;

/** Spring 컨텍스트 없이 advice 규칙만 검사한다. */
class ApiExceptionHandlerTest {
    final ApiExceptionHandler handler = new ApiExceptionHandler();

    String codeFor(int status) {
        ProblemDetail p = ProblemDetail.forStatus(status);
        handler.beforeBodyWrite(p, null, null, null, null, null);
        return (String) p.getProperties().get("code");
    }

    @Test
    void fillsGenericCodeFromStatus() {
        assertThat(codeFor(404)).isEqualTo("NOT_FOUND");
        assertThat(codeFor(405)).isEqualTo("METHOD_NOT_ALLOWED");
        assertThat(codeFor(406)).isEqualTo("NOT_ACCEPTABLE");
        assertThat(codeFor(415)).isEqualTo("UNSUPPORTED_MEDIA_TYPE");
        assertThat(codeFor(409)).isEqualTo("BAD_REQUEST");
        assertThat(codeFor(400)).isEqualTo("BAD_REQUEST");
        assertThat(codeFor(500)).isEqualTo("INTERNAL_ERROR");
        assertThat(codeFor(503)).isEqualTo("INTERNAL_ERROR");
    }

    @Test
    void keepsExistingCode() {
        ProblemDetail p = ApiExceptionHandler.problem(HttpStatus.NOT_FOUND, "ORDER_NOT_FOUND", "x").getBody();
        handler.beforeBodyWrite(p, null, null, null, null, null);
        assertThat(p.getProperties()).containsEntry("code", "ORDER_NOT_FOUND");
    }

    /** 현재 경로에서는 도달하지 않지만(방어용) 매핑과 응답을 고정한다. */
    @Test
    void typeMismatchIsValidationFailed() throws Exception {
        MethodArgumentTypeMismatchException e = new MethodArgumentTypeMismatchException("x", Integer.class, "n", null, null);
        Method m = new ExceptionHandlerMethodResolver(ApiExceptionHandler.class).resolveMethodByThrowable(e);
        assertThat(m).isNotNull();
        @SuppressWarnings("unchecked")
        ResponseEntity<ProblemDetail> r = (ResponseEntity<ProblemDetail>) m.invoke(handler, e);
        assertThat(r.getStatusCode().value()).isEqualTo(400);
        assertThat(r.getBody().getType().toString()).isEqualTo("about:blank");
        assertThat(r.getBody().getProperties()).containsEntry("code", "VALIDATION_FAILED");
    }
}
