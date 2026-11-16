package com.roport.service.control;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.autoconfigure.condition.ConditionalOnProperty;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.HttpStatus;
import org.springframework.http.server.ServerHttpRequest;
import org.springframework.http.server.ServerHttpResponse;
import org.springframework.scheduling.annotation.EnableScheduling;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;
import org.springframework.web.socket.handler.ConcurrentWebSocketSessionDecorator;
import org.springframework.web.socket.handler.TextWebSocketHandler;
import org.springframework.web.socket.server.HandshakeInterceptor;
import tools.jackson.databind.ObjectMapper;

/** Explicitly enabled single-site link; bearer credentials never enter a URL or log. */
@Configuration
@EnableWebSocket
@EnableScheduling
@ConditionalOnProperty(name = "service.control.enabled", havingValue = "true")
public class ControlWebSocket extends TextWebSocketHandler implements WebSocketConfigurer {
    private final ControlExchange exchange;
    private final ObjectMapper json;
    private final String site;
    private final byte[] authorization;
    private final Map<String, WebSocketSession> sessions = new ConcurrentHashMap<>();

    public ControlWebSocket(ControlExchange exchange, ObjectMapper json,
            @Value("${service.control.site}") String site, @Value("${service.control.token}") String token) {
        if (token.isBlank() || site.isBlank() || site.contains("/")) throw new IllegalArgumentException("control site/token required");
        this.exchange = exchange;
        this.json = json;
        this.site = site;
        authorization = ("Bearer " + token).getBytes(StandardCharsets.UTF_8);
    }

    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
        registry.addHandler(this, "/ws/v1/sites/" + site + "/control").addInterceptors(new HandshakeInterceptor() {
            @Override
            public boolean beforeHandshake(ServerHttpRequest req, ServerHttpResponse res, WebSocketHandler h, Map<String, Object> attrs) {
                String header = req.getHeaders().getFirst("Authorization");
                boolean secure = "https".equals(req.getURI().getScheme())
                        || req.getRemoteAddress().getAddress().isLoopbackAddress();
                if (!secure || header == null || !MessageDigest.isEqual(authorization, header.getBytes(StandardCharsets.UTF_8))) {
                    res.setStatusCode(HttpStatus.UNAUTHORIZED);
                    return false;
                }
                return true;
            }
            @Override
            public void afterHandshake(ServerHttpRequest req, ServerHttpResponse res, WebSocketHandler h, Exception error) {}
        });
    }

    @Override
    public void afterConnectionEstablished(WebSocketSession session) throws Exception {
        var wrapped = new ConcurrentWebSocketSessionDecorator(session, 10000, 1024 * 1024);
        if (sessions.putIfAbsent(site, wrapped) != null) {
            session.close(CloseStatus.POLICY_VIOLATION.withReason("control already connected"));
            return;
        }
        flush();
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        try {
            var response = exchange.receive(site, message.getPayload());
            if (response != null) sessions.get(site).sendMessage(new TextMessage(json.writeValueAsString(response)));
        } catch (IllegalArgumentException e) {
            session.close(CloseStatus.BAD_DATA.withReason("invalid control message"));
        }
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        sessions.computeIfPresent(site, (key, active) -> active.getId().equals(session.getId()) ? null : active);
    }

    @Scheduled(fixedDelay = 1000)
    public void flush() throws java.io.IOException {
        WebSocketSession session = sessions.get(site);
        if (session == null || !session.isOpen()) return;
        exchange.publishReceipts(site);
        for (String message : exchange.pending(site)) session.sendMessage(new TextMessage(message));
    }
}
