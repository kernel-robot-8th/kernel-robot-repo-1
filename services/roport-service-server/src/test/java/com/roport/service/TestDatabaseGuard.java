package com.roport.service;

import java.sql.Connection;
import java.sql.ResultSet;
import org.springframework.boot.flyway.autoconfigure.FlywayMigrationStrategy;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

/**
 * 테스트 전용(src/test, 컴포넌트 스캔으로 모든 @SpringBootTest에 적용). 기본 SERVICE_DB_URL은 개발 DB다.
 * Flyway가 무엇이든 적용하기 전에 실제 접속 DB 이름이 _test로 끝나는지 확인해, 아니면 컨텍스트 시작을 실패시킨다.
 */
@Configuration(proxyBeanMethods = false)
class TestDatabaseGuard {
    @Bean
    FlywayMigrationStrategy requireTestDatabase() {
        return flyway -> {
            String name;
            try (Connection c = flyway.getConfiguration().getDataSource().getConnection();
                    ResultSet rs = c.createStatement().executeQuery("SELECT current_database()")) {
                rs.next();
                name = rs.getString(1);
            } catch (java.sql.SQLException e) {
                throw new IllegalStateException("테스트 DB 이름을 확인할 수 없습니다", e);
            }
            if (!name.endsWith("_test")) {
                throw new IllegalStateException(
                        "SERVICE_DB_URL은 이름이 _test로 끝나는 격리 DB여야 한다(현재: " + name + "). 마이그레이션하지 않았다.");
            }
            flyway.migrate();
        };
    }
}
