# 팀 LAN 전용 배포용. application.properties의 127.0.0.1 바인딩은 SERVER_ADDRESS로 덮어쓴다.
FROM eclipse-temurin:21-jdk AS build
WORKDIR /app
COPY gradlew settings.gradle.kts build.gradle.kts ./
COPY gradle gradle
COPY src src
RUN ./gradlew --no-daemon bootJar

FROM eclipse-temurin:21-jre
RUN useradd --system --create-home roport
WORKDIR /app
COPY --from=build /app/build/libs/*.jar app.jar
USER roport
ENV SERVER_ADDRESS=0.0.0.0
EXPOSE 8080
HEALTHCHECK --interval=15s --timeout=3s --retries=5 CMD bash -c 'exec 3<>/dev/tcp/127.0.0.1/8080; printf "GET /actuator/health HTTP/1.0\\r\\n\\r\\n" >&3; grep -q "\"status\":\"UP\"" <&3'
CMD ["java", "-jar", "app.jar"]
