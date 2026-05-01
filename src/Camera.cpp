#include "Camera.h"

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

void Camera::reset(const glm::vec3& center, float sceneRadius) {
    float d = std::max(sceneRadius * 1.45f, 2.0f);

    // Keep the original/simple Y-up camera behavior, but start from the front.
    // The room itself is flipped when loaded, instead of making the camera Z-up.
    position = center + glm::vec3(0.0f, d * 0.35f, d);

    yaw = -90.0f;
    pitch = -18.0f;
    speed = std::max(sceneRadius * 0.55f, 1.0f);
    updateVectors();
}

glm::mat4 Camera::viewMatrix() const {
    return glm::lookAt(position, position + front, up);
}

void Camera::moveForward(float dt) {
    position += front * speed * dt;
}

void Camera::moveBackward(float dt) {
    position -= front * speed * dt;
}

void Camera::moveLeft(float dt) {
    glm::vec3 right = glm::normalize(glm::cross(front, up));
    position -= right * speed * dt;
}

void Camera::moveRight(float dt) {
    glm::vec3 right = glm::normalize(glm::cross(front, up));
    position += right * speed * dt;
}

void Camera::moveUp(float dt) {
    position += up * speed * dt;
}

void Camera::moveDown(float dt) {
    position -= up * speed * dt;
}

void Camera::addYawPitch(float dx, float dy) {
    yaw += dx * mouseSensitivity;
    pitch += dy * mouseSensitivity;
    pitch = std::max(-89.0f, std::min(89.0f, pitch));
    updateVectors();
}

void Camera::updateVectors() {
    float cy = std::cos(glm::radians(yaw));
    float sy = std::sin(glm::radians(yaw));
    float cp = std::cos(glm::radians(pitch));
    float sp = std::sin(glm::radians(pitch));

    front = glm::normalize(glm::vec3(cy * cp, sp, sy * cp));
    glm::vec3 right = glm::normalize(glm::cross(front, worldUp));
    up = glm::normalize(glm::cross(right, front));
}
