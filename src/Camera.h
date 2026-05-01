#pragma once

#include <glm/glm.hpp>

class Camera {
public:
    glm::vec3 position {3.0f, 2.0f, 3.0f};
    glm::vec3 front {0.0f, 0.0f, -1.0f};
    glm::vec3 up {0.0f, 1.0f, 0.0f};
    glm::vec3 worldUp {0.0f, 1.0f, 0.0f};

    float yaw = -135.0f;
    float pitch = -20.0f;
    float speed = 2.5f;
    float mouseSensitivity = 0.12f;

    Camera() = default;

    void reset(const glm::vec3& center, float sceneRadius);
    glm::mat4 viewMatrix() const;

    void moveForward(float dt);
    void moveBackward(float dt);
    void moveLeft(float dt);
    void moveRight(float dt);
    void moveUp(float dt);
    void moveDown(float dt);

    void addYawPitch(float dx, float dy);

private:
    void updateVectors();
};
