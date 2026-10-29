#pragma once

// Platform-independent controls consumed by native gameplay systems.
namespace ac5 {

struct ButtonInput {
    bool held = false;
    bool pressed = false;
    bool released = false;
};

struct FlightInput {
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
    ButtonInput primary_fire;
    ButtonInput secondary_fire;
};

} // namespace ac5
