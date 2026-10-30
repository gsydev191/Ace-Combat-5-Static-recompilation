#include "input.hpp"

#include <algorithm>
#include <cmath>

namespace ac5 {

namespace {

float axis_value(Sint16 raw) {
    constexpr float dead_zone = 0.18f;
    const float value = raw < 0 ? static_cast<float>(raw) / 32768.0f
                                : static_cast<float>(raw) / 32767.0f;
    if (std::abs(value) <= dead_zone) return 0.0f;
    const float scaled = (std::abs(value) - dead_zone) / (1.0f - dead_zone);
    return std::copysign(scaled, value);
}

float combine(float keyboard, float controller) {
    return std::abs(controller) > std::abs(keyboard) ? controller : keyboard;
}

} // namespace

InputSystem::InputSystem() {
    open_available_gamepad();
}

void InputSystem::open_available_gamepad() {
    if (gamepad_) return;
    int count = 0;
    SDL_JoystickID* gamepads = SDL_GetGamepads(&count);
    if (!gamepads) return;
    for (int i = 0; i < count && !gamepad_; ++i)
        gamepad_ = SDL_OpenGamepad(gamepads[i]);
    SDL_free(gamepads);
}

InputSystem::~InputSystem() {
    if (gamepad_) SDL_CloseGamepad(gamepad_);
}

void InputSystem::handle_event(const SDL_Event& event) {
    if (event.type == SDL_EVENT_GAMEPAD_ADDED && !gamepad_) {
        gamepad_ = SDL_OpenGamepad(event.gdevice.which);
        if (!gamepad_) open_available_gamepad();
    } else if (event.type == SDL_EVENT_GAMEPAD_REMOVED && gamepad_ &&
               SDL_GetGamepadID(gamepad_) == event.gdevice.which) {
        SDL_CloseGamepad(gamepad_);
        gamepad_ = nullptr;
        open_available_gamepad();
    }
}

FlightInput InputSystem::sample() {
    int key_count = 0;
    const bool* keys = SDL_GetKeyboardState(&key_count);
    const auto down = [keys, key_count](SDL_Scancode key) {
        return key >= 0 && key < key_count && keys[key];
    };
    const float keyboard_pitch = static_cast<float>(down(SDL_SCANCODE_DOWN) || down(SDL_SCANCODE_S)) -
                                 static_cast<float>(down(SDL_SCANCODE_UP) || down(SDL_SCANCODE_W));
    const float keyboard_roll = static_cast<float>(down(SDL_SCANCODE_RIGHT) || down(SDL_SCANCODE_D)) -
                                static_cast<float>(down(SDL_SCANCODE_LEFT) || down(SDL_SCANCODE_A));
    const float keyboard_yaw = static_cast<float>(down(SDL_SCANCODE_E)) -
                               static_cast<float>(down(SDL_SCANCODE_Q));
    const float keyboard_throttle = static_cast<float>(down(SDL_SCANCODE_R)) -
                                   static_cast<float>(down(SDL_SCANCODE_F));

    FlightInput input{};
    input.pitch = keyboard_pitch;
    input.roll = keyboard_roll;
    input.yaw = keyboard_yaw;
    input.throttle = keyboard_throttle;
    input.primary_fire.held = down(SDL_SCANCODE_SPACE);
    input.secondary_fire.held = down(SDL_SCANCODE_LCTRL) || down(SDL_SCANCODE_RCTRL);
    input.approach_control.held = down(SDL_SCANCODE_P);
    input.gear_toggle.held = down(SDL_SCANCODE_G);
    if (gamepad_) {
        input.pitch = combine(input.pitch,
            -axis_value(SDL_GetGamepadAxis(gamepad_, SDL_GAMEPAD_AXIS_LEFTY)));
        input.roll = combine(input.roll,
            axis_value(SDL_GetGamepadAxis(gamepad_, SDL_GAMEPAD_AXIS_LEFTX)));
        input.yaw = combine(input.yaw,
            axis_value(SDL_GetGamepadAxis(gamepad_, SDL_GAMEPAD_AXIS_RIGHTX)));
        const float left_trigger = std::max(0.0f,
            SDL_GetGamepadAxis(gamepad_, SDL_GAMEPAD_AXIS_LEFT_TRIGGER) / 32767.0f);
        const float right_trigger = std::max(0.0f,
            SDL_GetGamepadAxis(gamepad_, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) / 32767.0f);
        input.throttle = combine(input.throttle, right_trigger - left_trigger);
        input.primary_fire.held = input.primary_fire.held ||
            SDL_GetGamepadButton(gamepad_, SDL_GAMEPAD_BUTTON_SOUTH);
        input.secondary_fire.held = input.secondary_fire.held ||
            SDL_GetGamepadButton(gamepad_, SDL_GAMEPAD_BUTTON_EAST);
    }
    input.primary_fire.pressed = input.primary_fire.held && !previous_primary_fire_;
    input.primary_fire.released = !input.primary_fire.held && previous_primary_fire_;
    input.secondary_fire.pressed = input.secondary_fire.held && !previous_secondary_fire_;
    input.secondary_fire.released = !input.secondary_fire.held && previous_secondary_fire_;
    input.approach_control.pressed = input.approach_control.held &&
                                     !previous_approach_control_;
    input.approach_control.released = !input.approach_control.held &&
                                      previous_approach_control_;
    input.gear_toggle.pressed = input.gear_toggle.held && !previous_gear_toggle_;
    input.gear_toggle.released = !input.gear_toggle.held && previous_gear_toggle_;
    previous_primary_fire_ = input.primary_fire.held;
    previous_secondary_fire_ = input.secondary_fire.held;
    previous_approach_control_ = input.approach_control.held;
    previous_gear_toggle_ = input.gear_toggle.held;
    return input;
}

} // namespace ac5
