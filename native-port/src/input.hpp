#pragma once

#include "flight_input.hpp"

#include <SDL3/SDL.h>

namespace ac5 {

class InputSystem {
public:
    InputSystem();
    ~InputSystem();
    InputSystem(const InputSystem&) = delete;
    InputSystem& operator=(const InputSystem&) = delete;

    void handle_event(const SDL_Event& event);
    ac5::FlightInput sample();

private:
    void open_available_gamepad();

    SDL_Gamepad* gamepad_ = nullptr;
    bool previous_primary_fire_ = false;
    bool previous_secondary_fire_ = false;
    bool previous_weapon_switch_ = false;
    bool previous_approach_control_ = false;
    bool previous_gear_toggle_ = false;
    bool primary_fire_pressed_ = false;
    bool primary_fire_released_ = false;
    bool secondary_fire_pressed_ = false;
    bool secondary_fire_released_ = false;
    bool weapon_switch_pressed_ = false;
    bool weapon_switch_released_ = false;
    bool approach_control_pressed_ = false;
    bool approach_control_released_ = false;
    bool gear_toggle_pressed_ = false;
    bool gear_toggle_released_ = false;
};

} // namespace ac5
