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
    bool previous_approach_control_ = false;
};

} // namespace ac5
