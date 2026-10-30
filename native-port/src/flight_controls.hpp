#pragma once

namespace ac5 {

// Native outputs of the aircraft throttle-lever routine at 0x0016bf50.
// All channels are normalized to [0, 1].
struct EngineControlOutput {
    float afterburner = 0.0f;
    float dry_power = 0.0f;
    float airbrake = 0.0f;
};

// Convert the game's -1..1 throttle lever into engine and airbrake controls.
// -1 is full airbrake, -0.6 is idle, 0 is cruise, 0.9 is military power,
// and 1 is full afterburner.
[[nodiscard]] EngineControlOutput map_engine_lever(float lever) noexcept;

} // namespace ac5
