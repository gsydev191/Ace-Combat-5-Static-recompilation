#include "flight_dynamics.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ac5 {
namespace {

bool finite_values(const FlightSpeedRules& rules, const FlightSpeedInput& input) {
    const std::array values{
        rules.spool_up_gain, rules.spool_down_gain, rules.climb_penalty_percent,
        rules.cruise_kmh, rules.maximum_kmh, rules.minimum_kmh,
        rules.stall_kmh, rules.full_control_kmh, rules.dive_limit_kmh,
        rules.dive_extra_kmh, input.altitude_units, input.pitch_radians,
        input.throttle_lever, input.current_speed_units_per_tick};
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

double density_scale(double altitude_units) {
    double density_height = 0.0;
    if (altitude_units < 31210.0)
        density_height = altitude_units / 100.0;
    else if (altitude_units < 46815.0)
        density_height = 312.1 + altitude_units - 31210.0;
    else
        density_height = 15917.1 + 10.0 * (altitude_units - 46815.0);
    return std::exp2(-density_height / 13420.0);
}

} // namespace

FlightSpeedOutput advance_airborne_speed(const FlightSpeedRules& rules,
                                         const FlightSpeedInput& input) noexcept {
    FlightSpeedOutput output{};
    if (!finite_values(rules, input) ||
        (input.ticks_per_second != 30 && input.ticks_per_second != 60) ||
        rules.spool_up_gain < 0.0 || rules.spool_down_gain < 0.0 ||
        rules.cruise_kmh <= 0.0 || rules.maximum_kmh <= 0.0 ||
        rules.minimum_kmh < 0.0 || rules.stall_kmh <= 0.0 ||
        rules.full_control_kmh <= rules.stall_kmh ||
        rules.dive_limit_kmh <= 0.0) {
        return output;
    }

    const double density = density_scale(input.altitude_units);
    if (!(density > 0.0) || !std::isfinite(density)) return output;
    const double inverse_density_scale = 1.0 / density;
    const double pitch_scale = input.pitch_radians / 1.5707963;
    const double mode_scale = input.reduced_speed_mode ? 0.5 : 1.0;
    const double speed_scale = 1.4222219 / input.ticks_per_second * mode_scale;
    const double thin_air = 0.3 * (1.0 - density);

    double cruise = rules.cruise_kmh * speed_scale;
    double maximum = rules.maximum_kmh * speed_scale;
    double minimum = rules.minimum_kmh * speed_scale;
    cruise += thin_air * cruise;
    maximum += thin_air * maximum;
    minimum += thin_air * minimum;

    const double stall_speed = rules.stall_kmh * speed_scale * inverse_density_scale;
    const double full_control_speed = rules.full_control_kmh * speed_scale *
                                      inverse_density_scale;
    const double dive_limit = rules.dive_limit_kmh * speed_scale *
                              inverse_density_scale;
    if (pitch_scale >= 0.0) {
        const double climb_factor = 1.0 - pitch_scale *
                                    rules.climb_penalty_percent / 100.0;
        cruise *= climb_factor;
        maximum *= climb_factor;
        minimum *= climb_factor;
    } else {
        const double dive_addition = rules.dive_extra_kmh * speed_scale *
                                     inverse_density_scale * -pitch_scale;
        cruise += dive_addition;
        maximum = std::min(maximum + dive_addition, dive_limit);
        minimum += dive_addition;
    }

    const double lever = std::clamp(input.throttle_lever, -1.0, 1.0);
    const double target = lever >= 0.0
        ? cruise + (maximum - cruise) * lever
        : cruise + (minimum - cruise) * -lever;
    const double speed = input.current_speed_units_per_tick;
    const double gain = speed > target ? rules.spool_down_gain : rules.spool_up_gain;
    output.speed_units_per_tick = speed + (target - speed) *
        (gain / 100.0 * 60.0 / input.ticks_per_second);
    output.target_units_per_tick = target;
    output.stall_factor = output.speed_units_per_tick < stall_speed
        ? 1.0
        : output.speed_units_per_tick < full_control_speed
            ? (full_control_speed - output.speed_units_per_tick) /
                (full_control_speed - stall_speed)
            : 0.0;
    output.valid = std::isfinite(output.speed_units_per_tick) &&
                   std::isfinite(output.target_units_per_tick) &&
                   std::isfinite(output.stall_factor);
    return output;
}

} // namespace ac5
