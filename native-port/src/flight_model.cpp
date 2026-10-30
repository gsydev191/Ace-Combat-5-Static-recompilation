#include "flight_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ac5 {
namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs) {
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            for (std::size_t k = 0; k < 3; ++k)
                result[row][column] += lhs[row][k] * rhs[k][column];
    return result;
}

Matrix3 euler_matrix(const std::array<double, 3>& euler) {
    const double cp = std::cos(euler[0]), sp = std::sin(euler[0]);
    const double cy = std::cos(euler[1]), sy = std::sin(euler[1]);
    const double cr = std::cos(euler[2]), sr = std::sin(euler[2]);
    const Matrix3 pitch{{{{1.0, 0.0, 0.0}},
                         {{0.0, cp, sp}},
                         {{0.0, -sp, cp}}}};
    const Matrix3 yaw{{{{cy, 0.0, -sy}},
                       {{0.0, 1.0, 0.0}},
                       {{sy, 0.0, cy}}}};
    const Matrix3 roll{{{{cr, sr, 0.0}},
                        {{-sr, cr, 0.0}},
                        {{0.0, 0.0, 1.0}}}};
    return multiply(multiply(roll, pitch), yaw);
}

std::array<double, 3> to_euler(const Matrix3& matrix) {
    return {-std::asin(std::clamp(matrix[2][1], -1.0, 1.0)),
            std::atan2(matrix[2][0], matrix[2][2]),
            std::atan2(matrix[0][1], matrix[1][1])};
}

bool finite(const std::array<double, 3>& values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

} // namespace

bool advance_aircraft_flight_tick(const AircraftFlightRules& rules,
                                  const AircraftFlightInput& input,
                                  AircraftFlightState& state) noexcept {
    if (!finite(state.euler) || !finite(state.rates) ||
        !std::isfinite(state.speed_units_per_tick) ||
        !std::isfinite(input.altitude_units) || !std::isfinite(input.pitch) ||
        !std::isfinite(input.roll) || !std::isfinite(input.yaw) ||
        !std::isfinite(input.throttle_lever)) return false;

    const FlightSpeedInput speed_input{
        input.altitude_units, state.euler[0], input.throttle_lever,
        state.speed_units_per_tick, input.ticks_per_second,
        input.reduced_speed_mode};
    const FlightSpeedOutput speed = advance_airborne_speed(rules.speed, speed_input);
    if (!speed.valid) return false;

    const auto axis_input = [&input, &speed](double stick, double current_rate) {
        return FlightAxisInput{stick, speed.speed_units_per_tick,
            speed.stall_speed_units_per_tick,
            speed.full_control_speed_units_per_tick, current_rate,
            input.ticks_per_second};
    };
    const FlightAxisOutput pitch = advance_flight_axis(
        rules.pitch, axis_input(input.pitch, state.rates[0]));
    const FlightAxisOutput roll = advance_flight_axis(
        rules.roll, axis_input(-input.roll, state.rates[2]));
    const FlightAxisOutput yaw = advance_flight_axis(
        rules.yaw, axis_input(input.yaw, state.rates[1]));
    if (!pitch.valid || !roll.valid || !yaw.valid) return false;

    AircraftFlightState next = state;
    next.speed_units_per_tick = speed.speed_units_per_tick;
    next.stall_factor = speed.stall_factor;
    next.rates = {pitch.rate_radians_per_tick,
                  yaw.rate_radians_per_tick,
                  roll.rate_radians_per_tick};
    if (next.rates[0] != 0.0 || next.rates[1] != 0.0 || next.rates[2] != 0.0)
        next.euler = to_euler(multiply(euler_matrix(next.rates),
                                       euler_matrix(state.euler)));
    if (!finite(next.euler) || !finite(next.rates) ||
        !std::isfinite(next.speed_units_per_tick) ||
        !std::isfinite(next.stall_factor)) return false;
    state = next;
    return true;
}

} // namespace ac5
