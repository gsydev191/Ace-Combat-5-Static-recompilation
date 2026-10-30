#include "flight_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ac5 {
namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Quaternion = std::array<double, 4>;
constexpr double pi = 3.1415925;

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs) {
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            for (std::size_t k = 0; k < 3; ++k)
                result[row][column] += lhs[row][k] * rhs[k][column];
    return result;
}

Matrix3 transpose(const Matrix3& matrix) {
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            result[row][column] = matrix[column][row];
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

Quaternion game_quaternion(const Matrix3& row_matrix) {
    const Matrix3 matrix = transpose(row_matrix);
    const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if (trace > 0.0) {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        return {(matrix[2][1] - matrix[1][2]) / scale,
                (matrix[0][2] - matrix[2][0]) / scale,
                (matrix[1][0] - matrix[0][1]) / scale, scale / 4.0};
    }
    if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
        const double scale = std::sqrt(1.0 + matrix[0][0] - matrix[1][1] -
                                       matrix[2][2]) * 2.0;
        return {scale / 4.0,
                (matrix[0][1] + matrix[1][0]) / scale,
                (matrix[0][2] + matrix[2][0]) / scale,
                (matrix[2][1] - matrix[1][2]) / scale};
    }
    if (matrix[1][1] > matrix[2][2]) {
        const double scale = std::sqrt(1.0 + matrix[1][1] - matrix[0][0] -
                                       matrix[2][2]) * 2.0;
        return {(matrix[0][1] + matrix[1][0]) / scale, scale / 4.0,
                (matrix[1][2] + matrix[2][1]) / scale,
                (matrix[0][2] - matrix[2][0]) / scale};
    }
    const double scale = std::sqrt(1.0 + matrix[2][2] - matrix[0][0] -
                                   matrix[1][1]) * 2.0;
    return {(matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale, scale / 4.0,
            (matrix[1][0] - matrix[0][1]) / scale};
}

Matrix3 quaternion_matrix(double x, double y, double z, double w) {
    return {{{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
              2.0 * (x * z + y * w)}},
             {{2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
              2.0 * (y * z - x * w)}},
             {{2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
              1.0 - 2.0 * (x * x + y * y)}}}};
}

std::array<double, 3> slerp_euler(const std::array<double, 3>& from,
                                  const std::array<double, 3>& to,
                                  double amount) {
    const Quaternion first = game_quaternion(euler_matrix(from));
    Quaternion second = game_quaternion(euler_matrix(to));
    double dot = 0.0;
    for (std::size_t i = 0; i < first.size(); ++i) dot += first[i] * second[i];
    if (dot < 0.0) {
        for (double& component : second) component = -component;
        dot = -dot;
    }
    dot = std::min(dot, 1.0);
    Quaternion blended = first;
    if (1.0 - dot * dot != 0.0) {
        const double sine = std::sqrt(1.0 - dot * dot);
        const double theta = std::acos(dot);
        const double first_weight = std::sin(theta - theta * amount) / sine;
        const double second_weight = std::sin(theta * amount) / sine;
        for (std::size_t i = 0; i < blended.size(); ++i)
            blended[i] = first_weight * first[i] + second_weight * second[i];
    }
    return to_euler(quaternion_matrix(-blended[0], -blended[1],
                                      -blended[2], blended[3]));
}

std::array<double, 3> row_multiply(const std::array<double, 3>& vector,
                                   const Matrix3& matrix) {
    std::array<double, 3> result{};
    for (std::size_t column = 0; column < 3; ++column)
        for (std::size_t row = 0; row < 3; ++row)
            result[column] += vector[row] * matrix[row][column];
    return result;
}

bool finite(const std::array<double, 3>& values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

} // namespace

bool advance_aircraft_flight_tick(const AircraftFlightRules& rules,
                                  const AircraftFlightInput& input,
                                  AircraftFlightState& state) noexcept {
    if (!finite(state.position) || !finite(state.euler) ||
        !finite(state.path_euler) || !finite(state.rates) ||
        !std::isfinite(state.speed_units_per_tick) ||
        !std::isfinite(state.path_lag) || !std::isfinite(input.pitch) ||
        !std::isfinite(input.roll) || !std::isfinite(input.yaw) ||
        !std::isfinite(input.throttle_lever)) return false;

    const FlightSpeedInput speed_input{
        state.position[1], state.euler[0], input.throttle_lever,
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

    next.path_euler = state.path_lag < 1.0
        ? slerp_euler(state.path_euler, next.euler, state.path_lag)
        : next.euler;
    std::array<double, 3> velocity = row_multiply(
        {0.0, 0.0, -next.speed_units_per_tick}, euler_matrix(next.path_euler));
    velocity[1] -= next.stall_factor * next.stall_factor *
                   (input.reduced_speed_mode ? 2.5 : 5.0);
    if (!input.suppress_inverted_sink) {
        const double normalized_roll = next.euler[2] / pi;
        velocity[1] -= normalized_roll * normalized_roll *
            (rules.speed.inverted_sink_mps * 5.12 / input.ticks_per_second);
    }
    if (input.reduced_speed_mode) {
        const double half_cruise = rules.speed.cruise_kmh * 1.4222219 * 0.5 /
                                   input.ticks_per_second;
        if (next.speed_units_per_tick < half_cruise)
            velocity[1] -= (1.0 - next.speed_units_per_tick / half_cruise) * 0.5;
        velocity[1] -= 0.09999999403953552;
    }
    for (std::size_t i = 0; i < 3; ++i) next.position[i] += velocity[i];

    if (!finite(next.position) || !finite(next.path_euler) ||
        !finite(next.euler) || !finite(next.rates) ||
        !std::isfinite(next.speed_units_per_tick) ||
        !std::isfinite(next.stall_factor)) return false;
    state = next;
    return true;
}

} // namespace ac5
