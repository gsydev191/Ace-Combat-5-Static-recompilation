#include "mission_steering.hpp"

#include <algorithm>
#include <bit>
#include <cmath>

namespace ac5 {
namespace {

constexpr double pi = 3.14159265358979323846;

double fold_angle(double angle) noexcept {
    while (angle > pi) angle -= 2.0 * pi;
    while (angle <= -pi) angle += 2.0 * pi;
    return angle;
}

bool valid_state(const Ac5MissionSteeringState& state) noexcept {
    return std::all_of(state.position.begin(), state.position.end(),
                       [](double value) { return std::isfinite(value); }) &&
        std::all_of(state.attitude.begin(), state.attitude.end(),
                    [](double value) { return std::isfinite(value); }) &&
        std::all_of(state.velocity.begin(), state.velocity.end(),
                    [](double value) { return std::isfinite(value); }) &&
        std::isfinite(state.speed_world_units_per_tick) &&
        std::all_of(state.order_start_position.begin(), state.order_start_position.end(),
                    [](double value) { return std::isfinite(value); }) &&
        std::all_of(state.order_start_attitude.begin(), state.order_start_attitude.end(),
                    [](double value) { return std::isfinite(value); });
}

std::int16_t target_speed_kmh(const Ac5MissionOrder& order) noexcept {
    return std::bit_cast<std::int16_t>(
        static_cast<std::uint16_t>(order.raw_words[13]));
}

double speed_per_tick(std::int16_t speed_kmh, double rate) noexcept {
    return static_cast<double>(speed_kmh) * 1000.0 / 3600.0 / rate * 5.12;
}

std::uint8_t waypoint_mode(const Ac5MissionOrder& order) noexcept {
    return static_cast<std::uint8_t>(order.raw_words[14] >> 24);
}

void set_flag(std::uint16_t& flags, std::uint16_t mask, bool enabled) noexcept {
    flags = enabled ? static_cast<std::uint16_t>(flags | mask)
                    : static_cast<std::uint16_t>(flags & ~mask);
}

bool forward_displacement(const std::array<double, 3>& attitude, double speed,
                          std::array<double, 3>& displacement) noexcept {
    const double pitch = attitude[0], yaw = attitude[1];
    const double cp = std::cos(pitch);
    displacement = {-std::sin(yaw) * cp * speed,
                    std::sin(pitch) * speed,
                    -std::cos(yaw) * cp * speed};
    return std::all_of(displacement.begin(), displacement.end(),
                       [](double value) { return std::isfinite(value); });
}

bool aim_angles(const std::array<double, 4>& position,
                const std::array<double, 4>& target,
                double& pitch, double& yaw) noexcept {
    const double dx = target[0] - position[0];
    const double dy = target[1] - position[1];
    const double dz = target[2] - position[2];
    const double horizontal = std::hypot(dx, dz);
    pitch = std::atan2(dy, horizontal);
    yaw = fold_angle(std::atan2(dx, dz) + pi);
    return std::isfinite(pitch) && std::isfinite(yaw);
}

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Quaternion = std::array<double, 4>;

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs) noexcept {
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
            for (std::size_t index = 0; index < 3; ++index)
                result[row][column] += lhs[row][index] * rhs[index][column];
    return result;
}

Quaternion euler_quaternion(const std::array<double, 3>& angles) noexcept {
    const double cp = std::cos(angles[0]), sp = std::sin(angles[0]);
    const double cy = std::cos(angles[1]), sy = std::sin(angles[1]);
    const double cr = std::cos(angles[2]), sr = std::sin(angles[2]);
    const Matrix3 pitch{{{{1.0, 0.0, 0.0}}, {{0.0, cp, sp}}, {{0.0, -sp, cp}}}};
    const Matrix3 yaw{{{{cy, 0.0, -sy}}, {{0.0, 1.0, 0.0}}, {{sy, 0.0, cy}}}};
    const Matrix3 roll{{{{cr, sr, 0.0}}, {{-sr, cr, 0.0}}, {{0.0, 0.0, 1.0}}}};
    const Matrix3 row = multiply(multiply(roll, pitch), yaw);
    const Matrix3 m{{{{row[0][0], row[1][0], row[2][0]}},
                     {{row[0][1], row[1][1], row[2][1]}},
                     {{row[0][2], row[1][2], row[2][2]}}}};
    const double trace = m[0][0] + m[1][1] + m[2][2];
    if (trace > 0.0) {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        return {(m[2][1] - m[1][2]) / scale,
                (m[0][2] - m[2][0]) / scale,
                (m[1][0] - m[0][1]) / scale, scale / 4.0};
    }
    if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double scale = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        return {scale / 4.0, (m[0][1] + m[1][0]) / scale,
                (m[0][2] + m[2][0]) / scale, (m[2][1] - m[1][2]) / scale};
    }
    if (m[1][1] > m[2][2]) {
        const double scale = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        return {(m[0][1] + m[1][0]) / scale, scale / 4.0,
                (m[1][2] + m[2][1]) / scale, (m[0][2] - m[2][0]) / scale};
    }
    const double scale = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    return {(m[0][2] + m[2][0]) / scale,
            (m[1][2] + m[2][1]) / scale, scale / 4.0,
            (m[1][0] - m[0][1]) / scale};
}

std::array<double, 3> quaternion_euler(const Quaternion& q) noexcept {
    const double x = -q[0], y = -q[1], z = -q[2], w = q[3];
    const double m21 = 2.0 * (y * z + x * w);
    const double m20 = 2.0 * (x * z - y * w);
    const double m22 = 1.0 - 2.0 * (x * x + y * y);
    const double m01 = 2.0 * (x * y - z * w);
    const double m11 = 1.0 - 2.0 * (x * x + z * z);
    return {-std::asin(std::clamp(m21, -1.0, 1.0)),
            std::atan2(m20, m22), std::atan2(m01, m11)};
}

std::array<double, 3> blend_attitude(const std::array<double, 3>& from,
                                     const std::array<double, 3>& to,
                                     double amount) noexcept {
    const Quaternion first = euler_quaternion(from);
    Quaternion second = euler_quaternion(to);
    double cosine = 0.0;
    for (std::size_t index = 0; index < 4; ++index)
        cosine += first[index] * second[index];
    if (cosine < 0.0) {
        for (double& value : second) value = -value;
        cosine = -cosine;
    }
    cosine = std::clamp(cosine, -1.0, 1.0);
    double a = 1.0 - amount, b = amount;
    if (1.0 - cosine > 1.0e-6) {
        const double angle = std::acos(cosine);
        const double sine = std::sin(angle);
        a = std::sin((1.0 - amount) * angle) / sine;
        b = std::sin(amount * angle) / sine;
    }
    Quaternion result{};
    for (std::size_t index = 0; index < 4; ++index)
        result[index] = a * first[index] + b * second[index];
    double norm = 0.0;
    for (double value : result) norm += value * value;
    norm = std::sqrt(norm);
    if (norm <= 1.0e-12) return from;
    for (double& value : result) value /= norm;
    return quaternion_euler(result);
}

bool advance_keyframed_path(const Ac5MissionPath& path,
                            const Ac5MissionSteeringInput& input,
                            Ac5MissionSteeringState& state) noexcept {
    if (path.keys.empty() || !input.reference_position_available ||
        !std::isfinite(input.reference_position[2]) ||
        !std::isfinite(path.z_margin) || !std::isfinite(path.z_start) ||
        !std::isfinite(path.tangent)) return false;
    double desired_z = input.reference_position[2] + path.z_margin;
    if (path.z_start < desired_z) {
        desired_z = path.keys.front().position[2] + path.z_margin;
    } else {
        const double difference = desired_z - state.position[2];
        if (difference < -48.0) desired_z = state.position[2] - 48.0;
        else if (difference > 2.0) desired_z = state.position[2] - 8.0;
    }
    std::size_t segment = 0;
    if (path.keys.front().position[2] > desired_z) {
        segment = 1;
        while (segment < path.keys.size() &&
               desired_z < path.keys[segment].position[2]) ++segment;
        if (segment == path.keys.size()) {
            state.waypoint_complete = true;
            return true;
        }
    }
    state.path_segment = segment;
    if (segment == 0) {
        const auto& key = path.keys.front();
        state.position = {key.position[0], key.position[1], key.position[2], 1.0};
        state.attitude = {key.attitude[0], key.attitude[1], key.attitude[2]};
        return true;
    }
    const auto& first = path.keys[segment - 1];
    const auto& second = path.keys[segment];
    const double z0 = first.position[2], z1 = second.position[2];
    if (!(z1 < z0)) return false;
    const double t = (desired_z - z0) / (z1 - z0);
    if (!std::isfinite(t)) return false;
    state.attitude = blend_attitude(
        {first.attitude[0], first.attitude[1], first.attitude[2]},
        {second.attitude[0], second.attitude[1], second.attitude[2]}, t);
    const double reach = static_cast<double>(path.tangent) * 0.33299997448921204;
    const auto forward = [](const Ac5MissionPathKey& key) {
        return std::array<double, 3>{
            -std::sin(key.attitude[1]) * std::cos(key.attitude[0]),
            std::sin(key.attitude[0]),
            -std::cos(key.attitude[1]) * std::cos(key.attitude[0])};
    };
    const auto a_forward = forward(first), b_forward = forward(second);
    const double m = 1.0 - t;
    const std::array<double, 4> weights{m*m*m, 3*m*m*t, 3*m*t*t, t*t*t};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const double a = first.position[axis], b = second.position[axis];
        state.position[axis] = weights[0] * a +
            weights[1] * (a + reach * a_forward[axis]) +
            weights[2] * (b - reach * b_forward[axis]) + weights[3] * b;
    }
    state.position[3] = 1.0;
    return true;
}

} // namespace

bool begin_ac5_mission_steering_order(
    const Ac5MissionOrder& order, double ticks_per_second,
    Ac5MissionSteeringState& state) noexcept {
    if (order.type != 2 || !std::isfinite(ticks_per_second) ||
        ticks_per_second <= 0.0 || !valid_state(state)) return false;
    const auto mode = waypoint_mode(order);
    if (mode > 8) return false;
    Ac5MissionSteeringState next = state;
    next.active_mode = mode;
    next.waypoint_order_started = true;
    next.waypoint_complete = false;
    next.path_segment = 0;
    const std::uint32_t payload_flags = order.raw_words[13] >> 16;
    const bool was_special = (next.flags & 0x3cu) != 0;
    set_flag(next.flags, 0x0001u, mode == 1 || mode == 2);
    set_flag(next.flags, 0x0002u, mode >= 1 && mode <= 3);
    set_flag(next.flags, 0x0040u, mode <= 3);
    set_flag(next.flags, 0x0080u,
             mode == 1 || (mode == 6 && (payload_flags & 0x200u) != 0));
    set_flag(next.flags, 0x0004u, mode == 4);
    set_flag(next.flags, 0x0008u, mode == 5);
    set_flag(next.flags, 0x0020u, mode == 7);
    set_flag(next.flags, 0x0100u, (payload_flags & 0x001u) != 0);
    set_flag(next.flags, 0x0010u, (payload_flags & 0x020u) != 0);
    set_flag(next.flags, 0x1000u, (payload_flags & 0x002u) != 0);
    set_flag(next.flags, 0x2000u, (payload_flags & 0x100u) != 0);
    const bool is_special = (next.flags & 0x3cu) != 0;
    if (was_special != is_special) set_flag(next.flags, 0x0400u, is_special);
    next.waypoint_parameter_3c = static_cast<std::uint8_t>(order.raw_words[15]);
    next.waypoint_parameter_3d = static_cast<std::uint8_t>(order.raw_words[15] >> 8);
    if (mode != 6) {
        next.order_start_position = next.position;
        next.order_start_attitude = next.attitude;
    }
    const double target_speed = speed_per_tick(target_speed_kmh(order), ticks_per_second);
    if ((next.flags & 0x0008u) == 0) next.speed_world_units_per_tick = target_speed;
    if (mode == 7) {
        next.velocity = {0.0, -target_speed, 0.0, 0.0};
    } else if (mode == 6) {
        next.velocity = {0.0, 0.0, 0.0, 1.0};
        next.order_start_position = next.position;
        next.order_start_attitude = next.attitude;
    }
    state = next;
    return true;
}

bool advance_ac5_mission_steering(
    const Ac5MissionOrder& order, const Ac5MissionSteeringInput& input,
    Ac5MissionSteeringState& state) noexcept {
    if (order.type != 2 || !state.waypoint_order_started ||
        state.active_mode != waypoint_mode(order) ||
        !std::isfinite(input.ticks_per_second) || input.ticks_per_second <= 0.0 ||
        !valid_state(state)) return false;

    const auto mode = state.active_mode;
    if (mode != 5 && mode != 7 && mode != 8 &&
        !std::all_of(input.target_position.begin(), input.target_position.end(),
                     [](double value) { return std::isfinite(value); })) return false;
    const std::uint32_t flags = order.raw_words[13] >> 16;
    Ac5MissionSteeringState next = state;

    if (mode == 8) {
        if (!order.path || !advance_keyframed_path(*order.path, input, next)) return false;
    } else if (mode == 7) {
        if (!std::isfinite(order.float_words[6]) || order.float_words[6] < 0.0f)
            return false;
        next.velocity[1] -= static_cast<double>(order.float_words[6]) * 5.12 /
            (input.ticks_per_second * input.ticks_per_second);
        for (std::size_t axis = 0; axis < 3; ++axis)
            next.position[axis] += next.velocity[axis];
    } else if (mode == 5) {
        const double acceleration = order.float_words[6];
        if (!std::isfinite(acceleration) || acceleration < 0.0) return false;
        const double target_speed = speed_per_tick(target_speed_kmh(order),
                                                   input.ticks_per_second);
        next.speed_world_units_per_tick = std::min(target_speed,
            next.speed_world_units_per_tick + acceleration * 5.12 /
                (input.ticks_per_second * input.ticks_per_second));
        next.attitude[0] = next.speed_world_units_per_tick <
                speed_per_tick(200, input.ticks_per_second)
            ? 0.0 : std::min(next.attitude[0] +
                (10.0 * pi / 180.0) / input.ticks_per_second,
                30.0 * pi / 180.0);
        std::array<double, 3> displacement{};
        if (!forward_displacement(next.attitude, next.speed_world_units_per_tick,
                                 displacement)) return false;
        for (std::size_t axis = 0; axis < 3; ++axis)
            next.position[axis] += displacement[axis];
    } else {
        double desired_pitch = 0.0, desired_yaw = 0.0;
        if (!aim_angles(next.position, input.target_position,
                        desired_pitch, desired_yaw)) return false;
        if (mode == 4) {
            const double turn_rate = order.float_words[5];
            if (!std::isfinite(turn_rate) || turn_rate < 0.0) return false;
            const double difference = fold_angle(desired_yaw - next.attitude[1]);
            const double maximum_turn = turn_rate * pi / 180.0 /
                input.ticks_per_second;
            if (std::abs(difference) > maximum_turn) {
                next.attitude[1] = fold_angle(next.attitude[1] +
                    std::copysign(maximum_turn, difference));
                next.speed_world_units_per_tick = 0.0;
            } else {
                next.attitude[1] = desired_yaw;
                next.speed_world_units_per_tick = speed_per_tick(
                    target_speed_kmh(order), input.ticks_per_second);
            }
            next.attitude[0] = desired_pitch;
        } else {
            next.attitude[0] = desired_pitch;
            next.attitude[1] = desired_yaw;
        }
        std::array<double, 3> displacement{};
        if (!forward_displacement(next.attitude, next.speed_world_units_per_tick,
                                 displacement)) return false;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            next.position[axis] += displacement[axis];
            if (mode == 6) next.velocity[axis] += displacement[axis];
        }
    }

    if (mode != 7 && mode != 8 && (flags & (1u << 7)) != 0) {
        const std::array<double, 4> distance{
            order.world_position[0] - next.order_start_position[0],
            order.world_position[1] - next.order_start_position[1],
            order.world_position[2] - next.order_start_position[2],
            -next.order_start_position[3]};
        const auto length4 = [](const std::array<double, 4>& value) {
            double sum = 0.0;
            for (double component : value) sum += component * component;
            return std::sqrt(sum);
        };
        const double denominator = length4(distance);
        const double amount = length4(next.velocity) / denominator;
        if (std::isfinite(amount)) {
            const std::array<double, 3> arrival{
                order.float_words[9] * pi / 180.0,
                order.float_words[10] * pi / 180.0,
                order.float_words[11] * pi / 180.0};
            if (!std::all_of(arrival.begin(), arrival.end(),
                             [](double value) { return std::isfinite(value); }))
                return false;
            next.attitude = blend_attitude(next.order_start_attitude,
                                           arrival, amount);
        }
    }

    if (!valid_state(next)) return false;
    state = next;
    return true;
}

} // namespace ac5
