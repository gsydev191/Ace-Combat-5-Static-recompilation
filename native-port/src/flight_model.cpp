#include "flight_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ac5 {

bool update_aircraft_ground_contact(AircraftGroundContactState& state,
                                    bool hit, int ticks_per_second,
                                    bool suppress_effect) noexcept {
    if (hit) {
        const bool fresh = (state.flags & 0x01u) == 0;
        if (fresh) {
            state.flags = static_cast<std::uint8_t>(state.flags | 0x0du);
            state.decay_ticks = 0;
            return !suppress_effect;
        }
        state.flags = static_cast<std::uint8_t>(state.flags & ~0x0cu);
        state.decay_ticks = 0;
        return false;
    }
    const int half_second = std::max(1, ticks_per_second / 2);
    // The original compares before incrementing, so it clears after the
    // half-second threshold has already been reached.
    if (state.decay_ticks < half_second) ++state.decay_ticks;
    else state.flags = static_cast<std::uint8_t>(state.flags & 0xf2u);
    return false;
}
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

void apply_airborne_trim_and_stall(const AircraftFlightRules& rules,
                                   double stall_factor, int ticks_per_second,
                                   const Matrix3& body_matrix,
                                   std::array<double, 3>& rates,
                                   bool apply_trim) {
    std::array<double, 3> reference =
        row_multiply({0.0, 0.0, -1.0}, body_matrix);
    reference[1] -= 0.19999999;
    reference = row_multiply(reference, transpose(body_matrix));
    const double length = std::sqrt(reference[0] * reference[0] +
                                    reference[1] * reference[1] +
                                    reference[2] * reference[2]);
    if (apply_trim && length > 0.0 && std::isfinite(length)) {
        for (double& component : reference) component /= length;
        const double baseline_y = -0.19999999 / std::sqrt(1.04);
        rates[1] += ((-reference[0] * rules.trim_yaw_gain) / 100.0 * 60.0) /
                    ticks_per_second;
        rates[0] += (((reference[1] - baseline_y) * rules.trim_pitch_gain) /
                     100.0 * 60.0) / ticks_per_second;
    }

    if (stall_factor > 0.5) {
        std::array<double, 3> nose = row_multiply({0.0, 0.0, -1.0}, body_matrix);
        const double excess = 2.0 * (stall_factor - 0.5);
        nose[1] -= (excess * excess * rules.stall_drop_percent / 100.0 * 60.0) /
                   ticks_per_second;
        const std::array<double, 3> local = row_multiply(nose, transpose(body_matrix));
        rates[0] = std::atan2(local[1],
                              std::sqrt(local[0] * local[0] + local[2] * local[2]));
        double yaw_rate = std::atan2(local[0], local[2]) + pi;
        if (yaw_rate > pi) yaw_rate -= 2.0 * pi;
        rates[1] = yaw_rate;
        rates[2] = 0.0;
    }
}

void approach_commands(const AircraftFlightInput& input,
                       const std::array<double, 3>& body_euler,
                       double& captured_pitch, double& pitch_command,
                       double& roll_command, double& yaw_command) {
    constexpr double pitch_low = -0.09999999403953552;
    constexpr double pitch_high = 0.09999999403953552;
    constexpr double yaw_scale = -0.7999999523162842;
    constexpr double roll_scale = -0.7853980660438538;
    constexpr double outer_deadzone = 0.06981316208839417;
    constexpr double inner_deadzone = 0.01745329052209854;
    constexpr double roll_gain = 3.141592502593994;
    constexpr double pitch_deadzone = 0.01745329052209854;
    constexpr double pitch_limit = 0.06981316208839417;
    constexpr double pitch_scale = -0.06981316208839417;

    yaw_command = input.roll * yaw_scale;
    if (!(pitch_low < input.pitch && input.pitch < pitch_high)) {
        captured_pitch = body_euler[0];
        pitch_command = input.pitch;
        roll_command = 0.0;
        return;
    }

    const double target_roll = input.roll * roll_scale;
    const double body_roll = body_euler[2];
    double roll_stick = 0.0;
    if (body_roll < target_roll - outer_deadzone) roll_stick = -1.0;
    else if (body_roll < target_roll - inner_deadzone)
        roll_stick = (body_roll - target_roll) * 0.25 * roll_gain / 180.0;
    else if (body_roll < target_roll + inner_deadzone) roll_stick = 0.0;
    else if (body_roll < target_roll + outer_deadzone)
        roll_stick = (body_roll - target_roll) * 0.25 * roll_gain / 180.0;
    else roll_stick = 1.0;
    roll_command = -roll_stick;

    const double ratio = std::sin(body_euler[0] - captured_pitch) /
                         std::cos(std::abs(body_roll));
    const double pitch_error = std::asin(std::clamp(ratio, -1.0, 1.0));
    pitch_command = 0.0;
    if (pitch_error < 0.0) pitch_command = 1.0;
    else if (pitch_error < pitch_deadzone) pitch_command = 0.0;
    else if (pitch_error < pitch_limit) pitch_command = pitch_error / pitch_scale;
}

bool finite(const std::array<double, 3>& values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

} // namespace

std::optional<std::array<double, 3>> aircraft_ground_marker_attitude(
    const AircraftGroundMarkers& markers) noexcept {
    for (const auto& point : markers)
        for (double value : point)
            if (!std::isfinite(value)) return std::nullopt;
    std::array<double, 3> forward{}, lateral{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        forward[axis] = markers[0][axis] -
            (markers[1][axis] + markers[2][axis]) * 0.5;
        lateral[axis] = markers[2][axis] - markers[1][axis];
    }
    const double length = std::hypot(forward[0], forward[1], forward[2]);
    if (!(length > 0.0) || !std::isfinite(length)) return std::nullopt;
    const double yaw = std::atan2(forward[0], -forward[2]);
    const double pitch = std::asin(std::clamp(forward[1] / length, -1.0, 1.0));
    const auto unyawed = row_multiply(lateral, euler_matrix({0.0, -yaw, 0.0}));
    const auto level = row_multiply(unyawed, euler_matrix({-pitch, 0.0, 0.0}));
    if (!(std::hypot(level[0], level[1]) > 0.0)) return std::nullopt;
    return std::array<double, 3>{pitch, yaw,
                                 -std::atan2(level[1], -level[0])};
}

std::optional<AircraftGroundTraceResult> normal_aircraft_ground_trace(
    AircraftFlightState& state, const AircraftGroundTraceModel& model,
    const AircraftGroundTraceTerrain* terrain) {
    AircraftGroundTraceResult result;
    constexpr double roll_limit = 1.570796251296997;
    if (state.euler[2] > roll_limit || state.euler[2] < -roll_limit) return result;
    if (!model.sample || (terrain &&
        (!terrain->height || !terrain->sweep || !terrain->vertical)))
        return std::nullopt;
    const auto channel = [&](double value) -> std::optional<double> {
        return model.gear_state == 2 ? std::optional<double>(value) : std::nullopt;
    };
    const auto pose100 = model.sample(channel(100.0));
    const auto pose50 = model.sample(channel(50.0));
    if (!pose100 || !pose50) return std::nullopt;
    for (const auto* pose : {&*pose100, &*pose50})
        for (const auto& point : *pose)
            for (double value : point)
                if (!std::isfinite(value)) return std::nullopt;
    if (!terrain) return result;

    AircraftFlightState staged_state = state;
    const Matrix3 rotation = euler_matrix(staged_state.euler);
    const auto world = [&](const std::array<double, 3>& point,
                           const Matrix3& matrix) {
        auto output = row_multiply(point, matrix);
        for (std::size_t axis = 0; axis < 3; ++axis)
            output[axis] += staged_state.position[axis];
        return output;
    };
    for (std::size_t index = 0; index < 3; ++index) {
        auto end = (*pose50)[index];
        auto full = (*pose100)[index];
        if (index == 0) end[0] = full[0] = 0.0;
        auto start = end;
        for (std::size_t axis = 0; axis < 3; ++axis)
            start[axis] += 10.0 * (full[axis] - end[axis]);
        end = world(end, rotation);
        start = world(start, rotation);
        const double height = terrain->height(end);
        if (!std::isfinite(height)) return std::nullopt;
        if (end[1] < height) {
            const AircraftGroundSweepHit hit = terrain->sweep(end, start);
            if (hit.result != 0 && hit.normal_y > 0.5 && hit.kind != 2) {
                if (!std::isfinite(hit.fraction)) return std::nullopt;
                result.fractions[index] = hit.fraction;
            }
        }
    }
    const auto& fractions = result.fractions;
    if (!(fractions[0] > 0.0 || fractions[1] > 0.0 || fractions[2] > 0.0))
        return result;
    const auto attitude = aircraft_ground_marker_attitude(*pose50);
    if (!attitude) return std::nullopt;
    auto aligned = to_euler(transpose(euler_matrix(*attitude)));
    aligned[1] = staged_state.euler[1];
    result.anchor = fractions[0] > fractions[1]
        ? (fractions[0] > fractions[2] ? 0 : 2)
        : (fractions[1] > fractions[2] ? 1 : 2);
    result.hit = true;
    staged_state.euler = aligned;

    const auto anchor_pose = model.sample(channel(50.0));
    if (!anchor_pose) return std::nullopt;
    const auto& point = (*anchor_pose)[static_cast<std::size_t>(result.anchor)];
    for (double value : point)
        if (!std::isfinite(value)) return std::nullopt;
    const Matrix3 aligned_rotation = euler_matrix(aligned);
    const auto anchor = world(point, aligned_rotation);
    auto upper = anchor;
    auto lower = anchor;
    upper[1] += 128.0;
    lower[1] -= 128.0;
    const AircraftGroundVerticalHit snap = terrain->vertical(upper, lower);
    if (snap.result == 1) {
        std::array<double, 3> delta{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(snap.point[axis])) return std::nullopt;
            delta[axis] = snap.point[axis] - anchor[axis];
        }
        delta = row_multiply(delta, transpose(aligned_rotation));
        for (std::size_t axis = 0; axis < 3; ++axis)
            staged_state.position[axis] += delta[axis];
    }
    state = staged_state;
    return result;
}

std::array<double, 3> aircraft_forward_direction(
    const std::array<double, 3>& euler) noexcept {
    return row_multiply({0.0, 0.0, -1.0}, euler_matrix(euler));
}

std::array<std::array<double, 3>, 3> aircraft_body_axes(
    const std::array<double, 3>& euler) noexcept {
    return euler_matrix(euler);
}

void set_aircraft_flight_mode(AircraftFlightFlags& flags,
                              std::uint8_t command) noexcept {
    switch (command) {
    case 0:
        flags.mode = static_cast<std::uint8_t>(flags.mode & 0xfeu);
        flags.contact = static_cast<std::uint8_t>(flags.contact & 0xfcu);
        break;
    case 1:
        flags.mode = static_cast<std::uint8_t>(flags.mode | 1u);
        flags.contact = static_cast<std::uint8_t>(flags.contact & 0xfcu);
        break;
    case 2:
        flags.mode = static_cast<std::uint8_t>(flags.mode | 1u);
        flags.contact = static_cast<std::uint8_t>((flags.contact & 0xfeu) | 2u);
        break;
    default:
        break;
    }
}

void set_aircraft_approach_control(AircraftFlightFlags& flags,
                                   bool enabled) noexcept {
    flags.contact = static_cast<std::uint8_t>(enabled
        ? flags.contact | 0x20u
        : flags.contact & static_cast<std::uint8_t>(~0x20u));
}

void advance_aircraft_flight_flags(AircraftFlightFlags& flags) noexcept {
    if ((flags.contact & 0x0fu) == 0x0fu && (flags.mode & 1u) != 0 &&
        (flags.mode & 2u) == 0) {
        flags.contact = static_cast<std::uint8_t>((flags.contact & 0xf1u) | 1u);
    } else if ((flags.contact & 0x03u) == 0x02u && (flags.mode & 1u) != 0) {
        flags.contact = static_cast<std::uint8_t>(flags.contact | 0x0du);
    } else if ((flags.contact & 0x03u) == 0x03u &&
               (flags.mode & 0x03u) == 0x01u) {
        flags.contact = static_cast<std::uint8_t>(flags.contact & 0xfdu);
    }
}

bool advance_aircraft_flight_tick(const AircraftFlightRules& rules,
                                  const AircraftFlightInput& input,
                                  AircraftFlightState& state) noexcept {
    if (!finite(state.position) || !finite(state.euler) ||
        !finite(state.path_euler) || !finite(state.rates) ||
        !std::isfinite(state.speed_units_per_tick) ||
        !std::isfinite(state.path_lag) || !std::isfinite(state.approach_pitch) ||
        !std::isfinite(input.pitch) ||
        !std::isfinite(input.roll) || !std::isfinite(input.yaw) ||
        !std::isfinite(input.throttle_lever) ||
        !std::isfinite(input.contact_acceleration_scale) ||
        !std::isfinite(input.contact_acceleration_bias)) return false;

    const bool ground_contact = input.ground_contact_override.value_or(
        (state.flags.contact & 1u) != 0);
    const bool reduced_speed_mode = (state.flags.mode & 1u) != 0;
    const bool approach_control = (state.flags.contact & 0x20u) != 0;
    const FlightSpeedInput speed_input{
        state.position[1], state.euler[0], input.throttle_lever,
        state.speed_units_per_tick, input.ticks_per_second,
        reduced_speed_mode};
    const FlightSpeedOutput speed = advance_airborne_speed(rules.speed, speed_input);
    if (!speed.valid) return false;

    double ground_speed_cap = speed.maximum_units_per_tick;
    if (ground_contact) {
        FlightSpeedInput ground_cap_input = speed_input;
        ground_cap_input.reduced_speed_mode = true;
        const FlightSpeedOutput ground_cap =
            advance_airborne_speed(rules.speed, ground_cap_input);
        if (!ground_cap.valid) return false;
        ground_speed_cap = ground_cap.maximum_units_per_tick;
    }

    AircraftFlightState next = state;
    if (input.reset_contact_speed) {
        next.speed_units_per_tick = 0.0;
    } else if (ground_contact) {
        const double acceleration = 35.555546 /
            input.ticks_per_second / input.ticks_per_second;
        next.speed_units_per_tick = std::clamp(
            state.speed_units_per_tick + acceleration *
                (input.contact_acceleration_scale * input.throttle_lever +
                 input.contact_acceleration_bias),
            0.0, ground_speed_cap);
    } else {
        next.speed_units_per_tick = speed.speed_units_per_tick;
    }

    const auto axis_input = [&input, &speed, &next](double stick, double current_rate) {
        return FlightAxisInput{stick, next.speed_units_per_tick,
            speed.stall_speed_units_per_tick,
            speed.full_control_speed_units_per_tick, current_rate,
            input.ticks_per_second};
    };
    double pitch_command = input.pitch;
    double roll_command = -input.roll;
    double yaw_command = input.yaw;
    if (approach_control)
        approach_commands(input, state.euler, next.approach_pitch,
                          pitch_command, roll_command, yaw_command);
    const FlightAxisOutput pitch = advance_flight_axis(
        rules.pitch, axis_input(pitch_command, state.rates[0]));
    const FlightAxisOutput roll = advance_flight_axis(
        rules.roll, axis_input(roll_command, state.rates[2]));
    const FlightAxisOutput yaw = advance_flight_axis(
        approach_control ? rules.pitch : rules.yaw,
        axis_input(yaw_command, state.rates[1]));
    if (!pitch.valid || !roll.valid || !yaw.valid) return false;

    next.stall_factor = ground_contact ? 0.0 : speed.stall_factor;
    next.rates = {pitch.rate_radians_per_tick,
                  yaw.rate_radians_per_tick,
                  roll.rate_radians_per_tick};
    if (!ground_contact)
        apply_airborne_trim_and_stall(rules, next.stall_factor,
                                      input.ticks_per_second,
                                      euler_matrix(state.euler), next.rates,
                                      !approach_control);
    if (next.rates[0] != 0.0 || next.rates[1] != 0.0 || next.rates[2] != 0.0)
        next.euler = to_euler(multiply(euler_matrix(next.rates),
                                       euler_matrix(state.euler)));

    next.path_euler = state.path_lag < 1.0
        ? slerp_euler(state.path_euler, next.euler, state.path_lag)
        : next.euler;
    std::array<double, 3> velocity = row_multiply(
        {0.0, 0.0, -next.speed_units_per_tick}, euler_matrix(next.path_euler));
    velocity[1] -= next.stall_factor * next.stall_factor *
                   (reduced_speed_mode ? 2.5 : 5.0);
    if (!input.suppress_inverted_sink && !approach_control) {
        const double normalized_roll = next.euler[2] / pi;
        velocity[1] -= normalized_roll * normalized_roll *
            (rules.speed.inverted_sink_mps * aircraft_world_units_per_meter /
             input.ticks_per_second);
    }
    if (reduced_speed_mode) {
        const double half_cruise = rules.speed.cruise_kmh *
                                   aircraft_world_units_per_kmh_per_second * 0.5 /
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
    advance_aircraft_flight_flags(next.flags);
    state = next;
    return true;
}

bool advance_aircraft_carrier_tick(const AircraftFlightRules& rules,
                                   const AircraftFlightInput& input,
                                   AircraftFlightState& state) noexcept {
    if (input.ticks_per_second != 30 && input.ticks_per_second != 60)
        return false;

    const bool carrier_hold = (state.flags.contact & 0x03u) == 0x03u &&
                              (state.flags.mode & 0x02u) != 0;
    const bool within_hold = carrier_hold &&
        state.carrier_hold_ticks < 2 * input.ticks_per_second;
    AircraftFlightInput tick_input = input;
    const AircraftFlightFlags original_flags = state.flags;
    std::uint8_t next_hold_ticks = state.carrier_hold_ticks;

    if (within_hold) {
        tick_input.throttle_lever = 0.0;
        if (input.throttle_lever == 1.0) {
            if (next_hold_ticks < 2 * input.ticks_per_second)
                ++next_hold_ticks;
        } else {
            tick_input.reset_contact_speed = true;
        }
        if (!advance_aircraft_flight_tick(rules, tick_input, state)) return false;
        state.flags = original_flags;
        state.carrier_hold_ticks = next_hold_ticks;
        return true;
    }

    const bool carrier_prehold = (state.flags.contact & 0x03u) == 0x01u &&
                                 (state.flags.mode & 0x02u) != 0;
    if (carrier_hold || carrier_prehold)
        tick_input.contact_acceleration_scale = 1.2;
    if (carrier_prehold)
        tick_input.contact_acceleration_bias = -2.0;
    return advance_aircraft_flight_tick(rules, tick_input, state);
}

} // namespace ac5
