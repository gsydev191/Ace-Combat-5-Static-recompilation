#pragma once

#include "data_archive.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ac5 {

struct Ac5MissionMember {
    std::size_t source_offset = 0;
    std::array<float, 3> formation_offset{};
    std::uint32_t flags = 0;
    std::string object_name;
    std::string callsign;
    float hit_radius = 0.0f;
    std::uint16_t kill_points = 0;
    std::int16_t model_index = -1;
    std::uint16_t object_record_id = 0;
    std::uint8_t hit_points = 0;
    std::uint8_t hit_type = 0;
    std::uint8_t object_class = 0;
    std::uint8_t movement_kind = 0;
};

struct Ac5MissionPathKey {
    std::array<float, 3> position{};
    std::array<float, 3> attitude{};
};

struct Ac5MissionPath {
    float tangent = 0.0f;
    float z_start = 0.0f;
    float z_margin = 0.0f;
    std::vector<Ac5MissionPathKey> keys;
};

struct Ac5MissionOrder {
    std::size_t source_offset = 0;
    std::uint8_t type = 0;
    std::array<std::uint32_t, 24> raw_words{};
    std::array<float, 24> float_words{};
    bool has_world_position = false;
    std::array<float, 3> world_position{};
    float radius = 0.0f;
    float distance = 0.0f;
    float heading_degrees = 0.0f;
    // Trigger fields are used by type-0 spawn gates and type-1 Act triggers.
    bool has_trigger = false;
    float trigger_seconds = 0.0f;
    std::array<float, 3> trigger_position{};
    float trigger_radius = 0.0f;
    std::int16_t trigger_value = 0;
    std::uint16_t trigger_state = 0;
    std::uint8_t trigger_kind = 0;
    std::uint8_t trigger_phase = 0;
    std::uint8_t trigger_movement_mode = 0;
    std::uint8_t trigger_group = 0xff;
    std::uint8_t trigger_member = 0xff;
    std::optional<Ac5MissionPath> path;
};

// Retains the source script hierarchy: unit -> sets -> acts -> orders.
struct Ac5MissionUnit {
    std::size_t index = 0;
    std::uint8_t side = 0;
    std::vector<Ac5MissionMember> members;
    std::vector<std::vector<std::vector<Ac5MissionOrder>>> sets;
};

enum class Ac5MissionTriggerMode { spawn_gate, act_trigger };

struct Ac5MissionTriggerContext {
    std::uint8_t phase = 0;
    std::uint64_t frame = 0;
    float ticks_per_second = 60.0f;
    std::array<float, 3> player_position{};
    bool player_active = false;
    std::unordered_map<std::uint16_t, std::int32_t> states;
    // Returns whether the referenced group/member is still active.
    std::function<bool(std::uint8_t, std::uint8_t)> target_is_active;
};

struct Ac5MissionSpawnContext {
    double model_altitude_offset = 0.0;
    bool terrain_height_available = false;
    double terrain_height_at_spawn = 0.0;
    bool relative_target_available = false;
    std::array<double, 4> relative_target_position{};
    double relative_target_yaw = 0.0;
};

struct Ac5MissionSpawnTransform {
    std::array<double, 4> position{};
    // Pitch, yaw, roll in radians.
    std::array<double, 3> attitude{};
};

struct Ac5MissionOrderRuntime {
    std::size_t order_index = 0;
    std::optional<std::size_t> counted_loop_marker;
    std::uint8_t counted_loop_remaining = 0;
    std::uint64_t elapsed_order_ticks = 0;
    std::uint64_t order_serial = 0;
};

struct Ac5MissionUnitRuntime {
    std::int32_t current_act = -1;
    Ac5MissionOrderRuntime order;
    std::unordered_set<std::uint64_t> fired_spawn_triggers;
    std::uint64_t order_serial = 0;
};

struct Ac5MissionActStart {
    std::size_t act_index = 0;
    bool triggered = false;
    bool initial_spawn = false;
    std::uint8_t movement_mode = 0;
};

enum class Ac5MissionOrderStep { waiting, advanced, act_complete, unsupported };

struct Ac5MissionStopTarget {
    std::array<float, 3> position{};
    std::uint8_t side = 0;
    bool active = false;
};

struct Ac5MissionOrderContext {
    float ticks_per_second = 60.0f;
    std::uint8_t unit_side = 0;
    std::array<float, 3> unit_position{};
    std::span<const Ac5MissionStopTarget> stop_targets;
    std::array<double, 4> waypoint_unit_position{};
    double model_altitude_offset = 0.0;
    bool terrain_height_available = false;
    double terrain_height_at_waypoint = 0.0;
    bool follow_target_available = false;
    bool follow_target_active = false;
    std::array<double, 4> follow_target_position{};
    // Evaluated after the ballistic movement step, matching the source branch.
    bool below_ground_after_ballistic_movement = false;
    bool keyframed_path_complete = false;
};

// Advance one recovered unit order, including stop waits, jumps, and flags.
// Order indices and loop state are caller-owned so each live mission unit can
// advance independently without renderer or legacy-runtime objects.
[[nodiscard]] Ac5MissionOrderStep step_ac5_mission_order(
    std::span<const Ac5MissionOrder> orders, Ac5MissionOrderRuntime& runtime,
    std::unordered_map<std::uint16_t, std::int32_t>& states,
    const Ac5MissionOrderContext& context = {});

// Evaluate a decoded type-0 spawn gate or type-1 triggered-Act condition.
// Missing runtime inputs (player or target activity) never trigger an Act.
[[nodiscard]] bool evaluate_ac5_mission_trigger(
    const Ac5MissionOrder& order, Ac5MissionTriggerMode mode,
    const Ac5MissionTriggerContext& context);

// Resolve type-0 spawn orders with absolute, additive, or yaw-rotated
// target-relative placement modes at payload +0x3c. Terrain, model altitude,
// and target inputs are explicit.
[[nodiscard]] bool resolve_ac5_mission_spawn_transform(
    const Ac5MissionOrder& order, const Ac5MissionSpawnContext& context,
    Ac5MissionSpawnTransform& transform, std::string& error);

// Return eligible Act indices in source order. Selection/activation remains
// the caller's responsibility because the game applies additional unit state.
[[nodiscard]] std::vector<std::size_t> eligible_ac5_mission_acts(
    std::span<const std::vector<Ac5MissionOrder>> acts,
    const Ac5MissionTriggerContext& context);

// Select an Act with source-order arbitration and remember spawn gates already
// consumed for this phase (or permanently for state-triggered spawn Acts).
[[nodiscard]] std::optional<Ac5MissionActStart> select_ac5_mission_act(
    std::span<const std::vector<Ac5MissionOrder>> acts,
    Ac5MissionUnitRuntime& runtime,
    const Ac5MissionTriggerContext& context);

// Decode the CUnitTblBin child of a decompressed mission .mmb block. This
// returns native data only; it does not create renderer or PS2 runtime objects.
[[nodiscard]] bool decode_ac5_mission_units(
    std::span<const std::uint8_t> mmb,
    std::vector<Ac5MissionUnit>& units,
    std::string& error);

// Load the sparse slot-0 MMB from a decompressed DATA.PAC mission-pack member,
// then decode its CUnitTblBin. The archive is read-only and remains independent
// of the renderer.
[[nodiscard]] bool load_ac5_mission_units(
    const DataArchive& archive, std::size_t pack_member,
    std::vector<Ac5MissionUnit>& units,
    std::string& error);

} // namespace ac5
