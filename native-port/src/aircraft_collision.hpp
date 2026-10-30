#pragma once

#include "data_archive.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>

namespace ac5 {

// Recovered airframe/world collision rule: trace both endpoints of each body
// axis through the aircraft origin. A point at or below the sampled surface
// counts as a hit. Extents are [xmin, xmax, ymin, ymax, zmin, zmax] in the
// same world units as position and the surface sampler.
//
// Returns nullopt when an input or sampled surface is non-finite, or when the
// sampler reports that the query could not be served.
[[nodiscard]] std::optional<std::size_t> count_airframe_surface_hits(
    const std::array<double, 3>& position,
    const std::array<std::array<double, 3>, 3>& body_axes,
    const std::array<double, 6>& extents,
    const std::function<bool(double x, double z, double& height)>& sample_surface);

// Read the six collision extents from the player's AHM payload in an AC5
// aircraft mission-entry bundle. The returned values are in game world units.
[[nodiscard]] bool load_ac5_aircraft_hit_extents(
    const DataArchive& archive, std::size_t mission_entry_member,
    std::array<double, 6>& extents, std::string& error);

} // namespace ac5
