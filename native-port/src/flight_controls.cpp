#include "flight_controls.hpp"

#include <algorithm>
#include <cmath>

namespace ac5 {

EngineControlOutput map_engine_lever(float lever) noexcept {
    if (std::isnan(lever)) lever = 0.0f;
    lever = std::clamp(lever, -1.0f, 1.0f);

    if (lever >= 0.9f)
        return { (lever - 0.9f) / 0.1f, 1.0f, 0.0f };
    if (lever >= -0.6f)
        return { 0.0f, (lever + 0.6f) / 1.5f, 0.0f };
    return { 0.0f, 0.0f, (-0.6f - lever) / 0.4f };
}

} // namespace ac5
