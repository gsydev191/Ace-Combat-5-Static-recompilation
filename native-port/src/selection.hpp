#pragma once

#include <cstddef>
#include <optional>

namespace ac5 {

// Advances the selected item for one frame. When both directions are pressed,
// the generated menu routine applies forward then backward, so they cancel for
// any valid current selection. An empty list has no selected item.
inline std::optional<std::size_t> advance_selection(
    std::size_t selected, std::size_t item_count,
    bool forward_pressed, bool backward_pressed) {
    if (item_count == 0) return std::nullopt;
    if (selected >= item_count) selected = 0;

    if (forward_pressed) {
        ++selected;
        if (selected == item_count) selected = 0;
    }
    if (backward_pressed) {
        if (selected == 0) selected = item_count - 1;
        else --selected;
    }
    return selected;
}

} // namespace ac5
