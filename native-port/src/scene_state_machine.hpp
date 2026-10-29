#pragma once

#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace ac5 {

// Native equivalent of the scene helper's current-state, substate, and
// deferred-next-state fields. Transition side effects remain with the caller.
template <class State, class Substate = unsigned char>
class SceneStateMachine {
    static_assert(std::is_enum_v<State>, "scene state IDs must be an enum type");
    static_assert(std::is_enum_v<Substate> || std::is_integral_v<Substate>,
                  "scene substate IDs must be enum or integer values");

public:
    struct Transition {
        State previous;
        State current;
    };

    explicit SceneStateMachine(State initial_state)
        : current_state_(std::move(initial_state)) {}

    const State& current_state() const noexcept { return current_state_; }
    const Substate& substate() const noexcept { return substate_; }
    bool has_pending_transition() const noexcept { return pending_state_.has_value(); }

    void set_substate(Substate substate) { substate_ = std::move(substate); }
    void request_transition(State state) { pending_state_ = std::move(state); }
    void cancel_transition() noexcept { pending_state_.reset(); }

    std::optional<Transition> commit_transition() {
        if (!pending_state_) return std::nullopt;
        Transition transition{current_state_, std::move(*pending_state_)};
        current_state_ = transition.current;
        pending_state_.reset();
        substate_ = Substate{};
        return transition;
    }

private:
    State current_state_;
    Substate substate_{};
    std::optional<State> pending_state_;
};

template <class State, class Substate, class Context>
class SceneDispatcher {
    static_assert(std::is_enum_v<State>, "scene state IDs must be an enum type");
    static_assert(std::is_enum_v<Substate> || std::is_integral_v<Substate>,
                  "scene substate IDs must be enum or integer values");

public:
    using Machine = SceneStateMachine<State, Substate>;
    using Handler = std::function<void(Context&, Machine&)>;

    // Returns false when this state/substate pair already has a handler.
    bool register_handler(State state, Substate substate, Handler handler) {
        if (!handler) throw std::invalid_argument("scene handler must be callable");
        return handlers_.emplace(std::pair{state, substate}, std::move(handler)).second;
    }

    // Returns false when the machine's current pair has no registered handler.
    bool dispatch(Context& context, Machine& machine) const {
        const auto found = handlers_.find({machine.current_state(), machine.substate()});
        if (found == handlers_.end()) return false;
        found->second(context, machine);
        return true;
    }

private:
    std::map<std::pair<State, Substate>, Handler> handlers_;
};

} // namespace ac5
