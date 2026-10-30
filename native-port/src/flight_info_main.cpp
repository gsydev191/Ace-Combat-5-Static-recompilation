#include "command_line.hpp"
#include "flight_rules_loader.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace {

void print_axis(const char* name, const ac5::FlightAxisRules& axis) {
    std::printf("%s breakpoints km/h:", name);
    for (const double value : axis.breakpoints_kmh) std::printf(" %.2f", value);
    std::printf("\n%s rates deg/s:", name);
    for (const double value : axis.rates_degrees_per_second) std::printf(" %.2f", value);
    std::printf("\n%s slew gains: %.2f -> %.2f%%\n", name,
                axis.low_input_gain_percent, axis.full_input_gain_percent);
}

} // namespace

int main(int argc, char** argv) {
    std::vector<std::string> arguments;
    std::string error;
    if (!ac5::command_line_arguments_utf8(argc, argv, arguments, error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    if (arguments.size() != 2 || arguments[1] == "--help" || arguments[1] == "-h") {
        std::fprintf(arguments.size() == 2 ? stdout : stderr,
                     "usage: ac5_flight_info <aircraft.pmb>\n");
        return arguments.size() == 2 ? 0 : 2;
    }

    ac5::AircraftFlightRules rules;
    if (!ac5::load_aircraft_flight_rules(arguments[1], rules, error)) {
        std::fprintf(stderr, "Flight rules load failed: %s\n", error.c_str());
        return 1;
    }
    std::printf("speed km/h: cruise %.2f, max %.2f, min %.2f, stall %.2f, "
                "full-control %.2f, dive-limit %.2f, dive-extra %.2f\n",
                rules.speed.cruise_kmh, rules.speed.maximum_kmh,
                rules.speed.minimum_kmh, rules.speed.stall_kmh,
                rules.speed.full_control_kmh, rules.speed.dive_limit_kmh,
                rules.speed.dive_extra_kmh);
    std::printf("spool gains: up %.3f%%, down %.3f%%, climb penalty %.2f%%, "
                "inverted sink %.2f\n",
                rules.speed.spool_up_gain, rules.speed.spool_down_gain,
                rules.speed.climb_penalty_percent, rules.speed.inverted_sink_mps);
    print_axis("pitch", rules.pitch);
    print_axis("roll", rules.roll);
    print_axis("yaw", rules.yaw);
    std::printf("trim: pitch %.3f, yaw %.3f, stall drop %.2f%%\n",
                rules.trim_pitch_gain, rules.trim_yaw_gain,
                rules.stall_drop_percent);
    return 0;
}
