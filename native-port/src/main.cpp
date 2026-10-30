#include "application.hpp"
#include "command_line.hpp"

#include <cstdio>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    std::vector<std::string> arguments;
    std::string error;
    if (!ac5::command_line_arguments_utf8(argc, argv, arguments, error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    std::vector<char*> argument_pointers;
    argument_pointers.reserve(arguments.size());
    for (std::string& argument : arguments) argument_pointers.push_back(argument.data());
    Application app;
    return app.run(static_cast<int>(argument_pointers.size()), argument_pointers.data());
}
