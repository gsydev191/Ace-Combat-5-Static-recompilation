#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#include <cwchar>
#endif

namespace ac5 {

inline bool command_line_arguments_utf8(int argc, char** argv,
                                        std::vector<std::string>& output,
                                        std::string& error) {
    output.clear();
#ifdef _WIN32
    (void)argc;
    (void)argv;
    int wide_argc = 0;
    LPWSTR* wide_argv = CommandLineToArgvW(GetCommandLineW(), &wide_argc);
    if (!wide_argv) {
        error = "cannot read the Windows command line";
        return false;
    }
    output.reserve(static_cast<std::size_t>(wide_argc));
    for (int i = 0; i < wide_argc; ++i) {
        const std::size_t wide_size = std::wcslen(wide_argv[i]);
        if (wide_size == 0) {
            output.emplace_back();
            continue;
        }
        if (wide_size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            LocalFree(wide_argv);
            output.clear();
            error = "a command-line argument is too long";
            return false;
        }
        const int wide_length = static_cast<int>(wide_size);
        const int byte_count = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS,
                                                   wide_argv[i], wide_length,
                                                   nullptr, 0, nullptr, nullptr);
        if (byte_count <= 0) {
            LocalFree(wide_argv);
            output.clear();
            error = "cannot convert a Windows command-line argument to UTF-8";
            return false;
        }
        std::string argument(static_cast<std::size_t>(byte_count), '\0');
        if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide_argv[i], wide_length,
                                argument.data(), byte_count, nullptr, nullptr) != byte_count) {
            LocalFree(wide_argv);
            output.clear();
            error = "cannot convert a Windows command-line argument to UTF-8";
            return false;
        }
        output.push_back(std::move(argument));
    }
    LocalFree(wide_argv);
    return true;
#else
    (void)error;
    output.reserve(static_cast<std::size_t>(argc));
    for (int i = 0; i < argc; ++i) output.emplace_back(argv[i]);
    return true;
#endif
}

} // namespace ac5
