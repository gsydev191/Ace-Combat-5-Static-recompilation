#include "data_archive.hpp"
#include "utf8_path.hpp"

#include <charconv>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

using namespace ac5;

namespace {

bool parse_index(const char* text, std::size_t& value) {
    const std::string_view input(text);
    std::uint64_t parsed = 0;
    const auto result = std::from_chars(input.data(), input.data() + input.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != input.data() + input.size() ||
        parsed > static_cast<std::uint64_t>(SIZE_MAX)) return false;
    value = static_cast<std::size_t>(parsed);
    return true;
}

bool write_file(const std::filesystem::path& path,
                const std::vector<std::uint8_t>& data, std::string& error) {
    std::error_code ec;
    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            error = "cannot create output directory: " + path_to_utf8(path.parent_path());
            return false;
        }
    }
    std::ofstream output(path, std::ios::binary);
    if (!output || !output.write(reinterpret_cast<const char*>(data.data()),
                                 static_cast<std::streamsize>(data.size()))) {
        error = "cannot write output file: " + path_to_utf8(path);
        return false;
    }
    return true;
}

std::filesystem::path safe_archive_path(const std::string& name) {
    std::string portable_name = name;
    for (char& c : portable_name) {
        if (c == '\\') c = '/';
        if (static_cast<unsigned char>(c) < 0x20 || c == ':') return {};
    }
    const std::filesystem::path candidate = path_from_utf8(portable_name);
    if (candidate.empty() || candidate.is_absolute() || candidate.has_root_name() ||
        candidate.has_root_directory()) return {};
    for (const auto& part : candidate) {
        const std::string component = path_to_utf8(part);
        if (component.empty() || component == ".." || component == "." ||
            component.back() == '.' || component.back() == ' ') return {};

        std::string device_name = component.substr(0, component.find('.'));
        for (char& c : device_name)
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        if (device_name == "CON" || device_name == "PRN" || device_name == "AUX" ||
            device_name == "NUL" || device_name == "CLOCK$" ||
            (device_name.size() == 4 &&
             (device_name.starts_with("COM") || device_name.starts_with("LPT")) &&
             device_name[3] >= '1' && device_name[3] <= '9')) return {};
    }
    return candidate;
}

bool path_is_within(const std::filesystem::path& root,
                    const std::filesystem::path& candidate) {
    const auto relative = candidate.lexically_relative(root);
    if (relative.empty()) return candidate == root;
    if (relative.has_root_path()) return false;
    const auto first = relative.begin();
    return first != relative.end() && *first != "..";
}

bool ensure_safe_directories(const std::filesystem::path& canonical_root,
                             const std::filesystem::path& directory,
                             std::string& error) {
    const auto relative = directory.lexically_relative(canonical_root);
    if (relative.empty() || relative.has_root_path() ||
        (relative.begin() != relative.end() && *relative.begin() == "..")) {
        error = "archive output directory is outside the selected output root: " +
                path_to_utf8(directory);
        return false;
    }

    std::filesystem::path current = canonical_root;
    std::error_code ec;
    for (const auto& component : relative) {
        current /= component;
        const auto status = std::filesystem::symlink_status(current, ec);
        if (ec == std::errc::no_such_file_or_directory ||
            (!ec && status.type() == std::filesystem::file_type::not_found)) {
            ec.clear();
            if (!std::filesystem::create_directory(current, ec) || ec) {
                error = "cannot create archive output directory: " + path_to_utf8(current);
                return false;
            }
        } else if (ec) {
            error = "cannot inspect archive output directory: " + path_to_utf8(current);
            return false;
        }

        const auto resolved = std::filesystem::weakly_canonical(current, ec);
        if (ec || !path_is_within(canonical_root, resolved)) {
            error = "archive path resolves outside the output directory: " +
                    path_to_utf8(current);
            return false;
        }
        if (!std::filesystem::is_directory(current, ec) || ec) {
            error = "archive output path component is not a directory: " +
                    path_to_utf8(current);
            return false;
        }
    }
    return true;
}

std::string portable_path_key(const std::filesystem::path& path) {
    std::string key;
    for (const auto& component : path) {
        if (!key.empty()) key.push_back('/');
        std::string text = path_to_utf8(component);
        for (char& c : text) {
            if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + ('a' - 'A'));
        }
        key += text;
    }
    return key;
}

bool path_conflicts(const std::string& key,
                    const std::unordered_set<std::string>& used_paths) {
    if (used_paths.contains(key)) return true;
    std::size_t separator = key.find('/');
    while (separator != std::string::npos) {
        if (used_paths.contains(key.substr(0, separator))) return true;
        separator = key.find('/', separator + 1);
    }
    const std::string descendant_prefix = key + '/';
    for (const std::string& used : used_paths) {
        if (used.starts_with(descendant_prefix)) return true;
    }
    return false;
}

bool extract_all(const DataArchive& archive, const std::filesystem::path& output_dir,
                 std::size_t first_member, std::size_t last_member,
                 std::string& error) {
    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
            error = "cannot create output directory: " + path_to_utf8(output_dir);
        return false;
    }
    const auto canonical_root = std::filesystem::weakly_canonical(output_dir, ec);
    if (ec) {
        error = "cannot resolve output directory: " + path_to_utf8(output_dir);
        return false;
    }

    std::uint64_t files_written = 0, bytes_written = 0;
    for (std::size_t member = first_member; member < last_member; ++member) {
        std::vector<ArchiveFile> files;
        if (!archive.split_member(member, files, error)) return false;
        std::unordered_set<std::string> used_paths;
        for (const ArchiveFile& file : files) {
            std::filesystem::path relative = safe_archive_path(archive.file_name(member, file.index));
            std::string key = relative.empty() ? std::string{} : portable_path_key(relative);
            if (relative.empty() || path_conflicts(key, used_paths)) {
                const std::string slot = std::to_string(file.index);
                relative = std::filesystem::path("__ac5_slots") / slot;
                key = portable_path_key(relative);
                for (std::size_t suffix = 1; path_conflicts(key, used_paths); ++suffix) {
                    relative = std::filesystem::path("__ac5_slots") /
                               (slot + "_" + std::to_string(suffix));
                    key = portable_path_key(relative);
                }
            }
            used_paths.insert(std::move(key));
            std::ostringstream folder;
            folder << std::setw(4) << std::setfill('0') << member;
            const auto path = canonical_root / folder.str() / relative;
            if (!ensure_safe_directories(canonical_root, path.parent_path(), error)) return false;
            const auto existing = std::filesystem::symlink_status(path, ec);
            if (!ec && std::filesystem::is_symlink(existing)) {
                error = "refusing to overwrite a symlink in the archive output: " + path_to_utf8(path);
                return false;
            }
            if (ec == std::errc::no_such_file_or_directory) ec.clear();
            if (ec) {
                error = "cannot inspect archive output path: " + path_to_utf8(path);
                return false;
            }
            if (!write_file(path, file.contents, error)) return false;
            ++files_written;
            bytes_written += file.contents.size();
        }
        std::cout << "member " << member << ": " << files.size() << " files\n";
    }
    std::cout << "extracted " << files_written << " files (" << bytes_written
              << " bytes) into " << path_to_utf8(output_dir) << '\n';
    return true;
}

void print_usage() {
    std::cerr << "usage: ac5_extract <DATA.TBL> <DATA.PAC> <member> [file] <output>\n"
                 "       ac5_extract --named <DATA.TBL> <DATA.PAC> <names.txt> "
                 "<member> <internal-path> <output>\n"
                 "       ac5_extract --all <DATA.TBL> <DATA.PAC> <names.txt|-> "
                 "<output-dir> [member]\n";
}

} // namespace

int main(int argc, char** argv) {
    const bool named_mode = argc == 8 && std::string_view(argv[1]) == "--named";
    const bool all_mode = (argc == 6 || argc == 7) && std::string_view(argv[1]) == "--all";
    if (!named_mode && !all_mode && argc != 5 && argc != 6) {
        print_usage();
        return 2;
    }

    const int table_arg = named_mode || all_mode ? 2 : 1;
    DataArchive archive;
    std::string error;
    if (!archive.open(path_from_utf8(argv[table_arg]),
                      path_from_utf8(argv[table_arg + 1]), error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (named_mode && !archive.load_names(path_from_utf8(argv[4]), error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (all_mode && std::string_view(argv[4]) != "-" &&
        !archive.load_names(path_from_utf8(argv[4]), error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (all_mode) {
        std::size_t first = 0, last = archive.member_count();
        if (argc == 7) {
            if (!parse_index(argv[6], first) || first >= last) {
                std::cerr << "member index is outside DATA.TBL\n";
                return 2;
            }
            last = first + 1;
        }
        if (!extract_all(archive, path_from_utf8(argv[5]), first, last, error)) {
            std::cerr << error << '\n';
            return 1;
        }
        return 0;
    }

    const bool extract_inner_file = !named_mode && argc == 6;
    std::size_t member_index = 0, file_index = 0;
    const int member_arg = named_mode ? 5 : 3;
    if (!parse_index(argv[member_arg], member_index) ||
        (extract_inner_file && !parse_index(argv[4], file_index))) {
        std::cerr << "member and file indices must be non-negative integers\n";
        return 2;
    }
    const char* output_name = named_mode ? argv[7] : argv[extract_inner_file ? 5 : 4];
    std::vector<std::uint8_t> data;
    const bool read_ok = named_mode
        ? archive.read_named_file(member_index, argv[6], data, error)
        : extract_inner_file
            ? archive.read_file(member_index, file_index, data, error)
            : archive.read_member(member_index, data, error);
    if (!read_ok) {
        std::cerr << error << '\n';
        return 1;
    }
    if (!write_file(path_from_utf8(output_name), data, error)) {
        std::cerr << error << '\n';
        return 1;
    }
    std::cout << "wrote " << data.size() << " bytes\n";
    return 0;
}
