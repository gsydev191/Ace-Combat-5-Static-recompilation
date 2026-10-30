#include "data_archive.hpp"

#include "ulz.hpp"
#include "utf8_path.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <limits>
#include <new>
#include <span>
#include <stdexcept>
#include <sstream>
#include <utility>

namespace ac5 {

namespace {

std::uint32_t read_u32_le(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) |
           (static_cast<std::uint32_t>(data[3]) << 24);
}

std::string normalize_name(std::string_view name) {
    std::string normalized;
    normalized.reserve(name.size());
    for (unsigned char c : name) {
        if (c == '\\') c = '/';
        if (c >= 'A' && c <= 'Z') c = static_cast<unsigned char>(c + ('a' - 'A'));
        normalized.push_back(static_cast<char>(c));
    }
    return normalized;
}

bool read_exact(const std::filesystem::path& path, std::uint64_t offset,
                std::span<std::uint8_t> output, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot open file: " + path_to_utf8(path);
        return false;
    }
    const std::uint64_t max_offset =
        static_cast<std::uint64_t>(std::numeric_limits<std::streamoff>::max());
    const std::uint64_t read_size = static_cast<std::uint64_t>(output.size());
    if (offset > max_offset || read_size > max_offset - offset ||
        read_size > static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max())) {
        error = "file range is too large: " + path_to_utf8(path);
        return false;
    }
    if (output.empty()) return true;
    file.seekg(static_cast<std::streamoff>(offset));
    if (!file || !file.read(reinterpret_cast<char*>(output.data()),
                             static_cast<std::streamsize>(output.size()))) {
        error = "short read from file: " + path_to_utf8(path);
        return false;
    }
    return true;
}

} // namespace

bool ArchiveMemberLoad::begin(const std::filesystem::path& path,
                              std::uint32_t offset, std::uint32_t packed_size,
                              std::uint32_t unpacked_size, std::string& error) {
    packed_.clear();
    expected_size_ = unpacked_size;
    initialized_ = false;
    finished_ = false;
    failed_ = false;
    try {
        packed_.resize(packed_size);
    } catch (const std::bad_alloc&) {
        error = "DATA.PAC member allocation failed";
        failed_ = true;
        return false;
    } catch (const std::length_error&) {
        error = "DATA.PAC member size exceeds the platform's vector limit";
        failed_ = true;
        return false;
    }
    if (!read_exact(path, offset, packed_, error)) {
        failed_ = true;
        return false;
    }
    try {
        if (!decoder_.begin(packed_, error)) {
            failed_ = true;
            return false;
        }
    } catch (const std::bad_alloc&) {
        error = "ULZ decoder allocation failed";
        failed_ = true;
        return false;
    } catch (const std::length_error&) {
        error = "ULZ decoder size exceeds the platform's vector limit";
        failed_ = true;
        return false;
    }
    initialized_ = true;
    if (decoder_.finished()) {
        if (decoder_.output().size() != expected_size_) {
            error = "decoded DATA.PAC member size does not match DATA.TBL";
            failed_ = true;
            return false;
        }
        finished_ = true;
    }
    return true;
}

bool ArchiveMemberLoad::step(std::size_t max_control_words, std::string& error) {
    if (!initialized_ || failed_) {
        error = "archive member load is not active";
        return false;
    }
    if (finished_) return true;
    if (!decoder_.step(max_control_words, error)) {
        failed_ = true;
        return false;
    }
    if (decoder_.finished()) {
        if (decoder_.output().size() != expected_size_) {
            error = "decoded DATA.PAC member size does not match DATA.TBL";
            failed_ = true;
            return false;
        }
        finished_ = true;
    }
    return true;
}

bool ArchiveMemberLoad::take_output(std::vector<std::uint8_t>& output,
                                   std::string& error) {
    if (!initialized_ || failed_ || !finished_) {
        error = "archive member output is not complete";
        return false;
    }
    if (!decoder_.take_output(output, error)) return false;
    initialized_ = false;
    finished_ = false;
    return true;
}

bool DataArchive::open(const std::filesystem::path& table_path,
                       const std::filesystem::path& archive_path,
                       std::string& error) {
    std::ifstream table(table_path, std::ios::binary | std::ios::ate);
    if (!table) {
        error = "cannot open DATA.TBL: " + path_to_utf8(table_path);
        return false;
    }
    const auto table_size = table.tellg();
    if (table_size < 8) {
        error = "DATA.TBL is shorter than its header";
        return false;
    }
    table.seekg(0);
    std::array<std::uint8_t, 8> header{};
    if (!table.read(reinterpret_cast<char*>(header.data()), header.size())) {
        error = "cannot read DATA.TBL header";
        return false;
    }
    const std::uint32_t count = read_u32_le(header.data());
    const std::uint64_t expected_size = 8ull + 12ull * count;
    if (expected_size != static_cast<std::uint64_t>(table_size)) {
        error = "DATA.TBL size does not match its member count";
        return false;
    }
    if (expected_size - 8u >
        static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max())) {
        error = "DATA.TBL records exceed the platform's readable stream size";
        return false;
    }
    if (count > (std::numeric_limits<std::size_t>::max() / sizeof(MemberEntry))) {
        error = "DATA.TBL member count is too large";
        return false;
    }

    std::vector<std::uint8_t> records;
    std::vector<MemberEntry> members;
    try {
        records.resize(static_cast<std::size_t>(count) * 12u);
        members.resize(count);
    } catch (const std::bad_alloc&) {
        error = "DATA.TBL record allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "DATA.TBL record count exceeds the platform's vector limit";
        return false;
    }
    if (!table.read(reinterpret_cast<char*>(records.data()),
                    static_cast<std::streamsize>(records.size()))) {
        error = "cannot read DATA.TBL records";
        return false;
    }
    const std::size_t sizes_at = static_cast<std::size_t>(count) * 8u;
    for (std::size_t i = 0; i < count; ++i) {
        members[i] = {read_u32_le(records.data() + i * 8u),
                      read_u32_le(records.data() + i * 8u + 4u),
                      read_u32_le(records.data() + sizes_at + i * 4u)};
    }

    std::error_code ec;
    const std::uint64_t archive_size = std::filesystem::file_size(archive_path, ec);
    if (ec) {
        error = "cannot stat DATA.PAC: " + path_to_utf8(archive_path);
        return false;
    }
    for (const MemberEntry& member : members) {
        const std::uint64_t end = static_cast<std::uint64_t>(member.offset) + member.packed_size;
        if (end > archive_size) {
            error = "DATA.TBL member range extends past DATA.PAC";
            return false;
        }
    }
    archive_path_ = archive_path;
    members_ = std::move(members);
    names_.clear();
    names_by_slot_.clear();
    ++generation_;
    if (generation_ == 0) ++generation_;
    return true;
}

bool DataArchive::load_names(const std::filesystem::path& names_path, std::string& error) {
    std::ifstream file(names_path);
    if (!file) {
        error = "cannot open archive name table: " + path_to_utf8(names_path);
        return false;
    }
    std::unordered_multimap<std::string, NamedFile> names;
    std::unordered_map<std::uint64_t, std::string> names_by_slot;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream row(line);
        std::size_t member = 0, index = 0;
        std::string name;
        if (!(row >> member >> index >> name) || name.empty() ||
            (!line.empty() && line[0] == '#') || member >= members_.size() ||
            index > std::numeric_limits<std::uint32_t>::max()) continue;
        names.emplace(normalize_name(name), NamedFile{member, index});
        names_by_slot.insert_or_assign((static_cast<std::uint64_t>(member) << 32) | index, name);
    }
    if (file.bad()) {
        error = "error while reading archive name table: " + path_to_utf8(names_path);
        return false;
    }
    names_ = std::move(names);
    names_by_slot_ = std::move(names_by_slot);
    return true;
}

bool DataArchive::begin_read_member(std::size_t member_index, ArchiveMemberLoad& load,
                                    std::string& error) const {
    if (member_index >= members_.size()) {
        error = "member index is outside DATA.TBL";
        return false;
    }
    const MemberEntry& entry = members_[member_index];
    if (!load.begin(archive_path_, entry.offset, entry.packed_size,
                    entry.unpacked_size, error)) return false;
    return true;
}

bool DataArchive::read_member(std::size_t member_index,
                              std::vector<std::uint8_t>& output,
                              std::string& error) const {
    ArchiveMemberLoad load;
    if (!begin_read_member(member_index, load, error)) return false;
    while (!load.finished()) {
        if (!load.step(2500, error)) return false;
    }
    return load.take_output(output, error);
}

bool DataArchive::read_file(std::size_t member_index, std::size_t file_index,
                            std::vector<std::uint8_t>& output,
                            std::string& error) const {
    std::vector<std::uint8_t> member;
    if (!read_member(member_index, member, error)) return false;
    if (member.size() < 4) {
        error = "DATA.PAC member has no inner file table";
        return false;
    }
    const std::uint32_t count = read_u32_le(member.data());
    const std::uint64_t table_end = 4ull + 4ull * count;
    if (count > 0x10000u || table_end > member.size()) {
        error = "DATA.PAC inner file table extends past its member";
        return false;
    }
    if (file_index >= count) {
        error = "file index is outside the DATA.PAC member";
        return false;
    }
    const std::uint32_t offset = read_u32_le(member.data() + 4u + file_index * 4u);
    if (offset == 0) {
        error = "requested DATA.PAC file slot is empty";
        return false;
    }
    if (offset < table_end || offset >= member.size()) {
        error = "DATA.PAC inner file offset is invalid";
        return false;
    }
    std::vector<std::uint32_t> offsets;
    offsets.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint32_t candidate = read_u32_le(member.data() + 4u + i * 4u);
        if (candidate && (candidate < table_end || candidate >= member.size())) {
            error = "DATA.PAC inner file table contains an invalid offset";
            return false;
        }
        if (candidate) offsets.push_back(candidate);
    }
    std::sort(offsets.begin(), offsets.end());
    const auto next = std::upper_bound(offsets.begin(), offsets.end(), offset);
    const std::size_t end = next == offsets.end() ? member.size() : *next;
    output.assign(member.begin() + offset, member.begin() + static_cast<std::ptrdiff_t>(end));
    return true;
}

bool DataArchive::read_named_file(std::size_t member, std::string_view name,
                                  std::vector<std::uint8_t>& output,
                                  std::string& error) const {
    const std::string normalized = normalize_name(name);
    const auto range = names_.equal_range(normalized);
    std::size_t match = std::numeric_limits<std::size_t>::max();
    for (auto it = range.first; it != range.second; ++it) {
        if (it->second.member != member) continue;
        if (match != std::numeric_limits<std::size_t>::max() && match != it->second.file) {
            error = "archive path is ambiguous in the requested member: " + std::string(name);
            return false;
        }
        match = it->second.file;
    }
    if (match != std::numeric_limits<std::size_t>::max())
        return read_file(member, match, output, error);
    error = "archive name was not found for the requested member: " + std::string(name);
    return false;
}

bool DataArchive::split_member(std::size_t member_index,
                               std::vector<ArchiveFile>& files,
                               std::string& error) const {
    std::vector<std::uint8_t> member;
    if (!read_member(member_index, member, error)) return false;
    if (member.size() < 4) {
        error = "DATA.PAC member has no inner file table";
        return false;
    }
    const std::uint32_t count = read_u32_le(member.data());
    const std::uint64_t table_end = 4ull + 4ull * count;
    if (count > 0x10000u || table_end > member.size()) {
        error = "DATA.PAC inner file table is invalid";
        return false;
    }
    std::vector<std::uint32_t> offsets(count);
    std::vector<std::uint32_t> sorted_offsets;
    sorted_offsets.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint32_t offset = read_u32_le(member.data() + 4u + i * 4u);
        if (offset && (offset < table_end || offset >= member.size())) {
            error = "DATA.PAC inner file table contains an invalid offset";
            return false;
        }
        offsets[i] = offset;
        if (offset) sorted_offsets.push_back(offset);
    }
    std::sort(sorted_offsets.begin(), sorted_offsets.end());

    std::vector<ArchiveFile> result;
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        const std::uint32_t offset = offsets[i];
        if (!offset) continue;
        const auto next = std::upper_bound(sorted_offsets.begin(), sorted_offsets.end(), offset);
        const std::size_t end = next == sorted_offsets.end() ? member.size() : *next;
        ArchiveFile file{i, {}};
        file.contents.assign(member.begin() + offset,
                             member.begin() + static_cast<std::ptrdiff_t>(end));
        result.push_back(std::move(file));
    }
    files = std::move(result);
    return true;
}

std::string DataArchive::file_name(std::size_t member, std::size_t file) const {
    if (member > std::numeric_limits<std::uint32_t>::max() ||
        file > std::numeric_limits<std::uint32_t>::max()) return {};
    const std::uint64_t key = (static_cast<std::uint64_t>(member) << 32) | file;
    const auto found = names_by_slot_.find(key);
    return found == names_by_slot_.end() ? std::string{} : found->second;
}

bool ArchiveBatchLoad::begin(std::string& error) {
    if (status_ != ArchiveBatchStatus::ready) {
        error = "archive batch load has already been started";
        return false;
    }
    if (!plan_.matches(archive_)) {
        error = "archive load plan does not match the currently open archive";
        status_ = ArchiveBatchStatus::failed;
        return false;
    }
    outputs_.resize(plan_.size());
    next_member_ = 0;
    current_started_ = false;
    status_ = plan_.empty() ? ArchiveBatchStatus::complete : ArchiveBatchStatus::active;
    return true;
}

bool ArchiveBatchLoad::step(std::size_t max_control_words, std::string& error) {
    if (status_ == ArchiveBatchStatus::complete) return true;
    if (status_ != ArchiveBatchStatus::active) {
        error = status_ == ArchiveBatchStatus::failed
            ? "archive batch load is in a failed state"
            : "archive batch load has not been started";
        return false;
    }
    if (!plan_.matches(archive_)) {
        error = "DATA.TBL or DATA.PAC changed during archive batch loading";
        status_ = ArchiveBatchStatus::failed;
        return false;
    }

    if (!current_started_) {
        if (!archive_.begin_read_member(plan_[next_member_], current_load_, error)) {
            status_ = ArchiveBatchStatus::failed;
            return false;
        }
        current_started_ = true;
    }
    if (!current_load_.finished() && !current_load_.step(max_control_words, error)) {
        status_ = ArchiveBatchStatus::failed;
        return false;
    }
    if (!current_load_.finished()) return true;
    if (!current_load_.take_output(outputs_[next_member_], error)) {
        status_ = ArchiveBatchStatus::failed;
        return false;
    }
    current_started_ = false;
    ++next_member_;
    if (next_member_ == plan_.size()) status_ = ArchiveBatchStatus::complete;
    return true;
}

bool ArchiveBatchLoad::take_outputs(
    std::vector<std::vector<std::uint8_t>>& outputs, std::string& error) {
    if (status_ != ArchiveBatchStatus::complete) {
        error = "archive batch outputs are not complete";
        return false;
    }
    outputs = std::move(outputs_);
    current_load_ = ArchiveMemberLoad{};
    next_member_ = 0;
    current_started_ = false;
    status_ = ArchiveBatchStatus::ready;
    return true;
}

} // namespace ac5
