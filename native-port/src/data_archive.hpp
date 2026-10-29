#pragma once

#include "ulz.hpp"

#include <array>
#include <cstdint>
#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ac5 {

struct ArchiveFile {
    std::size_t index;
    std::vector<std::uint8_t> contents;
};

class ArchiveMemberLoad {
public:
    ArchiveMemberLoad() = default;
    ArchiveMemberLoad(const ArchiveMemberLoad&) = delete;
    ArchiveMemberLoad& operator=(const ArchiveMemberLoad&) = delete;
    ArchiveMemberLoad(ArchiveMemberLoad&&) noexcept = default;
    ArchiveMemberLoad& operator=(ArchiveMemberLoad&&) noexcept = default;

    // Decompression advances by at most max_control_words. The packed bytes are
    // owned by this object and the partial output remains readable between steps.
    // Moving transfers the packed vector allocation together with its decoder view.
    bool step(std::size_t max_control_words, std::string& error);
    bool active() const { return initialized_ && !failed_ && !finished_; }
    bool finished() const { return finished_; }
    bool failed() const { return failed_; }
    const std::vector<std::uint8_t>& output() const { return decoder_.output(); }
    bool take_output(std::vector<std::uint8_t>& output, std::string& error);

private:
    friend class DataArchive;
    bool begin(const std::filesystem::path& path, std::uint32_t offset,
               std::uint32_t packed_size, std::uint32_t unpacked_size,
               std::string& error);

    std::vector<std::uint8_t> packed_;
    UlzDecoder decoder_;
    std::uint32_t expected_size_ = 0;
    bool initialized_ = false;
    bool finished_ = false;
    bool failed_ = false;
};

class DataArchive {
public:
    bool open(const std::filesystem::path& table_path,
              const std::filesystem::path& archive_path,
              std::string& error);
    bool load_names(const std::filesystem::path& names_path, std::string& error);
    std::size_t member_count() const { return members_.size(); }
    std::uint64_t generation() const { return generation_; }
    bool begin_read_member(std::size_t member, ArchiveMemberLoad& load,
                           std::string& error) const;
    bool read_member(std::size_t member, std::vector<std::uint8_t>& output,
                     std::string& error) const;
    bool read_file(std::size_t member, std::size_t file,
                   std::vector<std::uint8_t>& output, std::string& error) const;
    bool split_member(std::size_t member, std::vector<ArchiveFile>& files,
                      std::string& error) const;
    bool read_named_file(std::size_t member, std::string_view name,
                         std::vector<std::uint8_t>& output, std::string& error) const;
    std::string file_name(std::size_t member, std::size_t file) const;

private:
    struct MemberEntry {
        std::uint32_t offset;
        std::uint32_t packed_size;
        std::uint32_t unpacked_size;
    };

    std::filesystem::path archive_path_;
    std::vector<MemberEntry> members_;
    std::uint64_t generation_ = 0;
    struct NamedFile { std::size_t member; std::size_t file; };
    std::unordered_multimap<std::string, NamedFile> names_;
    std::unordered_map<std::uint64_t, std::string> names_by_slot_;
};

// Native request plan for the generated archive loader's bounded member
// queue. Duplicate requests are retained and processed in insertion order.
class ArchiveLoadPlan {
public:
    static constexpr std::size_t max_members = 7;

    explicit ArchiveLoadPlan(const DataArchive& archive)
        : archive_member_count_(archive.member_count()),
          archive_generation_(archive.generation()) {}

    bool queue_member(std::size_t member, std::string& error) {
        if (member >= archive_member_count_) {
            error = "member index is outside DATA.TBL";
            return false;
        }
        if (size_ == members_.size()) {
            error = "archive load plan is limited to seven members";
            return false;
        }
        members_[size_++] = member;
        return true;
    }

    void clear() { size_ = 0; }
    std::size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    bool matches(const DataArchive& archive) const {
        return archive_generation_ != 0 &&
               archive_generation_ == archive.generation() &&
               archive_member_count_ == archive.member_count();
    }
    std::size_t operator[](std::size_t index) const { return members_.at(index); }
    std::span<const std::size_t> members() const { return {members_.data(), size_}; }

private:
    std::size_t archive_member_count_;
    std::uint64_t archive_generation_;
    std::array<std::size_t, max_members> members_{};
    std::size_t size_ = 0;
};

enum class ArchiveBatchStatus { ready, active, complete, failed };

// Executes a member request plan incrementally. The referenced archive must
// outlive the batch and must not be reopened during its lifetime. Each step
// handles one member and applies the ULZ control-word budget to that member.
class ArchiveBatchLoad {
public:
    ArchiveBatchLoad(const DataArchive& archive, ArchiveLoadPlan plan)
        : archive_(archive), plan_(std::move(plan)) {}
    ArchiveBatchLoad(const ArchiveBatchLoad&) = delete;
    ArchiveBatchLoad& operator=(const ArchiveBatchLoad&) = delete;

    bool begin(std::string& error);
    bool step(std::size_t max_control_words, std::string& error);
    bool take_outputs(std::vector<std::vector<std::uint8_t>>& outputs,
                      std::string& error);

    ArchiveBatchStatus status() const { return status_; }
    std::size_t completed_count() const { return next_member_; }
    std::size_t member_count() const { return plan_.size(); }

private:
    const DataArchive& archive_;
    ArchiveLoadPlan plan_;
    ArchiveMemberLoad current_load_;
    std::vector<std::vector<std::uint8_t>> outputs_;
    std::size_t next_member_ = 0;
    bool current_started_ = false;
    ArchiveBatchStatus status_ = ArchiveBatchStatus::ready;
};

} // namespace ac5
