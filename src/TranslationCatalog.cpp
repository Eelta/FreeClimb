#include "TranslationCatalog.h"
#include <algorithm>
#include <cstdint>
#include <exception>
#include <fstream>
#include <optional>
#include <set>
#include <utility>
#ifdef _WIN32
#include <Windows.h>
#endif

namespace fc {
namespace {
constexpr std::size_t maxFileBytes = 1024 * 1024;
constexpr std::size_t maxLineBytes = 16384;
constexpr std::size_t maxLanguages = 64;
constexpr std::size_t maxWarnings = 192;
constexpr std::string_view languageName = "$FC_LANGUAGE_NAME";
constexpr char missingText[] = "[Missing translation]";
using TextMap = std::map<std::string, std::string, std::less<>>;

bool asciiSpace(char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v';
}

char lowerAscii(char c) {
    return c >= 'A' && c <= 'Z' ? static_cast<char>(c + ('a' - 'A')) : c;
}

bool candidateFilename(std::string_view filename) {
    if (filename.size() < 14) return false;
    std::string folded(filename);
    std::transform(folded.begin(), folded.end(), folded.begin(), lowerAscii);
    return folded.starts_with("freeclimb_") && folded.ends_with(".txt");
}

std::optional<std::string> languageId(std::string_view value, bool legacy) {
    while (!value.empty() && asciiSpace(value.front())) value.remove_prefix(1);
    while (!value.empty() && asciiSpace(value.back())) value.remove_suffix(1);
    if (legacy && value == "0") return "english";
    if (legacy && value == "1") return "chinese";
    if (value.empty() || value.size() > 32) return std::nullopt;
    std::string result;
    result.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        const char c = lowerAscii(value[i]);
        if (!((c >= 'a' && c <= 'z') || (i && ((c >= '0' && c <= '9') || c == '_' || c == '-')))) return std::nullopt;
        result.push_back(c);
    }
    return result;
}

bool validKey(std::string_view key) {
    if (!key.starts_with("$FC_") || key.size() <= 4 || key.size() > 128) return false;
    for (const char c : key.substr(4)) if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) return false;
    return true;
}

bool validUtf8(std::string_view text) {
    for (std::size_t i = 0; i < text.size();) {
        const auto first = static_cast<std::uint8_t>(text[i++]);
        if (!first) return false;
        if (first < 0x80) continue;
        unsigned count = 0;
        std::uint32_t value = 0, minimum = 0;
        if (first >= 0xc2 && first <= 0xdf) { count = 1; value = first & 0x1f; minimum = 0x80; }
        else if (first >= 0xe0 && first <= 0xef) { count = 2; value = first & 0x0f; minimum = 0x800; }
        else if (first >= 0xf0 && first <= 0xf4) { count = 3; value = first & 0x07; minimum = 0x10000; }
        else return false;
        if (count > text.size() - i) return false;
        for (unsigned j = 0; j < count; ++j) {
            const auto c = static_cast<std::uint8_t>(text[i++]);
            if ((c & 0xc0) != 0x80) return false;
            value = (value << 6) | (c & 0x3f);
        }
        if (value < minimum || value > 0x10ffff || (value >= 0xd800 && value <= 0xdfff)) return false;
    }
    return true;
}

void appendUtf8(std::string& output, std::uint32_t value) {
    if (value < 0x80) output.push_back(static_cast<char>(value));
    else if (value < 0x800) {
        output.push_back(static_cast<char>(0xc0 | (value >> 6)));
        output.push_back(static_cast<char>(0x80 | (value & 0x3f)));
    } else if (value < 0x10000) {
        output.push_back(static_cast<char>(0xe0 | (value >> 12)));
        output.push_back(static_cast<char>(0x80 | ((value >> 6) & 0x3f)));
        output.push_back(static_cast<char>(0x80 | (value & 0x3f)));
    } else {
        output.push_back(static_cast<char>(0xf0 | (value >> 18)));
        output.push_back(static_cast<char>(0x80 | ((value >> 12) & 0x3f)));
        output.push_back(static_cast<char>(0x80 | ((value >> 6) & 0x3f)));
        output.push_back(static_cast<char>(0x80 | (value & 0x3f)));
    }
}

bool decodeText(std::string_view bytes, std::string& output) {
    if (bytes.starts_with("\xff\xfe")) {
        bytes.remove_prefix(2);
        if (bytes.size() % 2) return false;
        output.clear();
        output.reserve(bytes.size());
        auto word = [&](std::size_t offset) {
            return static_cast<std::uint32_t>(static_cast<std::uint8_t>(bytes[offset])) |
                (static_cast<std::uint32_t>(static_cast<std::uint8_t>(bytes[offset + 1])) << 8);
        };
        for (std::size_t i = 0; i < bytes.size(); i += 2) {
            std::uint32_t value = word(i);
            if (!value || (value >= 0xdc00 && value <= 0xdfff)) return false;
            if (value >= 0xd800 && value <= 0xdbff) {
                if (i + 3 >= bytes.size()) return false;
                const auto low = word(i + 2);
                if (low < 0xdc00 || low > 0xdfff) return false;
                value = 0x10000 + ((value - 0xd800) << 10) + (low - 0xdc00);
                i += 2;
            }
            appendUtf8(output, value);
        }
        return true;
    }
    if (bytes.starts_with("\xef\xbb\xbf")) bytes.remove_prefix(3);
    if (!validUtf8(bytes)) return false;
    output.assign(bytes);
    return true;
}

bool safeValue(std::string_view value) {
    if (value.size() > maxLineBytes || !validUtf8(value) || value.find("##") != std::string_view::npos) return false;
    for (const char c : value) {
        const auto b = static_cast<std::uint8_t>(c);
        if ((b < 0x20 && c != '\n') || b == 0x7f) return false;
    }
    return true;
}

bool unescape(std::string_view value, std::string& output) {
    output.clear();
    output.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (value[i] != '\\') output.push_back(value[i]);
        else {
            if (++i == value.size()) return false;
            if (value[i] == 'n') output.push_back('\n');
            else if (value[i] == '\\') output.push_back('\\');
            else return false;
        }
    }
    return safeValue(output);
}

void warning(std::vector<std::string>& output, std::string value) {
    if (output.size() < maxWarnings) output.push_back(std::move(value));
}

bool ordinaryPath(const std::filesystem::path& path, std::string& error) {
#ifdef _WIN32
    const auto attributes = GetFileAttributesW(path.c_str());
    if (attributes == INVALID_FILE_ATTRIBUTES) { error = "cannot read file attributes"; return false; }
    if (attributes & FILE_ATTRIBUTE_REPARSE_POINT) { error = "symbolic links and reparse points are unsupported"; return false; }
#else
    std::error_code ec;
    if (std::filesystem::is_symlink(std::filesystem::symlink_status(path, ec)) || ec) {
        error = "symbolic links or unreadable paths are unsupported"; return false;
    }
#endif
    return true;
}

bool loadFile(const std::filesystem::path& path, const TranslationFileAccess& files, const TextMap& defaults,
    TextMap& result, std::vector<std::string>& notices, std::string& error) {
    std::string bytes;
    if (!files.read(path, maxFileBytes, bytes, error)) return false;
    if (bytes.size() > maxFileBytes) { error = "file exceeds 1 MiB"; return false; }
    std::string decoded;
    if (!decodeText(bytes, decoded)) { error = "invalid Unicode encoding or NUL character"; return false; }
    std::set<std::string, std::less<>> seen;
    std::size_t start = 0, lineNumber = 0, unknown = 0;
    while (start < decoded.size()) {
        const auto end = decoded.find('\n', start);
        std::string_view line(decoded.data() + start, (end == std::string::npos ? decoded.size() : end) - start);
        start = end == std::string::npos ? decoded.size() : end + 1;
        ++lineNumber;
        if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
        auto fail = [&](std::string reason) { error = "line " + std::to_string(lineNumber) + ": " + std::move(reason); return false; };
        if (line.size() > maxLineBytes) return fail("line exceeds 16384 UTF-8 bytes");
        if (line.empty()) continue;
        if (line.find('\r') != std::string_view::npos) return fail("bare carriage return");
        if (line.front() == ';' || line.front() == '#') continue;
        const auto tab = line.find('\t');
        if (tab == std::string_view::npos || !validKey(line.substr(0, tab))) return fail("expected $FC_KEY followed by one tab");
        const auto key = line.substr(0, tab);
        if (!seen.emplace(key).second) return fail("duplicate key " + std::string(key));
        std::string value;
        if (!unescape(line.substr(tab + 1), value)) return fail("invalid value, escape, control character, or ## marker");
        if (key == languageName && (value.empty() || value.find('\n') != std::string::npos || value.size() > 256)) return fail("invalid language display name");
        if (defaults.contains(key)) result.emplace(key, std::move(value));
        else ++unknown;
    }
    if (unknown) notices.push_back(std::to_string(unknown) + " unknown translation keys ignored");
    const auto missing = defaults.size() - result.size();
    if (missing) notices.push_back(std::to_string(missing) + " missing translation keys use English fallback");
    return true;
}
}

TranslationDirectoryStatus TranslationFileAccess::list(const std::filesystem::path& directory,
    std::vector<std::filesystem::path>& filenames, std::string& error) const {
    filenames.clear();
    std::error_code ec;
    const auto status = std::filesystem::symlink_status(directory, ec);
    if ((!ec && !std::filesystem::exists(status)) || ec == std::errc::no_such_file_or_directory)
        return TranslationDirectoryStatus::missing;
    if (ec || !std::filesystem::is_directory(status)) { error = "not a readable directory"; return TranslationDirectoryStatus::unreadable; }
    if (!ordinaryPath(directory, error)) return TranslationDirectoryStatus::unreadable;
    std::filesystem::directory_iterator entry(directory, ec), end;
    while (!ec && entry != end) {
        const auto name = entry->path().filename();
        const auto utf8 = name.u8string();
        if (candidateFilename(std::string_view(reinterpret_cast<const char*>(utf8.data()), utf8.size()))) {
            filenames.push_back(name);
            if (filenames.size() > maxLanguages) break;
        }
        entry.increment(ec);
    }
    if (ec) { error = "directory scan failed"; return TranslationDirectoryStatus::unreadable; }
    return TranslationDirectoryStatus::ready;
}

bool TranslationFileAccess::read(const std::filesystem::path& file, std::size_t limit,
    std::string& bytes, std::string& error) const {
    bytes.clear();
    std::error_code ec;
    const auto status = std::filesystem::symlink_status(file, ec);
    if (ec || !std::filesystem::is_regular_file(status) || std::filesystem::is_symlink(status)) { error = "not a regular file"; return false; }
    if (!ordinaryPath(file, error)) return false;
    const auto size = std::filesystem::file_size(file, ec);
    if (ec || size > limit) { error = "unreadable file or file exceeds 1 MiB"; return false; }
    std::ifstream stream(file, std::ios::binary);
    if (!stream) { error = "cannot open file"; return false; }
    bytes.assign(static_cast<std::size_t>(size), '\0');
    if (size && !stream.read(bytes.data(), static_cast<std::streamsize>(size))) { error = "file changed or read failed"; return false; }
    if (stream.peek() != std::char_traits<char>::eof() || stream.bad()) { error = "file changed or read failed"; return false; }
    return true;
}

std::string normalizeLanguage(std::string_view language) {
    const auto id = languageId(language, true);
    return id ? *id : "english";
}

TranslationCatalog::TranslationCatalog(std::span<const TranslationEntry> englishDefaults) {
    for (const auto& entry : englishDefaults) {
        if (validKey(entry.key) && safeValue(entry.value)) defaults_.try_emplace(std::string(entry.key), std::string(entry.value));
    }
    defaults_.try_emplace(std::string(languageName), "English");
    rebuildLanguages();
}

bool TranslationCatalog::reload(const std::filesystem::path& directory) {
    const TranslationFileAccess files;
    return reload(directory, files);
}

bool TranslationCatalog::reload(const std::filesystem::path& directory, const TranslationFileAccess& files) try {
    warnings_.clear();
    std::vector<std::filesystem::path> filenames;
    std::string scanError;
    const auto status = files.list(directory, filenames, scanError);
    if (status == TranslationDirectoryStatus::missing) {
        external_.clear();
        rebuildLanguages();
        warning(warnings_, "Translation directory is missing; built-in English is active");
        return false;
    }
    if (status != TranslationDirectoryStatus::ready) {
        warning(warnings_, "Cannot read translation directory: " + scanError + "; previous translations retained");
        return false;
    }
    std::map<std::string, std::vector<std::filesystem::path>, std::less<>> candidates;
    std::size_t candidateCount = 0;
    bool valid = true;
    for (const auto& name : filenames) {
        if (name.empty() || name.has_parent_path() || name.has_root_path() || name != name.filename()) {
            warning(warnings_, "Translation entry is not a direct child filename; ignored");
            valid = false;
            continue;
        }
        const auto filenameUtf8 = name.u8string();
        const std::string filename(filenameUtf8.begin(), filenameUtf8.end());
        if (candidateFilename(filename)) {
            if (++candidateCount > maxLanguages) {
                warning(warnings_, "Translation directory exceeds 64 candidate languages; previous translations retained");
                return false;
            }
            const auto rawId = std::string_view(filename).substr(10, filename.size() - 14);
            const auto id = languageId(rawId, false);
            if (!id || rawId.size() != id->size()) {
                warning(warnings_, "Invalid translation filename ignored");
                valid = false;
            } else candidates[*id].push_back(directory / name);
        }
    }
    if (!candidates.contains("english")) warning(warnings_, "English translation file is missing; built-in English supplies fallback text");
    std::map<std::string, Texts, std::less<>> next;
    for (const auto& [id, paths] : candidates) {
        auto keepPrevious = [&] {
            if (const auto old = external_.find(id); old != external_.end()) next.emplace(old->first, old->second);
        };
        if (paths.size() != 1) {
            warning(warnings_, id + ": duplicate language files; previous translation retained");
            keepPrevious();
            valid = false;
            continue;
        }
        Texts texts;
        std::vector<std::string> notices;
        std::string error;
        if (!loadFile(paths.front(), files, defaults_, texts, notices, error)) {
            warning(warnings_, id + ": " + error + "; previous translation retained");
            keepPrevious();
            valid = false;
            continue;
        }
        for (const auto& notice : notices) warning(warnings_, id + ": " + notice);
        next.emplace(id, std::move(texts));
    }
    external_.swap(next);
    rebuildLanguages();
    return valid;
} catch (const std::filesystem::filesystem_error&) {
    warning(warnings_, "Translation path cannot be decoded or read; previous translations retained");
    return false;
}

const char* TranslationCatalog::text(std::string_view language, std::string_view key) const {
    const auto base = defaults_.find(key);
    if (base == defaults_.end()) return missingText;
    const auto id = normalizeLanguage(language);
    if (const auto selected = external_.find(id); selected != external_.end()) {
        if (const auto found = selected->second.find(key); found != selected->second.end()) return found->second.c_str();
    }
    if (const auto english = external_.find("english"); english != external_.end()) {
        if (const auto found = english->second.find(key); found != english->second.end()) return found->second.c_str();
    }
    return base->second.c_str();
}

void TranslationCatalog::rebuildLanguages() {
    languages_.clear();
    auto add = [&](const std::string& id) {
        std::string name = id == "english" ? defaults_.at(std::string(languageName)) : id;
        if (const auto language = external_.find(id); language != external_.end()) {
            if (const auto found = language->second.find(languageName); found != language->second.end()) name = found->second;
        }
        languages_.push_back({ id, std::move(name) });
    };
    add("english");
    if (external_.contains("chinese")) add("chinese");
    for (const auto& [id, texts] : external_) if (id != "english" && id != "chinese") add(id);
}

const std::vector<TranslationLanguage>& TranslationCatalog::languages() const { return languages_; }
const std::vector<std::string>& TranslationCatalog::warnings() const { return warnings_; }
}
