#pragma once
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace fc {
struct TranslationEntry {
    std::string_view key, value;
};
struct TranslationLanguage {
    std::string id, name;
};
std::string normalizeLanguage(std::string_view language);

enum class TranslationDirectoryStatus { ready, missing, unreadable };
class TranslationFileAccess {
public:
    virtual ~TranslationFileAccess() = default;
    virtual TranslationDirectoryStatus list(const std::filesystem::path& directory,
        std::vector<std::filesystem::path>& filenames, std::string& error) const;
    virtual bool read(const std::filesystem::path& file, std::size_t limit,
        std::string& bytes, std::string& error) const;
};

class TranslationCatalog {
public:
    explicit TranslationCatalog(std::span<const TranslationEntry> englishDefaults);
    bool reload(const std::filesystem::path& directory);
    bool reload(const std::filesystem::path& directory, const TranslationFileAccess& files);
    const char* text(std::string_view language, std::string_view key) const;
    const std::vector<TranslationLanguage>& languages() const;
    const std::vector<std::string>& warnings() const;

private:
    using Texts = std::map<std::string, std::string, std::less<>>;
    Texts defaults_;
    std::map<std::string, Texts, std::less<>> external_;
    std::vector<TranslationLanguage> languages_;
    std::vector<std::string> warnings_;
    void rebuildLanguages();
};
}
