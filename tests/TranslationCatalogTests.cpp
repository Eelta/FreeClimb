#include "TranslationCatalog.h"
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#ifdef _WIN32
#include <Windows.h>
#include <winioctl.h>
#include <cstring>
#endif

namespace {
int checks = 0, failures = 0;
constexpr fc::TranslationEntry defaults[]{
    {"$FC_LANGUAGE_NAME", "English"},
    {"$FC_TITLE", "FreeClimb"},
    {"$FC_ACTION", "Apply"},
    {"$FC_HELP", "Help"},
    {"$FC_PERCENT", "100%"}
};

void expect(bool condition, const char* message) {
    ++checks;
    if (!condition) { ++failures; std::cerr << "FAIL: " << message << '\n'; }
}

void write(const std::filesystem::path& path, std::string_view content) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out.write(content.data(), static_cast<std::streamsize>(content.size()));
}

std::string utf16(std::u16string_view content) {
    std::string bytes = "\xff\xfe";
    for (const auto c : content) {
        bytes.push_back(static_cast<char>(c & 255));
        bytes.push_back(static_cast<char>(c >> 8));
    }
    return bytes;
}

bool containsWarning(const fc::TranslationCatalog& catalog, std::string_view value) {
    for (const auto& warning : catalog.warnings()) if (warning.find(value) != std::string::npos) return true;
    return false;
}

std::filesystem::path makeDirectory(const std::filesystem::path& root, std::string_view name) {
    const auto path = root / name;
    std::filesystem::create_directories(path);
    return path;
}

void normalizeTests() {
    const std::array<std::pair<std::string, std::string>, 19> examples{{
        {"0", "english"}, {"1", "chinese"}, {" English ", "english"},
        {"CHINESE", "chinese"}, {"pt-BR", "pt-br"}, {"portuguese_br", "portuguese_br"},
        {"french", "french"}, {"", "english"}, {"2", "english"},
        {"../chinese", "english"}, {"french/../english", "english"}, {"_fr", "english"},
        {"-de", "english"}, {"中文", "english"}, {"en gb", "english"},
        {std::string(32, 'a'), std::string(32, 'a')}, {std::string(33, 'a'), "english"},
        {std::string("fr\0ench", 7), "english"}, {"\r\n1\t", "chinese"}
    }};
    for (const auto& [input, output] : examples) expect(fc::normalizeLanguage(input) == output, "Language normalization follows bounded ASCII identifier rules");
    fc::TranslationCatalog catalog(defaults);
    expect(catalog.languages().size() == 1 && catalog.languages()[0].id == "english", "Built-in English is always present");
    expect(std::string_view(catalog.text("unavailable", "$FC_ACTION")) == "Apply", "Uninstalled language falls back to built-in English");
    expect(std::string_view(catalog.text("english", "$UNTRUSTED%n##")) == "[Missing translation]", "Unknown key returns a fixed safe placeholder");
    std::string key = "$FC_ACTION", value = "Owned copy";
    std::array<fc::TranslationEntry, 1> local{{{key, value}}};
    fc::TranslationCatalog owned(local);
    key.assign("changed"); value.assign("changed");
    expect(std::string_view(owned.text("english", "$FC_ACTION")) == "Owned copy", "Constructor owns its defaults independently of caller strings");
}

void unicodeAndFallbackTests(const std::filesystem::path& scratch) {
    const auto directory = makeDirectory(scratch, "Unicode-翻译");
    write(directory / "FreeClimb_english.txt", "\xef\xbb\xbf$FC_LANGUAGE_NAME\tEnglish custom\r\n$FC_TITLE\tExternal English\r\n$FC_ACTION\tExternal Apply\r\n");
    write(directory / "FreeClimb_chinese.txt", utf16(u"$FC_LANGUAGE_NAME\t简体中文\r\n$FC_TITLE\t攀岩 🧗\r\n$FC_HELP\t第一行\\n第二行\r\n"));
    write(directory / "FreeClimb_francais.txt", "$FC_LANGUAGE_NAME\tFrançais\n$FC_TITLE\tEscalade\n$FC_UNKNOWN\tIgnored");
    write(directory / "FreeClimb_deutsch.txt", "$FC_ACTION\tÜbernehmen\n");
    fc::TranslationCatalog catalog(defaults);
    expect(catalog.reload(directory), "UTF-8 BOM and UTF-16LE BOM files load together");
    expect(std::string_view(catalog.text("chinese", "$FC_TITLE")) == "攀岩 🧗", "UTF-16 surrogate pair decodes to exact UTF-8 emoji");
    expect(std::string_view(catalog.text("1", "$FC_HELP")) == "第一行\n第二行", "Legacy language selection and explicit newline escape work");
    expect(std::string_view(catalog.text("chinese", "$FC_ACTION")) == "External Apply", "Missing selected key falls back to external English");
    expect(std::string_view(catalog.text("chinese", "$FC_PERCENT")) == "100%", "Missing external English key falls back to built-in English");
    expect(std::string_view(catalog.text("francais", "$FC_UNKNOWN")) == "[Missing translation]", "Unknown external keys cannot create unregistered UI values");
    expect(std::string_view(catalog.text("invalid/path", "$FC_TITLE")) == "External English", "Invalid language cannot read arbitrary paths");
    expect(containsWarning(catalog, "missing translation keys") && containsWarning(catalog, "unknown translation keys ignored"), "Partial and unknown keys report aggregate warnings");
    const auto& languages = catalog.languages();
    expect(languages.size() == 4 && languages[0].id == "english" && languages[1].id == "chinese" && languages[2].id == "deutsch" && languages[3].id == "francais", "Language list sorts English, Chinese, then ASCII IDs");
    expect(languages[0].name == "English custom" && languages[1].name == "简体中文" && languages[2].name == "deutsch", "Names use translation metadata and ID fallback");
    const char* stable = catalog.text("chinese", "$FC_TITLE");
    for (int i = 0; i < 100; ++i) { catalog.text("deutsch", "$FC_ACTION"); catalog.languages(); catalog.warnings(); }
    expect(std::string_view(stable) == "攀岩 🧗", "Returned text remains valid until reload");
    write(directory / "FreeClimb_PT-BR.txt", "$FC_LANGUAGE_NAME\tPortuguês\n$FC_ACTION\tAplicar");
    expect(catalog.reload(directory) && catalog.languages().back().id == "pt-br", "A newly discovered language appears after reload with normalized ID");
    expect(std::string_view(catalog.text("pt-BR", "$FC_ACTION")) == "Aplicar", "Case-normalized language selection reaches file content");
}

void invalidFileTests(const std::filesystem::path& scratch) {
    const auto directory = makeDirectory(scratch, "invalid");
    const auto file = directory / "FreeClimb_chinese.txt";
    write(file, "$FC_LANGUAGE_NAME\t中文\n$FC_TITLE\t上次有效");
    write(directory / "FreeClimb_english.txt", "$FC_TITLE\tEnglish v1\n");
    fc::TranslationCatalog catalog(defaults);
    expect(catalog.reload(directory), "Valid previous catalog is established");
    std::vector<std::string> bad{
        "$FC_TITLE\tDuplicate one\n$FC_TITLE\tDuplicate two\n",
        "$FC_TITLE\tLabel##hidden",
        "$FC_TITLE\tbad\\q",
        "$FC_TITLE\tbad\\",
        "$FC_TITLE no tab",
        "$FC_title\tlowercase key",
        "$FC_TITLE\tbad\tvalue",
        "$FC_TITLE\tbad\rvalue",
        "$FC_LANGUAGE_NAME\t",
        "$FC_LANGUAGE_NAME\tName\\nsecond line",
        "$FC_LANGUAGE_NAME\t" + std::string(257, 'x'),
        std::string("$FC_TITLE\tNUL\0value", 19),
        std::string("$FC_TITLE\t\xc0\xaf"),
        std::string("$FC_TITLE\t\xed\xa0\x80"),
        std::string("$FC_TITLE\t\xf4\x90\x80\x80"),
        std::string("$FC_TITLE\t\xe4\xb8"),
        std::string("\xfe\xff\0$", 4),
        std::string("\xff\xfe\x00\xd8", 4),
        std::string("\xff\xfe\x00\xdc", 4),
        std::string("\xff\xfe\x41", 3),
        utf16(u"$FC_TITLE\t") + std::string("\0\0", 2),
        "$FC_OTHER\tOne\n$FC_OTHER\tTwo"
    };
    for (const auto& content : bad) {
        write(file, content);
        expect(!catalog.reload(directory), "Malformed file returns failure");
        expect(std::string_view(catalog.text("chinese", "$FC_TITLE")) == "上次有效", "Malformed file preserves that language's previous valid values");
        expect(catalog.languages().size() == 2 && !catalog.warnings().empty(), "Rejected file retains available language and reports warning");
    }
    write(directory / "FreeClimb_english.txt", "$FC_TITLE\tEnglish v2");
    expect(!catalog.reload(directory) && std::string_view(catalog.text("english", "$FC_TITLE")) == "English v2", "One bad language does not prevent an independent valid language update");
    write(file, "$FC_TITLE\t恢复");
    expect(catalog.reload(directory) && std::string_view(catalog.text("chinese", "$FC_TITLE")) == "恢复", "Corrected file can replace retained old content");
    write(file, "$FC_TITLE\tUnsafe##id");
    fc::TranslationCatalog fresh(defaults);
    expect(!fresh.reload(directory) && fresh.languages().size() == 1 && std::string_view(fresh.text("chinese", "$FC_TITLE")) == "English v2", "First-load invalid language is not exposed and falls back safely");
}

void deletionAndDirectoryTests(const std::filesystem::path& scratch) {
    const auto directory = makeDirectory(scratch, "deletion");
    const auto file = directory / "FreeClimb_chinese.txt";
    write(file, "$FC_TITLE\t删除测试");
    fc::TranslationCatalog catalog(defaults);
    expect(catalog.reload(directory), "Deletion fixture loads");
    std::filesystem::remove(file);
    expect(catalog.reload(directory) && catalog.languages().size() == 1, "Removed file is removed from successful discovery");
    expect(std::string_view(catalog.text("chinese", "$FC_TITLE")) == "FreeClimb", "Deleted language falls back without retaining stale content");
    write(file, "$FC_TITLE\tRetained");
    expect(catalog.reload(directory), "Reload before invalid directory succeeds");
    const auto notDirectory = scratch / "regular-file.txt";
    write(notDirectory, "not a directory");
    expect(!catalog.reload(notDirectory) && std::string_view(catalog.text("chinese", "$FC_TITLE")) == "Retained", "Directory I/O/type failure retains previously loaded catalog");
    expect(!catalog.reload(scratch / "does-not-exist") && catalog.languages().size() == 1, "Missing directory clears external state to built-in English");
    expect(std::string_view(catalog.text("chinese", "$FC_TITLE")) == "FreeClimb", "Missing directory has reliable English fallback");
}

void plainTextTests(const std::filesystem::path& scratch) {
    const auto directory = makeDirectory(scratch, "plain-text");
    write(directory / "FreeClimb_english.txt", "; supported comment\n# second comment\n\n$FC_PERCENT\t100% %s %n %%\n$FC_HELP\tFirst\\nSecond\\\\nLiteral\n$FC_ACTION\t  Preserve spaces  ");
    fc::TranslationCatalog catalog(defaults);
    expect(catalog.reload(directory), "Plain text file accepts comments and supported escapes");
    expect(std::string_view(catalog.text("english", "$FC_PERCENT")) == "100% %s %n %%", "Percent sequences remain inert literal text");
    expect(std::string_view(catalog.text("english", "$FC_HELP")) == "First\nSecond\\nLiteral", "Only backslash-n and escaped backslash are interpreted once");
    expect(std::string_view(catalog.text("english", "$FC_ACTION")) == "  Preserve spaces  ", "Value whitespace is preserved");
    write(directory / "FreeClimb_english.txt", "$FC_ACTION\t");
    expect(catalog.reload(directory) && std::string_view(catalog.text("english", "$FC_ACTION")).empty(), "Explicit empty known value is distinct from a missing value");
}

void boundsAndPathsTests(const std::filesystem::path& scratch) {
    const auto directory = makeDirectory(scratch, "bounds");
    const auto file = directory / "FreeClimb_english.txt";
    constexpr std::string_view prefix = "$FC_TITLE\t";
    fc::TranslationCatalog catalog(defaults);
    const std::string limitValue(16384 - prefix.size(), 'x');
    write(file, std::string(prefix) + limitValue);
    expect(catalog.reload(directory), "Maximum 16384-byte line is accepted");
    write(file, std::string(prefix) + limitValue + "x");
    expect(!catalog.reload(directory) && std::string_view(catalog.text("english", "$FC_TITLE")) == limitValue, "Overlong line is rejected transactionally");
    write(file, std::string(1024 * 1024 + 1, 'x'));
    expect(!catalog.reload(directory) && containsWarning(catalog, "1 MiB"), "Oversized file is rejected before parsing");
    std::string exact;
    while (exact.size() + 2 <= 1024 * 1024) exact += "#\n";
    write(file, exact);
    expect(catalog.reload(directory), "An exact one MiB file with bounded lines is accepted");
    write(file, "$FC_TITLE\tSafe");
    write(directory / "FreeClimb_..txt", "$FC_TITLE\tTraversal");
    write(directory / "FreeClimb_2.txt", "$FC_TITLE\tNumeric alias");
    write(directory / "FreeClimb_ english.txt", "$FC_TITLE\tWhitespace alias");
    const auto nested = makeDirectory(directory, "nested");
    write(nested / "FreeClimb_hidden.txt", "$FC_TITLE\tNested");
    std::filesystem::create_directory(directory / "FreeClimb_directory.txt");
    expect(!catalog.reload(directory), "Invalid candidate filenames and non-file candidates report failure");
    expect(catalog.languages().size() == 1 && std::string_view(catalog.text("english", "$FC_TITLE")) == "Safe", "Invalid aliases, nested files and matching directories cannot inject a language");
    const auto outside = scratch / "outside.txt";
    write(outside, "$FC_TITLE\tOutside");
    std::error_code ec;
    std::filesystem::create_symlink(outside, directory / "FreeClimb_linked.txt", ec);
    if (!ec) {
        expect(!catalog.reload(directory) && catalog.languages().size() == 1, "Symbolic link outside directory cannot load translations");
    } else std::cout << "Symlink creation unavailable; literal path and nested-path checks performed\n";
    const auto linkedDirectory = scratch / "linked-directory";
    ec.clear();
    std::filesystem::create_directory_symlink(directory, linkedDirectory, ec);
    if (!ec) {
        expect(!catalog.reload(linkedDirectory) && catalog.languages().size() == 1,
            "Actual linked directory cannot bypass the reparse and symlink policy");
    } else std::cout << "Directory symlink creation unavailable\n";
    const auto many = makeDirectory(scratch, "many-languages");
    for (int i = 0; i < 64; ++i) write(many / ("FreeClimb_lang" + std::to_string(i) + ".txt"), "$FC_TITLE\tMany");
    expect(catalog.reload(many) && catalog.languages().size() == 65, "Sixty-four candidate files plus built-in English are supported");
    write(many / "FreeClimb_excess.txt", "$FC_TITLE\tExcess");
    expect(!catalog.reload(many) && catalog.languages().size() == 65 && containsWarning(catalog, "64 candidate"), "Discovery limit rejects the whole scan without order-dependent partial replacement");
}

struct OverlayFiles : fc::TranslationFileAccess {
    std::filesystem::path logicalDirectory, directoryBackend;
    std::map<std::filesystem::path, std::filesystem::path> backends;
    std::vector<std::filesystem::path> extraNames;
    mutable std::size_t reads{};
    mutable bool unexpectedPath{};
    fc::TranslationDirectoryStatus list(const std::filesystem::path& directory,
        std::vector<std::filesystem::path>& filenames, std::string& error) const override {
        if (directory != logicalDirectory) { unexpectedPath = true; error = "not a virtual directory"; return fc::TranslationDirectoryStatus::unreadable; }
        std::vector<std::filesystem::path> ignored;
        const auto status = TranslationFileAccess::list(directoryBackend, ignored, error);
        if (status != fc::TranslationDirectoryStatus::ready) return status;
        filenames.clear();
        for (const auto& [name, backend] : backends) filenames.push_back(name);
        filenames.insert(filenames.end(), extraNames.begin(), extraNames.end());
        return fc::TranslationDirectoryStatus::ready;
    }
    bool read(const std::filesystem::path& file, std::size_t limit,
        std::string& bytes, std::string& error) const override {
        ++reads;
        if (file.parent_path() != logicalDirectory) { unexpectedPath = true; error = "outside virtual directory"; return false; }
        const auto backend = backends.find(file.filename());
        if (backend == backends.end()) { unexpectedPath = true; error = "unmapped virtual file"; return false; }
        return TranslationFileAccess::read(backend->second, limit, bytes, error);
    }
};

void virtualOverlayTests(const std::filesystem::path& scratch) {
    OverlayFiles files;
    files.logicalDirectory = scratch / "virtual-game" / "Data" / "Interface" / "Translations";
    files.directoryBackend = makeDirectory(scratch, "overlay-directory-owner");
    const auto english = makeDirectory(scratch, "mod-base") / "FreeClimb_english.txt";
    const auto chinese = makeDirectory(scratch, "mod-chinese") / "FreeClimb_chinese.txt";
    write(english, "$FC_LANGUAGE_NAME\tEnglish\n$FC_TITLE\tOverlay English\n$FC_ACTION\tApply overlay");
    write(chinese, utf16(u"$FC_LANGUAGE_NAME\t简体中文\n$FC_TITLE\t虚拟中文\n$FC_ACTION\t应用"));
    files.backends.emplace(english.filename(), english);
    files.backends.emplace(chinese.filename(), chinese);
    expect(!std::filesystem::exists(files.logicalDirectory), "Virtual files need not exist in the host's unredirected game directory");
    for (const auto& [name, backend] : files.backends)
        expect(std::filesystem::weakly_canonical(backend).parent_path() != std::filesystem::weakly_canonical(files.directoryBackend),
            "Directory and each language intentionally resolve to different physical mod backends");
    fc::TranslationCatalog catalog(defaults);
    expect(catalog.reload(files.logicalDirectory, files) && catalog.languages().size() == 2,
        "Merged virtual directory exposes English and Chinese despite unrelated physical parents");
    expect(std::string_view(catalog.text("1", "$FC_ACTION")) == "应用", "Saved legacy Chinese selection loads through the virtual namespace");
    for (const auto language : {"chinese", "english", "chinese"}) {
        expect(catalog.reload(files.logicalDirectory, files) && catalog.languages().size() == 2,
            "Switching languages and reloading cannot discard an installed virtual Chinese file");
        expect(std::string_view(catalog.text(language, "$FC_ACTION")) == (language == std::string_view("chinese") ? "应用" : "Apply overlay"),
            "Both language selections retain their actual external values");
    }
    write(chinese, utf16(u"$FC_LANGUAGE_NAME\t简体中文\n$FC_ACTION\t已更新"));
    expect(catalog.reload(files.logicalDirectory, files) && std::string_view(catalog.text("chinese", "$FC_ACTION")) == "已更新",
        "Reload reads a changed virtual file from its current backend");
    write(chinese, "$FC_ACTION\tInvalid##identity");
    expect(!catalog.reload(files.logicalDirectory, files) && std::string_view(catalog.text("chinese", "$FC_ACTION")) == "已更新",
        "Virtual mapping does not weaken malformed-content rejection or transactional retention");
    write(chinese, "$FC_ACTION\tRestored");
    files.extraNames = {"../FreeClimb_escape.txt", "nested/FreeClimb_nested.txt", scratch / "FreeClimb_absolute.txt"};
    const auto readsBefore = files.reads;
    expect(!catalog.reload(files.logicalDirectory, files) && files.reads == readsBefore + 2 && !files.unexpectedPath,
        "Traversal, nested and absolute entries are rejected before any backend read");
    expect(catalog.languages().size() == 2 && containsWarning(catalog, "direct child"), "Unsafe enumeration entries cannot inject languages");
    files.extraNames.clear();
    files.backends.erase(chinese.filename());
    expect(catalog.reload(files.logicalDirectory, files) && catalog.languages().size() == 1,
        "Virtual removal still removes a language after a successful rescan");
    expect(!files.unexpectedPath, "All catalog access remains in the requested virtual namespace");
}

#ifdef _WIN32
bool createJunction(const std::filesystem::path& link, const std::filesystem::path& target) {
    const auto print = std::filesystem::absolute(target).native(), substitute = L"\\??\\" + print;
    const auto substituteBytes = static_cast<WORD>(substitute.size() * sizeof(wchar_t));
    const auto printBytes = static_cast<WORD>(print.size() * sizeof(wchar_t));
    const WORD payload = static_cast<WORD>(8 + substituteBytes + printBytes + 4), printOffset = substituteBytes + 2;
    std::vector<unsigned char> buffer(8 + payload, 0);
    const DWORD tag = IO_REPARSE_TAG_MOUNT_POINT;
    std::memcpy(buffer.data(), &tag, sizeof(tag));
    std::memcpy(buffer.data() + 4, &payload, sizeof(payload));
    std::memcpy(buffer.data() + 10, &substituteBytes, sizeof(substituteBytes));
    std::memcpy(buffer.data() + 12, &printOffset, sizeof(printOffset));
    std::memcpy(buffer.data() + 14, &printBytes, sizeof(printBytes));
    std::memcpy(buffer.data() + 16, substitute.c_str(), substituteBytes);
    std::memcpy(buffer.data() + 16 + printOffset, print.c_str(), printBytes);
    std::filesystem::create_directory(link);
    const auto handle = CreateFileW(link.c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
        FILE_FLAG_OPEN_REPARSE_POINT | FILE_FLAG_BACKUP_SEMANTICS, nullptr);
    if (handle == INVALID_HANDLE_VALUE) return false;
    DWORD returned{};
    const bool created = DeviceIoControl(handle, FSCTL_SET_REPARSE_POINT, buffer.data(),
        static_cast<DWORD>(buffer.size()), nullptr, 0, &returned, nullptr) != 0;
    CloseHandle(handle);
    return created;
}

void realReparseTests(const std::filesystem::path& scratch) {
    const auto directory = makeDirectory(scratch, "junction-target"), junction = scratch / "junction-alias";
    write(directory / "FreeClimb_chinese.txt", "$FC_LANGUAGE_NAME\tChinese\n$FC_TITLE\tSafe target");
    fc::TranslationCatalog catalog(defaults);
    expect(catalog.reload(directory) && catalog.languages().size() == 2, "Unlinked junction target is a valid language directory");
    if (!createJunction(junction, directory)) { std::cout << "Junction creation unavailable\n"; return; }
    expect((GetFileAttributesW(junction.c_str()) & FILE_ATTRIBUTE_REPARSE_POINT) != 0,
        "Fixture is an actual Windows reparse point without symbolic-link privileges");
    expect(!catalog.reload(junction) && catalog.languages().size() == 2 && std::string_view(catalog.text("chinese", "$FC_TITLE")) == "Safe target",
        "Actual reparse directory is refused while previous translations remain usable");
    const auto fileJunction = directory / "FreeClimb_linked.txt";
    if (createJunction(fileJunction, scratch)) {
        expect(!catalog.reload(directory) && catalog.languages().size() == 2,
            "Reparse entry named like a translation cannot load another backend as a language");
        std::filesystem::remove(fileJunction);
    }
    std::filesystem::remove(junction);
    expect(catalog.reload(directory) && catalog.languages().size() == 2, "Removing only junction entries preserves the real target contents");
}
#endif
}

int main() {
    try {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto scratch = std::filesystem::current_path() / ("translation-tests-" + std::to_string(stamp));
        std::filesystem::create_directories(scratch);
        normalizeTests();
        unicodeAndFallbackTests(scratch);
        invalidFileTests(scratch);
        deletionAndDirectoryTests(scratch);
        plainTextTests(scratch);
        boundsAndPathsTests(scratch);
        virtualOverlayTests(scratch);
#ifdef _WIN32
        realReparseTests(scratch);
#endif
        std::cout << checks << " translation checks, " << failures << " failures\n";
        return failures ? 1 : 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL exception: " << error.what() << '\n';
        return 1;
    }
}
