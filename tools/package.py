import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import zipfile


ROOT = Path(__file__).resolve().parents[1]
TRANSLATION_FILES = ("FreeClimb_english.txt", "FreeClimb_chinese.txt")
SOURCE_FILES = (
    "CMakeLists.txt", "README.md", "README.zh-CN.md", "LICENSE", "THIRD-PARTY-NOTICES.txt",
    "tools/build.ps1", "tools/dependencies.json", "tools/animation-runtime.json",
    "tools/package.py", "tools/validate.py", "tools/deploy.ps1", "tools/publication.json", "tools/ExportMotionHkx.cpp",
    "tools/AuthoringCli.cpp", "tools/authoring/core.py", "tools/authoring/app.py",
    "tools/MigrateAnimationPack.cpp", "tools/retarget_threepeat.py",
    "tools/authoring/Start.cmd", "tools/authoring/README.md", "tools/authoring/README.zh-CN.md", "tools/authoring/bin/FreeClimbAuthoring.exe",
    "tools/converter/README.md", "tools/converter/README.zh-CN.md",
    "tools/converter/Converter.manifest",
    "tools/converter/images/overview.png", "tools/converter/images/overview.zh-CN.png",
    "tools/converter/images/contacts.png", "tools/converter/images/contacts.zh-CN.png",
    "tools/converter/bin/FreeClimbHKXConverter.exe", "tools/converter/bin/FreeClimbConverter.exe",
    "tests/ReleasePipelineTests.py",
    "docs/ANIMATION-DIY.md", "docs/ANIMATION-DIY.zh-CN.md",
    "translations/FreeClimb_english.txt", "translations/FreeClimb_chinese.txt",
    "docs/MIXAMO-LICENSE.md", "docs/THREEPEAT-LICENSE.md", "docs/AUDIO-LICENSE.md",
    "src/vendor/SKSEMenuFramework/LICENSE",
    "src/vendor/SKSEMenuFramework/SOURCE.txt",
    "external/nlohmann/json.hpp", "external/nlohmann/LICENSE.MIT",
)
SOURCE_TYPES = {
    "src": {".h", ".hpp", ".cpp", ".c"},
    "tests": {".h", ".hpp", ".cpp", ".c", ".py", ".ps1"},
    "cmake": {".cmake"},
    "config": {".ini"},
    "tools/converter": {".h", ".cpp"},
}
SOURCE_EXCLUDED = {"cmake/LocalTests.cmake"}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def version():
    match = re.search(r"project\(FreeClimb\s+VERSION\s+([0-9.]+)",
                      (ROOT / "CMakeLists.txt").read_text(encoding="utf-8"))
    if not match:
        raise ValueError("Missing FreeClimb project version")
    return match.group(1)


def dependencies():
    return json.loads((ROOT / "tools/dependencies.json").read_text(encoding="utf-8"))


def publication():
    policy = json.loads((ROOT / "tools/publication.json").read_text(encoding="utf-8"))
    if policy.get('schema') != 1:
        raise ValueError('Unsupported publication policy')
    for key in ('runtime_archive', 'source_archive'):
        name = policy.get(key, '')
        if not re.fullmatch(r'[A-Za-z][A-Za-z-]*\.zip', name):
            raise ValueError(f'Invalid public archive name: {name}')
    if policy['runtime_archive'] == policy['source_archive']:
        raise ValueError('Runtime and source archive names must differ')
    return policy


def validate_author_baseline():
    for name, expected in publication()['protected_files'].items():
        if not safe_name(name) or not regular_file(ROOT / name, ROOT) or sha256(ROOT / name) != expected:
            raise ValueError(f'Author-protected file changed: {name}')


def runtime_baseline():
    excluded = set(publication()['excluded_runtime_files'])
    return {name: digest for name, digest in dependencies()['runtime_baseline'].items() if name not in excluded}


def safe_name(name):
    path = PurePosixPath(name)
    return bool(name) and not path.is_absolute() and '..' not in path.parts and '\\' not in name and ':' not in name


def regular_file(path, root):
    path, root = Path(path).absolute(), Path(root).absolute()
    if not path.is_file() or not path.resolve().is_relative_to(root.resolve()):
        return False
    for part in [path, *path.parents]:
        if part == root.parent:
            break
        if part.is_symlink() or getattr(part.stat(), 'st_file_attributes', 0) & 0x400:
            return False
    return True


def animation_manifest():
    animations = json.loads((ROOT / "tools/animation-runtime.json").read_text(encoding="utf-8"))
    files = animations.get("files", {})
    directory = "meshes/actors/character/animations/FreeClimb/"
    if animations.get("schema") != 2 or animations.get("motions") != 31 or animations.get("skeletonBones") != 99:
        raise ValueError("Expected schema 2 animation manifest with 31 clips and 99 bones")
    if animations.get("version") != version() or animations.get("pack") != directory + "pack.json":
        raise ValueError("Animation release version or pack entry mismatch")
    if len(files) != 52 or sum(name.endswith('.hkx') for name in files) != 25 or sum(name.startswith(directory + 'configs/') and name.endswith('.json') for name in files) != 25:
        raise ValueError("Default pack must contain 25 HKX files, 25 action configs, pack.json and skeleton.json")
    if any(directory + name not in files for name in tuple(f"{prefix}{name}.{extension}" for name in ("runUp", "runLeft", "runRight", "runDiagonalLeft", "runDiagonalRight", "contextHopLeft", "contextHopRight") for prefix, extension in (("", "hkx"), ("configs/", "json")))):
        raise ValueError("Missing directional wall-run or contextual-hop animation")
    if directory + "pack.json" not in files or directory + "skeleton.json" not in files:
        raise ValueError("Missing editable pack or skeleton manifest")
    if len({name.casefold() for name in files}) != len(files):
        raise ValueError("Duplicate animation paths")
    for name, digest in files.items():
        if not safe_name(name) or not name.startswith(directory) or Path(name).suffix not in {".json", ".hkx"} or not re.fullmatch('[0-9a-f]{64}', digest):
            raise ValueError(f"Invalid animation manifest entry: {name}")
    return animations


def validate_header_dependencies():
    for dependency in dependencies().get('header_dependencies', []):
        for name, digest in dependency['files'].items():
            if not safe_name(name) or not regular_file(ROOT / name, ROOT) or sha256(ROOT / name) != digest:
                raise ValueError(f"Pinned header dependency changed: {name}")


def source_files():
    validate_header_dependencies()
    validate_author_baseline()
    excluded = set(publication()['excluded_source_files'])
    paths = {ROOT / name for name in SOURCE_FILES}
    for name, expected in animation_manifest()['files'].items():
        path = ROOT / 'runtime' / name
        if not regular_file(path, ROOT / 'runtime') or sha256(path) != expected:
            raise ValueError(f'Converter base animation differs from the runtime manifest: {name}')
        paths.add(path)
    for directory, extensions in SOURCE_TYPES.items():
        paths.update(p for p in (ROOT / directory).rglob("*")
                     if p.is_file() and p.suffix.lower() in extensions
                     and p.relative_to(ROOT).as_posix() not in SOURCE_EXCLUDED)
    for path in paths:
        if path.relative_to(ROOT).as_posix() in excluded:
            raise ValueError(f'Author-removed source file was reintroduced: {path}')
        if not regular_file(path, ROOT):
            raise ValueError(f"Missing or linked source file: {path}")
        if not path.resolve().is_relative_to(ROOT):
            raise ValueError(f"Source file escaped project: {path}")
        if any(parent.is_symlink() for parent in path.parents if parent != ROOT and parent.is_relative_to(ROOT)):
            raise ValueError(f"Linked source directory: {path}")
    return sorted(paths)


def runtime_files(runtime_assets, dll):
    baseline = runtime_baseline()
    excluded = set(publication()['excluded_runtime_files'])
    files = {name: runtime_assets / name for name in baseline}
    files.update({
        "SKSE/Plugins/FreeClimb.dll": dll,
        "SKSE/Plugins/FreeClimb.ini": ROOT / "config/FreeClimb.ini",
    })
    files.update({"Interface/Translations/" + name: ROOT / "translations" / name
                  for name in TRANSLATION_FILES})
    animations = animation_manifest()
    animation_files = animations.get("files", {})
    directory = "meshes/actors/character/animations/FreeClimb/"
    if directory + "pack.json" not in animation_files or directory + "skeleton.json" not in animation_files:
        raise ValueError("Missing editable pack or skeleton manifest")
    if sum(name.endswith(".hkx") for name in animation_files) != 25:
        raise ValueError("The default release must contain 25 HKX files for 31 slots")
    for name, expected_hash in animation_files.items():
        source = runtime_assets / name
        if not name.startswith(directory) or ".." in Path(name).parts or Path(name).suffix not in {".json", ".hkx"}:
            raise ValueError(f"Unexpected animation asset path: {name}")
        if not regular_file(source, runtime_assets) or sha256(source) != expected_hash:
            raise ValueError(f"Missing or changed bundled animation asset: {name}")
        files[name] = source
    if any(name.endswith(".motion") for name in files):
        raise ValueError("Legacy motion libraries are not runtime files")
    for name, source in files.items():
        if name in excluded or 'license' in name.casefold() or 'notice' in name.casefold():
            raise ValueError(f'Non-runtime legal document entered Nexus package: {name}')
        if not safe_name(name) or not source.is_file() or source.is_symlink():
            raise ValueError(f"Missing or linked runtime input: {source}")
        expected = baseline.get(name)
        if expected and sha256(source) != expected:
            raise ValueError(f"Preserved runtime asset changed: {name}")
    return files


def dependency_files():
    files = {}
    records = []
    pinned = dependencies()["dependencies"]
    if len(pinned) != 5:
        raise ValueError("Expected the five pinned compiled dependencies")
    bundled_path = ROOT / 'DEPENDENCY-SOURCES.json'
    if bundled_path.exists():
        if not regular_file(bundled_path, ROOT):
            raise ValueError('Linked or invalid corresponding-source manifest')
        bundled = json.loads(bundled_path.read_text(encoding='utf-8'))
        bundled_records = bundled.get('dependencies', [])
        if bundled.get('schema') != 1 or not isinstance(bundled_records, list) or len(bundled_records) != 5:
            raise ValueError('Invalid corresponding-source manifest schema or dependency count')
        pinned_names = {item['name'] for item in pinned}
        if len(pinned_names) != 5 or any(not isinstance(item, dict) for item in bundled_records) or {item.get('name') for item in bundled_records} != pinned_names:
            raise ValueError('Corresponding-source dependencies do not match the lock')
        by_name = {item['name']: item for item in bundled_records}
        for dependency in pinned:
            record = by_name[dependency['name']]
            if any(record.get(field) != dependency[field] for field in ('name', 'directory', 'repository', 'commit')):
                raise ValueError(f"Corresponding-source pin mismatch: {dependency['name']}")
            relative_directory = dependency['directory']
            if not safe_name(relative_directory) or not relative_directory.startswith('external/'):
                raise ValueError(f'Unsafe dependency directory: {relative_directory}')
            directory = ROOT / relative_directory
            if not directory.is_dir() or not directory.resolve().is_relative_to(ROOT.resolve()):
                raise ValueError(f'Missing or escaped bundled dependency: {directory}')
            expected = record.get('files')
            if not isinstance(expected, dict) or not expected or len({name.casefold() for name in expected}) != len(expected):
                raise ValueError(f'Invalid bundled dependency file map: {directory}')
            for name, digest in expected.items():
                if not safe_name(name) or not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest):
                    raise ValueError(f'Invalid bundled dependency file entry: {name}')
            actual = set()
            for source in directory.rglob('*'):
                if source.is_symlink() or getattr(source.stat(), 'st_file_attributes', 0) & 0x400:
                    raise ValueError(f'Linked bundled dependency entry: {source}')
                if source.is_file():
                    actual.add(source.relative_to(directory).as_posix())
            if actual != set(expected):
                raise ValueError(f'Bundled dependency file set mismatch: {directory}')
            for relative, digest in expected.items():
                source = directory / relative
                if not regular_file(source, ROOT) or sha256(source) != digest:
                    raise ValueError(f'Bundled dependency hash mismatch: {source}')
                files[f'{relative_directory}/{relative}'] = source
        return files, bundled
    for dependency in pinned:
        directory = ROOT / dependency["directory"]
        actual = subprocess.check_output(["git", "-C", str(directory), "rev-parse", "HEAD"], text=True).strip()
        if actual != dependency["commit"]:
            raise ValueError(f"Dependency pin mismatch: {directory}")
        subprocess.run(["git", "-C", str(directory), "diff", "--quiet", "HEAD", "--"], check=True)
        entries = subprocess.check_output(["git", "-C", str(directory), "ls-files", "--stage", "-z"]).decode("utf-8").split("\0")
        hashes = {}
        omitted = []
        for entry in entries:
            if not entry:
                continue
            metadata, relative = entry.split("\t", 1)
            mode = metadata.split()[0]
            if mode == "160000":
                omitted.append({"path": relative, "reason": "unused optional submodule; VR disabled"})
                continue
            if mode == "120000" and relative in {".cody.md", ".cursorrules", ".factory/AGENTS.md", "COPILOT.md"}:
                omitted.append({"path": relative, "reason": "editor instruction alias; not a compiler input"})
                continue
            if relative.startswith("tests/REL/") and Path(relative).suffix.lower() in {".bin", ".csv"}:
                omitted.append({"path": relative, "reason": "external Address Library test data; not a compiler input"})
                continue
            source = directory / relative
            if mode not in {"100644", "100755"} or not safe_name(relative) or not regular_file(source, directory):
                raise ValueError(f"Unsupported dependency entry: {source}")
            hashes[relative] = sha256(source)
            files[f"{dependency['directory']}/{relative}"] = source
        records.append({**dependency, "files": hashes, "omitted_non_build_inputs": omitted})
    return files, {"schema": 1, "dependencies": records}


def make_zip(destination, files):
    temporary = destination.with_suffix(".zip.tmp")
    with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name, source in sorted(files.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, source.read_bytes(), compresslevel=9)
    with zipfile.ZipFile(temporary) as archive:
        if archive.testzip() is not None:
            raise ValueError(f"Archive CRC failure: {temporary}")
    temporary.replace(destination)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-assets", type=Path, default=ROOT / "runtime")
    parser.add_argument("--dll", type=Path, default=ROOT / "build-multiruntime/Release/FreeClimb.dll")
    arguments = parser.parse_args()
    release_version = version()
    files = runtime_files(arguments.runtime_assets.resolve(), arguments.dll.resolve())
    sources = {p.relative_to(ROOT).as_posix(): p for p in source_files()}
    policy = publication()
    package = ROOT / "package-nexus"
    release = ROOT / "release"
    diagnostics = ROOT / "diagnostics" / "publication"
    for directory in (package, release, diagnostics):
        if directory.is_symlink() or not directory.resolve().is_relative_to(ROOT):
            raise ValueError(f"Package directory escaped project: {directory}")
        directory.mkdir(parents=True, exist_ok=True)
    existing = {p.relative_to(package).as_posix() for p in package.rglob("*") if p.is_file()}
    if existing - files.keys():
        raise ValueError(f"Unexpected existing stage files: {sorted(existing - files.keys())}")
    for name, source in files.items():
        target = package / name
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.is_symlink() or not target.resolve().is_relative_to(package.resolve()):
            raise ValueError(f"Package output escaped stage: {target}")
        shutil.copy2(source, target)
    hashes = {name: sha256(package / name) for name in sorted(files)}
    (diagnostics / "SHA256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    runtime_zip = release / policy['runtime_archive']
    make_zip(runtime_zip, {name: package / name for name in files})
    dependency_sources, dependency_manifest = dependency_files()
    dependency_manifest_path = diagnostics / "DEPENDENCY-SOURCES.json"
    dependency_manifest_path.write_text(json.dumps(dependency_manifest, indent=2) + "\n")
    complete_sources = {**sources, **dependency_sources, "DEPENDENCY-SOURCES.json": dependency_manifest_path}
    complete_zip = release / policy['source_archive']
    make_zip(complete_zip, complete_sources)
    report = {
        "version": release_version,
        "runtime_files": len(files), "source_files": len(sources),
        "runtime_zip": str(runtime_zip), "runtime_sha256": sha256(runtime_zip),
        "corresponding_source_zip": str(complete_zip), "corresponding_source_sha256": sha256(complete_zip),
        "corresponding_source_files": len(complete_sources),
        "dll_sha256": hashes["SKSE/Plugins/FreeClimb.dll"],
        "manifest": str(diagnostics / "SHA256.json"),
    }
    (diagnostics / "package.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
