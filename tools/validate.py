import argparse
import configparser
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import struct
import wave
import zipfile

import package


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_independent_installation(ini, dll):
    for section in ini.sections():
        if section.casefold() == 'compatibility':
            require(not any(key.casefold() == 'requireskyparkour' for key in ini[section]),
                    'Default INI contains obsolete SkyParkour prerequisite key')
    lowered = dll.lower()
    for encoding in ('ascii', 'utf-16-le', 'utf-16-be'):
        require('skyparkourng.dll'.encode(encoding) not in lowered,
                'DLL contains the removed SkyParkour module prerequisite name')


def read_translation(path):
    blob = path.read_bytes()
    require(len(blob) <= 1024 * 1024, f'Translation file too large: {path}')
    text = blob[2:].decode('utf-16-le') if blob.startswith(b'\xff\xfe') else blob.removeprefix(b'\xef\xbb\xbf').decode('utf-8')
    require('\x00' not in text, f'NUL in translation file: {path}')
    values = {}
    for line in text.split('\n'):
        line = line.removesuffix('\r')
        require(len(line.encode('utf-8')) <= 16384, f'Translation line too long: {path}')
        require('\r' not in line, f'Bare carriage return in translation file: {path}')
        if not line or line.startswith((';', '#')):
            continue
        key, delimiter, value = line.partition('\t')
        require(delimiter and len(key) <= 128 and re.fullmatch(r'\$FC_[A-Z0-9_]+', key), f'Invalid translation record: {path}')
        require(key not in values, f'Duplicate translation key: {key}')
        require(not re.search(r'[\x00-\x1f\x7f]', value) and '##' not in value,
                f'Invalid translation value: {key}')
        require('\\' not in re.sub(r'\\[n\\]', '', value), f'Invalid translation escape: {key}')
        values[key] = re.sub(r'\\(n|\\)', lambda match: '\n' if match[1] == 'n' else '\\', value)
    if '$FC_LANGUAGE_NAME' in values:
        name = values['$FC_LANGUAGE_NAME']
        require(name and '\n' not in name and len(name.encode('utf-8')) <= 256, f'Invalid language name: {path}')
    return values


def validate_translations(directory, defaults_header):
    records = re.findall(r'\{\s*"(\$FC_[A-Z0-9_]+)"\s*,\s*("(?:\\.|[^"\\])*")\s*\}',
                         defaults_header.read_text(encoding='utf-8'))
    defaults = {key: json.loads(value) for key, value in records}
    require(len(defaults) == len(records) and len(defaults) >= 100, 'Invalid built-in translation catalog')
    languages = {}
    for name in package.TRANSLATION_FILES:
        path = directory / name
        require(path.read_bytes().startswith(b'\xff\xfe'), f'Default translation must use UTF-16 LE BOM: {name}')
        values = read_translation(path)
        require(values.keys() == defaults.keys(), f'Incomplete default translation: {name}')
        if name == 'FreeClimb_english.txt':
            require(values == defaults, 'English file differs from compiled fallback')
        languages[name] = {'keys': len(values), 'name': values['$FC_LANGUAGE_NAME']}
    return languages


def archive_matches(path, expected):
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        require(len(names) == len(set(n.casefold() for n in names)), 'Duplicate ZIP entries')
        require(set(names) == set(expected), f'Archive allowlist mismatch: {path}')
        for name in names:
            parts = PurePosixPath(name)
            require(not parts.is_absolute() and '..' not in parts.parts and '\\' not in name and ':' not in name, f'Unsafe ZIP path: {name}')
            require((archive.getinfo(name).external_attr >> 16) & 0xF000 != 0xA000, f'Linked ZIP entry: {name}')
            require(archive.read(name) == expected[name].read_bytes(), f'Archive content mismatch: {name}')
        require(archive.testzip() is None, f'CRC failure: {path}')
    return {'path': str(path), 'files': len(names), 'bytes': path.stat().st_size, 'sha256': package.sha256(path)}


def dll_version(data, expected):
    require(data[:2] == b'MZ', 'Invalid DLL DOS header')
    pe = struct.unpack_from('<I', data, 0x3C)[0]
    require(data[pe:pe + 4] == b'PE\0\0', 'Invalid DLL PE header')
    machine, section_count = struct.unpack_from('<HH', data, pe + 4)
    require(machine == 0x8664, 'DLL must be x64')
    optional = pe + 24
    section_table = optional + struct.unpack_from('<H', data, pe + 20)[0]
    require(struct.unpack_from('<H', data, optional)[0] == 0x20B, 'DLL must use PE32+')
    def offset(rva):
        for i in range(section_count):
            size, address, raw_size, raw_offset = struct.unpack_from('<4I', data, section_table + i * 40 + 8)
            if address <= rva < address + max(size, raw_size):
                require(rva - address < raw_size, 'Unbacked PE RVA')
                return raw_offset + rva - address
        raise ValueError(f'Unmapped PE RVA: {rva:x}')
    export_rva, export_size = struct.unpack_from('<II', data, optional + 112)
    function_count, name_count, functions, names, ordinals = struct.unpack_from('<5I', data, offset(export_rva) + 20)
    symbols = {}
    for i in range(name_count):
        start = offset(struct.unpack_from('<I', data, offset(names) + 4 * i)[0])
        name = data[start:data.index(b'\0', start)].decode('ascii')
        ordinal = struct.unpack_from('<H', data, offset(ordinals) + 2 * i)[0]
        require(ordinal < function_count, 'Invalid PE export ordinal')
        symbols[name] = struct.unpack_from('<I', data, offset(functions) + 4 * ordinal)[0]
    require({'SKSEPlugin_Load', 'SKSEPlugin_Query', 'SKSEPlugin_Version'} <= symbols.keys(), 'Missing SKSE exports')
    target = symbols['SKSEPlugin_Version']
    require(not export_rva <= target < export_rva + export_size, 'Forwarded version export')
    start = offset(target)
    schema, packed = struct.unpack_from('<II', data, start)
    actual = [(packed >> 24) & 255, (packed >> 16) & 255, (packed >> 4) & 4095, packed & 15]
    require(schema == 1 and actual == expected, f'Stale or incompatible DLL version: {actual}')
    require(data[start + 8:start + 17] == b'FreeClimb', 'Unexpected DLL identity')
    author = data[start + 0x108:start + 0x208]
    require(len(author) == 256 and b'\0' in author, 'Invalid DLL author metadata')
    require(author.split(b'\0', 1)[0] == b'Epsilona', 'DLL author must be Epsilona')
    flags_ex, flags = struct.unpack_from('<2I', data, start + 0x304)
    require(flags_ex == 3, 'Expected NoStructUse and AddressLibraryV5 metadata')
    require(flags == 0, 'Expected explicit runtime list instead of unrestricted version independence')
    runtimes = struct.unpack_from('<16I', data, start + 0x30C)
    lock = package.dependencies()
    supported, skse_ids = lock['supported_runtimes'], lock.get('skse_runtime_ids')
    require(isinstance(supported, list) and 0 < len(supported) <= 16 and
            all(isinstance(value, str) for value in supported) and len(set(supported)) == len(supported),
            'Invalid supported runtime list')
    gog = {'1.6.659.0': '1.6.659.1', '1.6.1179.0': '1.6.1179.1'}
    packed_supported = []
    for runtime in supported:
        require(re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+\.0', runtime), 'Invalid EXE runtime version')
        major, minor, patch, tweak = (int(x) for x in runtime.split('.'))
        require(major <= 255 and minor <= 255 and patch <= 4095 and
                runtime == f'{major}.{minor}.{patch}.{tweak}', 'Invalid EXE runtime version')
        packed_supported.append(major << 24 | minor << 16 | patch << 4 | int(runtime in gog))
    require(skse_ids == [gog.get(runtime, runtime) for runtime in supported],
            'Invalid EXE to SKSE runtime mapping')
    require(list(runtimes[:len(supported)]) == packed_supported, 'Explicit runtime whitelist mismatch')
    require(all(value in (0, 1 << 24) for value in runtimes[len(supported):]), 'Unexpected runtime compatibility tail')
    return {'architecture': 'x64', 'version': actual, 'author': 'Epsilona', 'exports': sorted(symbols),
            'supported_runtimes': supported, 'skse_runtime_ids': skse_ids,
            'address_library_v5': True, 'structs_cross_version': True,
            'version_independence_ex': flags_ex, 'version_independence': flags}


def validate_esp_author(blob):
    require(len(blob) >= 24 and blob[:4] == b'TES4', 'Missing ESP TES4 header')
    end = 24 + struct.unpack_from('<I', blob, 4)[0]
    require(end <= len(blob), 'Truncated ESP TES4 record')
    offset, authors = 24, []
    while offset < end:
        require(offset + 6 <= end, 'Truncated TES4 subrecord header')
        tag, size = struct.unpack_from('<4sH', blob, offset)
        offset += 6
        require(offset + size <= end, 'Truncated TES4 subrecord data')
        if tag == b'CNAM':
            authors.append(blob[offset:offset + size])
        offset += size
    require(authors == [b'Epsilona\0'], 'TES4 author must be Epsilona')


def read_json(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, f'Duplicate JSON key: {path}: {key}')
            result[key] = value
        return result
    return json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=unique)


def validate_animation_pack(root, manifest):
    directory = root / 'meshes/actors/character/animations/FreeClimb'
    def relative(name, suffix):
        require(isinstance(name, str) and package.safe_name(name) and name.endswith(suffix), f'Invalid pack path: {name}')
        path = directory / name
        require(package.regular_file(path, directory), f'Missing or linked pack file: {name}')
        require(path.relative_to(root).as_posix() in manifest['files'], f'Unlisted animation input: {name}')
        return path
    def format_check(value, expected):
        versions = (1, 2) if expected == 'FreeClimbActionGroup' else (1,)
        require(isinstance(value, dict) and value.get('format') == expected and value.get('version') in versions, f'Invalid {expected} document')
    def numeric(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    pack = read_json(directory / 'pack.json')
    format_check(pack, 'FreeClimbAnimationPack')
    skeleton = read_json(relative(pack['skeleton'], '.json'))
    format_check(skeleton, 'FreeClimbSkeleton')
    bones = skeleton['bones']
    require(len(bones) == 99, 'Expected 99 canonical skeleton bones')
    for index, bone in enumerate(bones):
        require(isinstance(bone['name'], str) and 0 < len(bone['name']) <= 100, 'Invalid skeleton bone name')
        require(isinstance(bone['parent'], int) and -1 <= bone['parent'] < index, 'Invalid skeleton hierarchy')
        for component, count in [('t', 3), ('q', 4), ('s', 3)]:
            require(len(bone[component]) == count and all(numeric(v) for v in bone[component]), 'Invalid skeleton transform')
        require(abs(sum(v*v for v in bone['q']) - 1) < .002, 'Invalid skeleton quaternion')
    slot_source = (package.ROOT / 'src/MotionSlots.h').read_text(encoding='utf-8')
    slots = re.findall(r'"([A-Za-z]+)"', slot_source.split('motionSlotNames', 1)[1])
    require(len(slots) == 31 and len(set(slots)) == 31, 'Invalid canonical motion-slot source')
    motions = pack['motions']
    require(len(motions) == 31 and {item['slot'] for item in motions} == set(slots), 'Pack must assign all 31 active slots exactly once')
    seen = {directory / 'pack.json', relative(pack['skeleton'], '.json')}
    directions = {'runUp', 'runLeft', 'runRight', 'runDiagonalLeft', 'runDiagonalRight'}
    owners = {name: name for name in directions}
    owners.update(runLaunch='runUp', runCatch='runUp', runLaunchLeft='runLeft', runLaunchRight='runRight', sideBrace='runLeft')
    contextual = {'contextHopLeft', 'contextHopRight'}
    owners.update(contextHang='contextHopLeft', contextHopLeft='contextHopLeft', contextHopRight='contextHopRight')
    group_documents = {}
    def timeline_range(clip):
        frames = clip.get('frameRange')
        require(isinstance(frames, list) and len(frames) == 2 and all(type(v) is int for v in frames)
                and 0 <= frames[0] < frames[1] <= 1200, 'Invalid timeline frame range')
        shift = clip.get('rootShift', [0, 0, 0])
        require(isinstance(shift, list) and len(shift) == 3 and all(numeric(v) and abs(v) <= 10000 for v in shift), 'Invalid timeline Root shift')
        return frames
    def private_references(value, owner_file):
        if isinstance(value, dict):
            if value.get('format') == 'FreeClimbClip':
                format_check(value, 'FreeClimbClip')
                require(value.get('file') == owner_file, 'Action stages and references must use their own HKX file')
            references = value.get('references')
            if references is not None:
                require(isinstance(references, dict) and 0 < len(references) <= 5
                        and set(references) <= {'launchApproach', 'kickTakeoff', 'kickLanding', 'kickRunLanding', 'kickRunBrace'}, 'Invalid private action references')
                require(all(isinstance(reference, dict) and reference.get('format') == 'FreeClimbClip'
                            and isinstance(reference.get('member'), str) and reference['member']
                            for reference in references.values()), 'Private references require named HKX members')
            for child in value.values():
                private_references(child, owner_file)
        elif isinstance(value, list):
            for child in value:
                private_references(child, owner_file)
    for item in motions:
        config_path = relative(item['config'], '.json')
        owner = owners.get(item['slot'])
        if owner:
            group = owner
            require(item['config'] == f'configs/{group}.json', 'Grouped slot must use its complete direction config')
            if group not in group_documents:
                document = read_json(config_path)
                format_check(document, 'FreeClimbActionGroup')
                clips = document.get('clips')
                hop = owner in contextual
                launch = 'contextHang' if hop else 'runLaunch' if owner == 'runUp' else 'runLaunchLeft' if owner.endswith('Left') else 'runLaunchRight'
                expected = {owner, launch} if hop else {owner, launch, 'runCatch'}
                if owner == 'runLeft':
                    expected.add('sideBrace')
                require(document.get('group') == ('contextHop' if hop else 'wallRun') and isinstance(clips, list) and len(clips) == len(expected), 'Invalid direction action group')
                require(all(isinstance(clip, dict) for clip in clips) and {clip.get('slot') for clip in clips} == expected, 'Action group must contain each stage exactly once')
                group_documents[group] = {clip['slot']: clip for clip in clips}
                require(document['version'] == 2 and document.get('direction') == owner, 'Invalid independent action direction')
                sequences = document.get('sequences')
                require(isinstance(sequences, list) and len(sequences) == 1 and isinstance(sequences[0], dict)
                        and sequences[0].get('slot') == owner, 'Independent action requires its own timeline')
                cycle = group_documents[group][owner]
                require(cycle.get('member') == owner, 'Grouped slot must select its matching animation member')
                loop = timeline_range(cycle)
                ranges = []
                roles = (('prepare', 'contextHang'), ('catch', 'contextHang')) if hop else (('launch', launch), ('catch', 'runCatch'))
                for role, slot in roles:
                    clip = sequences[0].get(role)
                    format_check(clip, 'FreeClimbClip')
                    require(hop and role == 'catch' or clip == group_documents[group][slot], 'Direction stages must match their timeline entries')
                    require(clip.get('slot') == slot and clip.get('file') == f'{owner}.hkx'
                            and clip.get('member') == owner, 'Grouped slot must select its matching animation member')
                    ranges.append(timeline_range(clip))
                require(ranges[0][1] <= loop[0] and loop[1] <= ranges[1][0], 'Action ranges must follow start, main, end order')
                if not hop and owner != 'runUp':
                    brace = sequences[0].get('brace')
                    format_check(brace, 'FreeClimbClip')
                    require(brace.get('slot') == 'sideBrace' and brace.get('file') == f'{owner}.hkx'
                            and brace.get('member') == owner + 'Brace', 'Wall-run brace must be private to its direction')
                    if owner == 'runLeft':
                        require(brace == group_documents[group]['sideBrace'], 'Canonical brace must match its direction reference')
                private_references(document, f'{owner}.hkx')
                if owner == 'runUp':
                    require(set(group_documents[group]['runLaunch'].get('references', {})) == {'launchApproach'}, 'Run entry requires its own approach reference')
            config = group_documents[group][item['slot']]
        else:
            require(config_path not in seen, 'Independent slots must have independent configs')
            config = read_json(config_path)
        format_check(config, 'FreeClimbClip')
        require(config['slot'] == item['slot'], 'Clip metadata slot mismatch')
        hkx_path = relative(config['file'], '.hkx')
        if owner:
            member = owner + 'Brace' if item['slot'] == 'sideBrace' else owner
            if item['slot'] != 'sideBrace':
                timeline_range(config)
            require(config['file'] == f'{owner}.hkx' and config.get('member') == member, 'Grouped slot must select its matching animation member')
        else:
            require(hkx_path not in seen and config.get('member', item['slot']) == item['slot'], 'Other default slots must have independent HKX files')
            private_references(config, config['file'])
            if item['slot'] in ('kickUp', 'kickLeft', 'kickRight'):
                expected_references = {'kickLanding', 'kickRunLanding', 'kickRunBrace'}
                if item['slot'] != 'kickUp':
                    expected_references.add('kickTakeoff')
                require(set(config.get('references', {})) == expected_references, 'Kick action requires its own takeoff and landing references')
        seen.update((config_path, hkx_path))
        blob = hkx_path.read_bytes()
        require(len(blob) >= 208 and blob[:8] == bytes.fromhex('57e0e05710c0c010') and b'hk_2010.2.0-r1' in blob[:64], 'Invalid Skyrim SE HKX header')
        require(numeric(config['stride']) and 0 <= config['stride'] <= 500 and numeric(config['height']) and -500 <= config['height'] <= 500, 'Invalid clip stride/height')
        require(len(config['travel']) == 3 and all(numeric(v) and -500 <= v <= 500 for v in config['travel']), 'Invalid clip travel')
        contacts = config['contacts']
        require(2 <= len(contacts) <= 1201 and all(len(row) == 4 and all(numeric(v) and 0 <= v <= 1 for v in row) for row in contacts), 'Invalid clip contact samples')
        if item['slot'] in ('contextHopLeft', 'contextHopRight'):
            require({'path', 'sourceHands', 'targetHands', 'verticalBlend'} <= config.keys(), 'Missing contextual side action profile')
        if item['slot'] == 'contextMantle':
            require({'releaseHands', 'unplant', 'replant', 'replantSamplePhase'} <= config.keys(), 'Missing contextual mantle profile')
    require({p.relative_to(root).as_posix() for p in seen} == set(manifest['files']), 'Pack references do not exactly cover the default manifest')
    return {'clips': 31, 'bones': 99, 'pack_files': len(seen), 'pack_name': pack.get('name', ''), 'format': manifest.get('format')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dll', type=Path, default=package.ROOT / 'build-multiruntime/Release/FreeClimb.dll')
    arguments = parser.parse_args()
    root = package.ROOT
    version = package.version()
    policy = package.publication()
    package.validate_author_baseline()
    stage = root / 'package-nexus'
    report_root = root / 'diagnostics' / 'publication'
    manifest = json.loads((report_root / 'SHA256.json').read_text())
    baseline = package.runtime_baseline()
    animations = package.animation_manifest()
    allowed = set(package.runtime_files(stage, arguments.dll))
    require(set(manifest) == allowed, 'Runtime manifest allowlist mismatch')
    require(not any(name.lower().endswith(('.motion', '.fbx')) for name in allowed), 'Legacy motion or source FBX entered runtime package')
    actual = {p.relative_to(stage).as_posix() for p in stage.rglob('*') if p.is_file()}
    require(actual == allowed, 'Stage allowlist mismatch')
    for name, digest in manifest.items():
        path = stage / name
        require(not path.is_symlink() and path.resolve().is_relative_to(stage.resolve()), f'Linked stage path: {name}')
        require(package.sha256(path) == digest, f'Stage hash mismatch: {name}')
    for name, digest in baseline.items():
        require(manifest[name] == digest, f'Preserved asset changed: {name}')
    for name, digest in animations['files'].items():
        require(manifest[name] == digest, f'Bundled animation hash mismatch: {name}')
    require((stage / 'SKSE/Plugins/FreeClimb.dll').read_bytes() == arguments.dll.read_bytes(), 'Staged DLL differs from build')
    require((stage / 'SKSE/Plugins/FreeClimb.ini').read_bytes() == (root / 'config/FreeClimb.ini').read_bytes(), 'Staged INI mismatch')
    ini = configparser.ConfigParser(interpolation=None, strict=True)
    ini.read(stage / 'SKSE/Plugins/FreeClimb.ini', encoding='utf-8-sig')
    require(ini.sections(), 'Empty runtime configuration')
    validate_independent_installation(ini, (stage / 'SKSE/Plugins/FreeClimb.dll').read_bytes())
    require(ini.get('Menu', 'Language') == 'english' and ini.getboolean('Stamina', 'Enabled'), 'Default menu must be English with stamina enabled')
    require(not ini.getboolean('General', 'Diagnostics'), 'Detailed diagnostics must default off')
    require(ini.getfloat('Movement', 'DownSpeed') == 78 and not ini.getboolean('General', 'LegacyAutomaticHops'), 'Movement defaults regressed')
    for name in ['Forward', 'Backward', 'Left', 'Right', 'Entry', 'RunModifier', 'Hop']:
        require(ini.has_option('Controls', name), f'Missing control setting: {name}')
    require(ini.get('Controls', 'Entry') == 'W+A+D+Space', 'Default entry combination regressed')
    require(not ini.has_option('Controls', 'EntryModifier') and not ini.has_option('Controls', 'HoldSeconds'), 'Retired entry settings remain')
    require(not ini.has_option('Animation', 'IdleBreathing'), 'Retired breathing setting remains')
    for name in baseline:
        if name.endswith('.wav'):
            with wave.open(str(stage / name), 'rb') as sound:
                require((sound.getnchannels(), sound.getsampwidth(), sound.getframerate(), sound.getcomptype()) == (1, 2, 44100, 'NONE'), f'Invalid WAV: {name}')
                samples = struct.unpack('<' + 'h' * sound.getnframes(), sound.readframes(sound.getnframes()))
                require(samples and samples[0] == samples[-1] == 0 and max(abs(v) for v in samples) < 32767, f'Clipped WAV: {name}')
    esp = (stage / 'FreeClimb.esp').read_bytes()
    tag, size, flags, form, _, record_version, _ = struct.unpack_from('<4sIIIIHH', esp)
    require(tag == b'TES4' and flags & 0x200 and record_version == 44, 'Invalid ESPFE header')
    validate_esp_author(esp)
    require(b'Skyrim.esm\0' in esp[:24 + size] and b'FC_AnimationState\0' in esp, 'Missing ESP recovery record')
    require(esp.count(b'SNDR') == 5, 'Missing sound descriptors')
    animation_report = validate_animation_pack(stage, animations)
    translation_report = validate_translations(stage / 'Interface/Translations', root / 'src/TranslationDefaults.h')
    dll = dll_version((stage / 'SKSE/Plugins/FreeClimb.dll').read_bytes(), [int(x) for x in version.split('.')] + [0])
    sources = {p.relative_to(root).as_posix(): p for p in package.source_files()}
    runtime_zip = archive_matches(root / 'release' / policy['runtime_archive'], {name: stage / name for name in allowed})
    dependency_sources, dependency_manifest = package.dependency_files()
    dependency_manifest_path = report_root / 'DEPENDENCY-SOURCES.json'
    require(json.loads(dependency_manifest_path.read_text()) == dependency_manifest, 'Corresponding-source manifest mismatch')
    complete_sources = {**sources, **dependency_sources, 'DEPENDENCY-SOURCES.json': dependency_manifest_path}
    require(not any(name.lower().endswith(('.motion', '.fbx')) for name in complete_sources), 'Source archive contains animation data')
    source_zip = archive_matches(root / 'release' / policy['source_archive'], complete_sources)
    report = {'status': 'pass', 'version': version, 'runtime': runtime_zip, 'source': source_zip, 'dll': dll, 'unchanged_assets': baseline, 'animation_pack': animation_report, 'translations': translation_report, 'scope': 'Structure, provenance hashes and both archives; not in-game validation; AE not verified in game'}
    (report_root / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
