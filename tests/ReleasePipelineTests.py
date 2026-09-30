import json
import os
import configparser
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import package
import validate


class ReleasePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scratch = Path(os.environ.get("FREECLIMB_TEST_WORK", ROOT / "build-settings-check"))
        scratch.mkdir(exist_ok=True)
        cls.temporary = tempfile.TemporaryDirectory(prefix='release-script-tests-', dir=scratch)
        cls.root = Path(cls.temporary.name)
        cls.manifest = package.animation_manifest()
        for name in cls.manifest['files']:
            target = cls.root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / 'runtime' / name, target)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_pack_structure(self):
        result = validate.validate_animation_pack(self.root, self.manifest)
        self.assertEqual((result['clips'], result['bones'], result['pack_files']), (35, 99, 72))

    def test_independent_installation_keeps_ownership_compatibility(self):
        ini = configparser.ConfigParser()
        ini.read_string('[General]\nEnabled=1\n[Custom]\nRequireSkyParkour=retained\n')
        validate.validate_independent_installation(ini, b'SkyParkourOngoing\0SkyParkourSliding\0')

    def test_independent_installation_rejects_old_default_key(self):
        for section, key in [('Compatibility', 'RequireSkyParkour'), ('cOmPaTiBiLiTy', 'rEqUiReSkYpArKoUr')]:
            ini = configparser.ConfigParser()
            ini.read_string(f'[{section}]\n{key}=0\n')
            with self.assertRaisesRegex(ValueError, 'obsolete SkyParkour prerequisite key'):
                validate.validate_independent_installation(ini, b'')

    def test_independent_installation_rejects_module_name(self):
        ini = configparser.ConfigParser()
        ini.read_string('[General]\nEnabled=1\n')
        for encoding in ('ascii', 'utf-16-le', 'utf-16-be'):
            for spelling in ('SkyParkourNG.dll', 'SKYPARKOURNG.DLL'):
                with self.assertRaisesRegex(ValueError, 'module prerequisite name'):
                    validate.validate_independent_installation(ini, b'prefix' + spelling.encode(encoding) + b'suffix')

    def test_pack_rejects_traversal(self):
        path = self.root / self.manifest['pack']
        original = path.read_bytes()
        value = json.loads(original)
        value['skeleton'] = '../skeleton.json'
        path.write_text(json.dumps(value), encoding='utf-8')
        try:
            with self.assertRaisesRegex(ValueError, 'Invalid pack path'):
                validate.validate_animation_pack(self.root, self.manifest)
        finally:
            path.write_bytes(original)

    def test_pack_rejects_slot_aliasing(self):
        path = self.root / self.manifest['pack']
        original = path.read_bytes()
        value = json.loads(original)
        value['motions'][1]['slot'] = value['motions'][0]['slot']
        path.write_text(json.dumps(value), encoding='utf-8')
        try:
            with self.assertRaisesRegex(ValueError, '35 active slots exactly once'):
                validate.validate_animation_pack(self.root, self.manifest)
        finally:
            path.write_bytes(original)

    def test_duplicate_json_keys(self):
        path = self.root / 'duplicate.json'
        path.write_text('{"a":1,"a":2}', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON key'):
            validate.read_json(path)

    def test_removed_regrab_is_not_packaged(self):
        prefix = "meshes/actors/character/animations/FreeClimb/"
        for name in ("contextRegrab.hkx", "configs/contextRegrab.json"):
            self.assertNotIn(prefix + name, self.manifest["files"])
            self.assertFalse((ROOT / "runtime" / prefix / name).exists())
            self.assertEqual(len(package.publication()["removed_runtime_files"][prefix + name]), 64)
        self.assertNotIn("tools/retarget_context_regrab.py", package.SOURCE_FILES)
        self.assertFalse((ROOT / "tools/retarget_context_regrab.py").exists())

    def test_retarget_source_allowlist(self):
        for name in ("tools/retarget_threepeat.py", "tools/MigrateAnimationPack.cpp"):
            self.assertIn(name, package.SOURCE_FILES)
        self.assertFalse(any(name.startswith("assets/") for name in package.SOURCE_FILES))

    def test_runtime_allowlist(self):
        files = package.runtime_files(ROOT / 'runtime', ROOT / 'config/FreeClimb.ini')
        self.assertEqual(len(files), len(package.runtime_baseline()) + 72 + 2 + len(package.TRANSLATION_FILES))
        self.assertFalse(any(name.lower().endswith(('.fbx', '.motion')) for name in files))
        self.assertFalse(any('license' in name.casefold() or 'notice' in name.casefold() for name in files))
        self.assertTrue(all('Interface/Translations/' + name in files for name in package.TRANSLATION_FILES))

    def test_default_translations_match_compiled_fallback(self):
        result = validate.validate_translations(ROOT / 'translations', ROOT / 'src/TranslationDefaults.h')
        self.assertEqual(set(result), set(package.TRANSLATION_FILES))

    def test_translation_format_rejects_broken_edits(self):
        path = self.root / 'translation.txt'
        for suffix in ('$FC_LANGUAGE_NAME\tAgain\n', '$FC_LABEL\tBad##id\n',
                       '$FC_LABEL\tBad\x00value\n', '$FC_LABEL\tBad\\tvalue\n',
                       '$FC_LABEL value without tab\n'):
            path.write_text('$FC_LANGUAGE_NAME\tExample\n' + suffix, encoding='utf-8')
            with self.assertRaises(ValueError):
                validate.read_translation(path)
        path.write_bytes(b'\xff\xfe\x00')
        with self.assertRaises(UnicodeError):
            validate.read_translation(path)

    def test_translation_text_is_not_a_format_string(self):
        path = self.root / 'translation.txt'
        path.write_text('$FC_LANGUAGE_NAME\tExample\n$FC_LABEL\t100% %n {value}\\nNext\\\\path\n', encoding='utf-8')
        self.assertEqual(validate.read_translation(path)['$FC_LABEL'], '100% %n {value}\nNext\\path')

    def test_translation_format_matches_runtime_rules(self):
        path = self.root / 'translation.txt'
        for content in ('$FC_LANGUAGE_NAME\tEnglish\\nOther', '$FC_LANGUAGE_NAME\t' + 'a' * 257,
                        '$FC_LANGUAGE_NAME\t' + '中' * 86, '$FC_LANGUAGE_NAME\t',
                        ' #indented comment', '   ', '#bad\rcomment', '#bad\x00comment',
                        '\ufeff$FC_LABEL\tDouble BOM', '$FC_' + 'A' * 125 + '\tToo long'):
            path.write_bytes(b'\xef\xbb\xbf' + content.encode('utf-8'))
            with self.assertRaises(ValueError):
                validate.read_translation(path)
        path.write_text('#comment\n;comment\n\n$FC_LABEL\t\n', encoding='utf-8')
        self.assertEqual(validate.read_translation(path), {'$FC_LABEL': ''})
        path.write_text('$FC_LANGUAGE_NAME\t' + 'a' * 256, encoding='utf-8')
        self.assertEqual(len(validate.read_translation(path)['$FC_LANGUAGE_NAME']), 256)

    def test_sources_and_pins(self):
        files = {p.relative_to(ROOT).as_posix() for p in package.source_files()}
        self.assertTrue({'tools/AuthoringCli.cpp', 'tools/authoring/app.py', 'tools/authoring/core.py',
                         'tools/authoring/Start.cmd', 'tools/authoring/README.md',
                         'tools/authoring/bin/FreeClimbAuthoring.exe', 'tests/AuthoringToolTests.py'} <= files)
        self.assertFalse(any(name.startswith('tools/authoring/work/') for name in files))
        self.assertTrue({'src/vendor/SKSEMenuFramework/SOURCE.txt', 'src/vendor/SKSEMenuFramework/LICENSE',
                         'external/nlohmann/json.hpp', 'external/nlohmann/LICENSE.MIT'} <= files)
        self.assertFalse(any(name.lower().endswith(('.fbx', '.motion')) for name in files))
        self.assertEqual(len(package.dependencies()['dependencies']), 5)
        self.assertTrue({'README.md', 'README.zh-CN.md', 'tools/publication.json', 'translations/FreeClimb_english.txt',
                         'translations/FreeClimb_chinese.txt', 'src/TranslationCatalog.cpp'} <= files)
        self.assertTrue(set(package.publication()['protected_files']) <= files)
        self.assertFalse(set(package.publication()['excluded_source_files']) & files)

    def test_public_names_are_separate_from_internal_version(self):
        policy = package.publication()
        self.assertEqual(policy['runtime_archive'], 'FreeClimb-Nexus.zip')
        self.assertEqual(policy['source_archive'], 'FreeClimb-source.zip')

    def test_retired_assets_are_not_required_or_shipped(self):
        retired = {'mantle', 'step', 'toFree', 'toBraced', 'freeHang', 'runDown', 'dropCatch', 'contextRegrab'}
        files = package.runtime_files(ROOT / 'runtime', ROOT / 'build-multiruntime/Release/FreeClimb.dll')
        self.assertFalse(any(Path(name).stem in retired for name in files))
        self.assertFalse(any('authoring' in name.lower() for name in files))
        removal = package.publication()['removed_runtime_files']
        self.assertEqual(len(removal), 16)
        self.assertEqual({Path(name).stem for name in removal}, retired)
        self.assertFalse(set(removal) & files.keys())

    def test_author_baseline_rejects_changed_notice(self):
        with patch.object(package, 'sha256', return_value='0' * 64):
            with self.assertRaisesRegex(ValueError, 'Author-protected file changed'):
                package.validate_author_baseline()

    def test_archive_determinism_and_content(self):
        source = self.root / 'source.txt'
        source.write_text('release fixture\n', encoding='utf-8')
        first, second = self.root / 'first.zip', self.root / 'second.zip'
        files = {'safe/source.txt': source}
        package.make_zip(first, files)
        package.make_zip(second, files)
        self.assertEqual(first.read_bytes(), second.read_bytes())
        self.assertEqual(validate.archive_matches(first, files)['files'], 1)
        source.write_text('changed', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'content mismatch'):
            validate.archive_matches(first, files)

    def test_archive_path_rules(self):
        for name in ('../file', '/root/file', 'C:/file', 'folder\\file'):
            self.assertFalse(package.safe_name(name))

    def test_bundled_dependencies_without_git(self):
        fixture = self.root / 'bundled-source'
        (fixture / 'tools').mkdir(parents=True)
        lock = package.dependencies()
        (fixture / 'tools/dependencies.json').write_text(json.dumps(lock), encoding='utf-8')
        records = []
        for dependency in lock['dependencies']:
            source = fixture / dependency['directory'] / 'include/fixture.h'
            source.parent.mkdir(parents=True)
            source.write_text(dependency['name'], encoding='utf-8')
            records.append({**dependency, 'files': {'include/fixture.h': package.sha256(source)}, 'omitted_non_build_inputs': []})
        bundled = {'schema': 1, 'dependencies': records}
        manifest = fixture / 'DEPENDENCY-SOURCES.json'
        manifest.write_text(json.dumps(bundled), encoding='utf-8')
        first = fixture / records[0]['directory'] / 'include/fixture.h'
        with patch.object(package, 'ROOT', fixture), patch.object(package.subprocess, 'check_output', side_effect=AssertionError('Git must not be called')), patch.object(package.subprocess, 'run', side_effect=AssertionError('Git must not be called')):
            files, result = package.dependency_files()
            self.assertEqual(len(files), 5)
            self.assertEqual(result, bundled)
            original = first.read_bytes()
            first.write_bytes(b'modified')
            with self.assertRaisesRegex(ValueError, 'hash mismatch'):
                package.dependency_files()
            first.write_bytes(original)
            extra = first.parent / 'unexpected.h'
            extra.write_text('extra', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'file set mismatch'):
                package.dependency_files()
            extra.unlink()
            records[0]['commit'] = '0' * 40
            manifest.write_text(json.dumps(bundled), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'pin mismatch'):
                package.dependency_files()
            records[0]['commit'] = lock['dependencies'][0]['commit']
            bundled['schema'] = 2
            manifest.write_text(json.dumps(bundled), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'schema'):
                package.dependency_files()


if __name__ == '__main__':
    unittest.main()
