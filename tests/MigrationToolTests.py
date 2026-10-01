import argparse
import hashlib
import json
import re
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

parser = argparse.ArgumentParser()
parser.add_argument('--tool', type=Path, required=True)
parser.add_argument('--pack', type=Path, required=True)
parser.add_argument('--work', type=Path, required=True)
OPTIONS, remaining = parser.parse_known_args()
sys.argv = [sys.argv[0], *remaining]


class MigrationToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OPTIONS.work.mkdir(parents=True, exist_ok=True)
        cls.temporary = tempfile.TemporaryDirectory(prefix='migration-', dir=OPTIONS.work)
        cls.work = Path(cls.temporary.name)
        cls.original = OPTIONS.pack.resolve()
        cls.pack = cls.work / 'source'
        shutil.copytree(cls.original.parent, cls.pack)
        skeleton = json.loads((cls.pack / 'skeleton.json').read_text())
        data = bytearray(struct.pack('<4I', 0x344D4346, 2, 99, 42))
        pose = bytearray()
        for bone in skeleton['bones']:
            name = bone['name'].encode()
            transform = struct.pack('<10f', *bone['t'], *bone['q'], *bone['s'])
            data += struct.pack('<iI', bone['parent'], len(name)) + name + transform
            pose += transform
        header = (Path(__file__).resolve().parents[1] / 'src/MotionSlots.h').read_text()
        names = re.findall(r'"([A-Za-z]*)"', header.split('motionSlotNames', 1)[1].split('}};', 1)[0])
        for slot in names[:42]:
            metadata = json.loads((cls.pack / 'configs' / (slot + '.json')).read_text()) if slot else {'stride': 0, 'height': 0, 'travel': [0, 0, 0]}
            data += struct.pack('<fI5f', 1, 2, metadata['stride'], metadata['height'], *metadata['travel'])
            for _ in range(2):
                data += pose + struct.pack('<4f', 1, 1, 0, 0)
        cls.legacy = cls.work / 'legacy42.motion'
        cls.legacy.write_bytes(data)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def invoke(self, output, *extra):
        command = [str(OPTIONS.tool.resolve()), str(self.legacy), str(self.pack), str(output), *map(str, extra)]
        return subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace', shell=False)

    def test_missing_original_never_creates_output(self):
        source = self.pack / 'contextMantle.hkx'
        temporary = self.pack / 'contextMantle.hkx.saved'
        source.rename(temporary)
        output = self.work / 'missing-output'
        try:
            result = self.invoke(output)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Missing original active animation: contextMantle', result.stderr)
            self.assertFalse(output.exists())
        finally:
            temporary.rename(source)

    def test_unexpected_argument_never_creates_output(self):
        output = self.work / 'invalid-output'
        result = self.invoke(output, self.pack / 'pack.json')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Usage:', result.stderr)
        self.assertFalse(output.exists())

    def test_all_active_animations_migrate_and_retired_assets_are_absent(self):
        output = self.work / 'converted'
        before = {p.relative_to(self.pack): hashlib.sha256(p.read_bytes()).digest() for p in self.pack.rglob('*') if p.is_file()}
        result = self.invoke(output)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        manifest = json.loads((output / 'pack.json').read_text())
        slots = {item['slot'] for item in manifest['motions']}
        self.assertEqual(len(slots), 35)
        self.assertIn('contextMantle', slots)
        for retired in ('mantle', 'step', 'toFree', 'toBraced', 'freeHang', 'runDown', 'dropCatch', 'contextRegrab'):
            self.assertNotIn(retired, slots)
            self.assertFalse((output / (retired + '.hkx')).exists())
            self.assertFalse((output / 'configs' / (retired + '.json')).exists())
        self.assertEqual(len([p for p in output.rglob('*') if p.is_file()]), 72)
        for slot in slots:
            self.assertEqual((output / (slot + '.hkx')).read_bytes(), (self.pack / (slot + '.hkx')).read_bytes())
            original = json.loads((self.pack / 'configs' / (slot + '.json')).read_text())
            migrated = json.loads((output / 'configs' / (slot + '.json')).read_text())
            self.assertEqual(migrated['slot'], slot)
            for key in ('stride', 'height', 'travel'):
                self.assertEqual(migrated[key], original[key])
        after = {p.relative_to(self.pack): hashlib.sha256(p.read_bytes()).digest() for p in self.pack.rglob('*') if p.is_file()}
        self.assertEqual(before, after)
        result = self.invoke(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Output directory must be empty', result.stderr)


if __name__ == '__main__':
    unittest.main(verbosity=2)
