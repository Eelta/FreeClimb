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


def standalone_member(content, member, metadata):
    data = bytearray(content)
    sections = [struct.unpack_from('<7I', data, 84 + 48 * i) for i in range(3)]
    pointers, fixups = {}, {}
    for index, (start, local, global_, virtuals, *_) in enumerate(sections):
        for begin, end, width in ((local, global_, 8), (global_, virtuals, 12)):
            for offset in range(begin, end - width + 1, width):
                row = start + offset
                source = struct.unpack_from('<I', data, row)[0]
                if source == 0xffffffff:
                    break
                target_section = index if width == 8 else struct.unpack_from('<I', data, row + 4)[0]
                target = struct.unpack_from('<I', data, row + width - 4)[0]
                pointers[start + source] = sections[target_section][0] + target
                fixups[start + source] = (row + width - 4, sections[target_section][0])
    def text(address):
        return bytes(data[address:data.index(0, address)]).decode('ascii')
    root_section, root_offset = struct.unpack_from('<2I', data, 24)
    root = sections[root_section][0] + root_offset
    variants = pointers[root]
    count = struct.unpack_from('<I', data, root + 8)[0]
    container = binding = container_variant = None
    for i in range(count):
        variant = variants + i * 24
        kind = text(pointers[variant + 8])
        if kind == 'hkaAnimationContainer':
            container, container_variant = pointers[variant + 16], variant
        elif kind == 'hkaAnimationBinding' and text(pointers[variant]) == member:
            binding = pointers[variant + 16]
    if container is None or binding is None:
        raise ValueError('Missing named fixture member: ' + member)
    animation = pointers[binding + 24]
    if 'frameRange' in metadata:
        first, last = metadata['frameRange']
        tracks = struct.unpack_from('<I', data, animation + 24)[0]
        transforms = pointers[animation + 56]
        count = struct.unpack_from('<I', data, animation + 64)[0]
        duration = struct.unpack_from('<f', data, animation + 20)[0]
        if tracks != 99 or count % tracks or not 0 <= first < last < count // tracks:
            raise ValueError('Invalid canonical timeline fixture range')
        source_frames = count // tracks
        interval = duration / (source_frames - 1)
        begin, end = interval * first, interval * last
        struct.pack_into('<f', data, animation + 20, interval * (last - first))
        row, base = fixups[animation + 56]
        selected = transforms + first * tracks * 48
        struct.pack_into('<I', data, row, selected - base)
        size = (last - first + 1) * tracks
        struct.pack_into('<2I', data, animation + 64, size, 0x80000000 | size)
        shift = metadata.get('rootShift', [0, 0, 0])
        for frame in range(last - first + 1):
            at = selected + frame * tracks * 48
            xyz = struct.unpack_from('<3f', data, at)
            struct.pack_into('<3f', data, at, *(xyz[k] - shift[k] for k in range(3)))
        if animation + 40 in pointers:
            annotations = pointers[animation + 40]
            for track in range(struct.unpack_from('<I', data, animation + 48)[0]):
                header = annotations + track * 24
                count = struct.unpack_from('<I', data, header + 16)[0]
                if not count:
                    continue
                events, kept = pointers[header + 8], []
                for event in range(count):
                    at = events + event * 16
                    time = struct.unpack_from('<f', data, at)[0]
                    target = pointers[at + 8]
                    marker = text(target).startswith('FreeClimb.SourceMotionBaked')
                    if marker or begin - .00005 <= time <= end + .00005:
                        kept.append((0 if marker else max(0, min(end - begin, time - begin)), target))
                for event, (time, target) in enumerate(kept):
                    at = events + event * 16
                    struct.pack_into('<f', data, at, time)
                    row, base = fixups[at + 8]
                    struct.pack_into('<I', data, row, target - base)
                struct.pack_into('<2I', data, header + 16, len(kept), 0x80000000 | len(kept))
        if animation + 32 in pointers:
            reference = pointers[animation + 32]
            samples, size = pointers[reference + 56], struct.unpack_from('<I', data, reference + 64)[0]
            if size != source_frames:
                raise ValueError('Reference fixture does not use the transform timeline grid')
            values = [struct.unpack_from('<4f', data, samples + frame * 16) for frame in range(first, last + 1)]
            for frame, value in enumerate(values):
                struct.pack_into('<4f', data, samples + frame * 16, *(value[k] - values[0][k] for k in range(4)))
            struct.pack_into('<f', data, reference + 48, interval * (last - first))
            struct.pack_into('<2I', data, reference + 64, len(values), 0x80000000 | len(values))
    for source, target in ((root, container_variant), (container + 32, pointers[container + 32]), (container + 48, pointers[container + 48])):
        row, base = fixups[source]
        struct.pack_into('<I', data, row, target - base)
        struct.pack_into('<2I', data, source + 8, 1, 0x80000001)
    for source, target in ((pointers[container + 32], animation), (pointers[container + 48], binding)):
        row, base = fixups[source]
        struct.pack_into('<I', data, row, target - base)
    return data


class MigrationToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        OPTIONS.work.mkdir(parents=True, exist_ok=True)
        cls.temporary = tempfile.TemporaryDirectory(prefix='migration-', dir=OPTIONS.work)
        cls.work = Path(cls.temporary.name)
        cls.original = OPTIONS.pack.resolve()
        cls.pack = cls.work / 'source'
        shutil.copytree(cls.original.parent, cls.pack)
        for config in list((cls.pack / 'configs').glob('*.json')):
            document = json.loads(config.read_text())
            for metadata in document.get('clips', [document]):
                if metadata.get('member'):
                    content = standalone_member((cls.original.parent / metadata['file']).read_bytes(), metadata['member'], metadata)
                    (cls.pack / (metadata['slot'] + '.hkx')).write_bytes(content)
                    metadata['file'] = metadata['slot'] + '.hkx'
                    for key in ('member', 'frameRange', 'rootShift'):
                        metadata.pop(key, None)
                (cls.pack / 'configs' / (metadata['slot'] + '.json')).write_text(json.dumps(metadata), encoding='utf-8')
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
        self.assertEqual(len(slots), 31)
        self.assertIn('contextMantle', slots)
        for retired in ('mantle', 'step', 'toFree', 'toBraced', 'freeHang', 'runDown', 'dropCatch', 'contextRegrab', 'flipUp', 'flipLeft', 'flipRight', 'sprintCatch'):
            self.assertNotIn(retired, slots)
            self.assertFalse((output / (retired + '.hkx')).exists())
            self.assertFalse((output / 'configs' / (retired + '.json')).exists())
        self.assertEqual(len([p for p in output.rglob('*') if p.is_file()]), 64)
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
