import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("freeclimb_authoring_core", ROOT / "tools/authoring/core.py")
core = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = core
SPEC.loader.exec_module(core)
sys.path.insert(0, str(ROOT / "tools/authoring"))
import app


class Value:
    def __init__(self, text=""):
        self.text = text

    def get(self):
        return self.text

    def set(self, text):
        self.text = text


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def fingerprint(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob("*") if p.is_file()}


class FakeNative:
    def __init__(self, fail=False):
        self.fail = fail
        self.calls = []
        self.staged_files = None
        self.selected = None

    def inspect(self, path):
        self.calls.append(("inspect", Path(path)))
        return {"ok": True, "duration": 2.0, "frames": 121, "tracks": 99}

    def validate(self, path):
        self.calls.append(("validate", Path(path)))
        pack = core.load_template(path)
        self.staged_files = fingerprint(pack.root)
        self.selected = pack.configurations["up"]
        if self.fail:
            raise core.AuthoringError("模拟原生校验拒绝")
        return {"ok": True, "loaded": len(core.ACTIVE_SLOTS), "error": "", "slots": []}


class AuthoringToolTests(unittest.TestCase):
    def setUp(self):
        work = Path(os.environ.get("FREECLIMB_TEST_WORK", ROOT / "tools/authoring/work"))
        work.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="tests-", dir=work)
        self.directory = Path(self.temp.name)
        self.base = self.directory / "基础包 空格"
        self.pack = self.base / "pack.json"
        manifest = {"format": "FreeClimbAnimationPack", "version": 1, "skeleton": "skeleton.json", "motions": []}
        write_json(self.base / "skeleton.json", {"test": "original fixture; native mocked"})
        for slot in core.ACTIVE_SLOTS:
            config = {"format": "FreeClimbClip", "version": 1, "slot": slot, "file": f"{slot}.hkx",
                      "stride": 90.0, "height": 0.0, "travel": [0.0, 0.0, 0.0],
                      "contacts": [[1.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]}
            if slot == "contextHopLeft":
                config.update(path=[[0, 0, 0, 0], [1, 1, 0, 0]], sourceHands=[[0, .1], [0, .2]],
                              targetHands=[[.7, .8], [.8, .9]], verticalBlend=[.2, .7])
            write_json(self.base / f"configs/{slot}.json", config)
            (self.base / f"{slot}.hkx").write_bytes(b"ORIGINAL-" + slot.encode())
            manifest["motions"].append({"slot": slot, "config": f"configs/{slot}.json"})
        write_json(self.pack, manifest)
        (self.base / "unreferenced-large-asset.fbx").write_bytes(b"DO NOT COPY")
        self.hkx = self.directory / "替换 动作.hkx"
        self.hkx.write_bytes(b"ORIGINAL INDEPENDENT REPLACEMENT FIXTURE")
        self.output = self.directory / "输出 覆盖.zip"
        self.work = self.directory / "work"

    def tearDown(self):
        self.temp.cleanup()

    def export(self, options=None, native=None, slot="up", output=None):
        return core.export_override(self.pack, slot, self.hkx, output or self.output,
                                    options, native or FakeNative(), self.work)

    def test_single_slot_archive_preserves_all_inputs(self):
        before = fingerprint(self.base)
        hkx_hash = hashlib.sha256(self.hkx.read_bytes()).hexdigest()
        native = FakeNative()
        result = self.export(native=native)
        self.assertEqual(result["loaded"], 35)
        with zipfile.ZipFile(self.output) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(set(archive.namelist()), {core.PREFIX + "up.hkx", core.PREFIX + "configs/up.json"})
            self.assertEqual(archive.read(core.PREFIX + "up.hkx"), self.hkx.read_bytes())
            self.assertEqual(json.loads(archive.read(core.PREFIX + "configs/up.json")), core.read_json(self.base / "configs/up.json"))
        self.assertEqual(len(native.staged_files), 72)
        self.assertNotIn("unreferenced-large-asset.fbx", native.staged_files)
        self.assertEqual(fingerprint(self.base), before)
        self.assertEqual(hashlib.sha256(self.hkx.read_bytes()).hexdigest(), hkx_hash)
        self.assertFalse(list(self.work.iterdir()))

    def test_curve_phase_fades_and_overlap(self):
        intervals = [core.SupportInterval(0, .25, .75, .25)]
        curve = core.generate_contacts(1, intervals, 9)
        self.assertEqual([r[0] for r in curve], [0, 0, 0, .5, 1, .5, 0, 0, 0])
        self.assertTrue(all(row[1:] == [0, 0, 0] for row in curve))
        overlap = core.generate_contacts(1, intervals + [core.SupportInterval(0, .375, .625, 0)], 9)
        self.assertEqual(overlap[3][0], 1)
        self.assertEqual(overlap[5][0], 1)
        full = core.generate_contacts(2, [core.SupportInterval(3, 0, 2, 0)])
        self.assertEqual(len(full), 121)
        self.assertEqual(full[0][3], 1)
        self.assertEqual(full[-1][3], 1)
        self.assertEqual(len(core.generate_contacts(.01, [], None)), 2)

    def test_invalid_intervals_and_numbers_are_rejected(self):
        cases = [core.SupportInterval(4, 0, 1, 0), core.SupportInterval(0, -.1, .5, 0),
                 core.SupportInterval(0, .5, .5, 0), core.SupportInterval(0, 0, 1.1, 0),
                 core.SupportInterval(0, 0, 1, .51), core.SupportInterval(0, 0, 1, -.1),
                 core.SupportInterval(0, float("nan"), 1, 0), core.SupportInterval(0, 0, 1, float("inf"))]
        for interval in cases:
            with self.subTest(interval=interval), self.assertRaises(core.AuthoringError):
                core.generate_contacts(1, [interval])
        for duration in (0, -1, 10.1, float("nan"), True):
            with self.subTest(duration=duration), self.assertRaises(core.AuthoringError):
                core.generate_contacts(duration, [])
        with self.assertRaises(core.AuthoringError):
            core.generate_contacts(1, [], 1202)

    def test_template_and_special_fields_unchanged(self):
        pack = core.load_template(self.pack)
        template = pack.configurations["contextHopLeft"]
        before = copy.deepcopy(template)
        config = core.configure_clip(template, 2, core.EditOptions(stride=75, travel=(None, 2, None)))
        self.assertEqual(template, before)
        for key in ("contacts", "path", "sourceHands", "targetHands", "verticalBlend"):
            self.assertEqual(config[key], template[key])
        self.assertEqual(config["stride"], 75)
        self.assertEqual(config["travel"], [0, 2, 0])

    def test_manual_contacts_use_new_duration(self):
        native = FakeNative()
        options = core.EditOptions(contact_mode="manual", intervals=[core.SupportInterval(1, 1, 2, .25)])
        self.export(options, native)
        curve = native.selected["contacts"]
        self.assertEqual(len(curve), 121)
        self.assertEqual(curve[0], [0, 0, 0, 0])
        self.assertEqual(curve[90][1], 1)
        self.assertEqual(curve[-1][1], 0)

    def test_sparse_hkx_cannot_silently_erase_support_marks(self):
        native = FakeNative()
        native.inspect = lambda path: {"ok": True, "duration": 10.0, "frames": 2, "tracks": 99}
        self.output.write_bytes(b"KEEP OUTPUT")
        options = core.EditOptions(contact_mode="manual", intervals=[core.SupportInterval(0, 1, 9, .1)])
        with self.assertRaisesRegex(core.AuthoringError, "未覆盖任何有效 HKX 采样帧"):
            self.export(options, native)
        self.assertEqual(self.output.read_bytes(), b"KEEP OUTPUT")
        self.assertEqual(native.calls, [])
        native.inspect = lambda path: {"ok": True, "duration": 2.0, "frames": 5, "tracks": 99}
        options.intervals = [core.SupportInterval(0, 0, 2, .25)]
        self.export(options, native)
        self.assertEqual(len(native.selected["contacts"]), 5)
        self.assertEqual([row[0] for row in native.selected["contacts"]], [0, 1, 1, 1, 0])
        options.intervals.append(core.SupportInterval(0, .10, .20, 0))
        with self.assertRaises(core.AuthoringError):
            self.export(options, native)

    def test_active_slots_exclude_removed_animations(self):
        self.assertEqual(len(core.ACTIVE_SLOTS), 35)
        self.assertEqual(core.ACTIVE_SLOTS[-1], "contextMantle")
        self.assertFalse({"mantle", "step", "contextRegrab"} & set(core.ACTIVE_SLOTS))
        with self.assertRaises(core.AuthoringError):
            self.export(slot="contextRegrab")
        self.assertFalse(self.output.exists())

    def test_path_escape_and_duplicate_paths(self):
        for value in ("../outside.json", "/absolute.json", "C:\\evil.json", "..\\outside.json", "a/../b", "a//b", "a\x00b"):
            with self.subTest(path=value), self.assertRaises(core.AuthoringError):
                core.relative_file(value)
        manifest = core.read_json(self.pack)
        manifest["motions"][0]["config"] = "../outside.json"
        write_json(self.pack, manifest)
        with self.assertRaises(core.AuthoringError):
            core.load_template(self.pack)

    def test_shared_or_case_aliased_files_cannot_change_two_slots(self):
        config = core.read_json(self.base / "configs/down.json")
        for name in ("up.hkx", "UP.HKX"):
            with self.subTest(path=name):
                config["file"] = name
                write_json(self.base / "configs/down.json", config)
                with self.assertRaisesRegex(core.AuthoringError, "文件路径重复"):
                    core.load_template(self.pack)

    def test_incomplete_or_retired_slots_and_duplicate_json_rejected(self):
        manifest = core.read_json(self.pack)
        missing = copy.deepcopy(manifest)
        missing["motions"].pop()
        write_json(self.pack, missing)
        with self.assertRaises(core.AuthoringError):
            core.load_template(self.pack)
        for retired in ("mantle", "step", "runDown", "dropCatch", "freeHang"):
            manifest["motions"][0]["slot"] = retired
            write_json(self.pack, manifest)
            with self.subTest(slot=retired), self.assertRaises(core.AuthoringError):
                core.load_template(self.pack)
        with self.assertRaisesRegex(core.AuthoringError, "重复字段"):
            core.parse_json('{"file":"up.hkx","file":"outside.hkx"}')

    def test_empty_manual_mode_cannot_silently_unload_every_limb(self):
        with self.assertRaisesRegex(core.AuthoringError, "至少需要一条"):
            self.export(core.EditOptions(contact_mode="manual"))
        self.assertFalse(self.output.exists())

    def test_symlink_escape_is_rejected(self):
        outside = self.directory / "outside.hkx"
        outside.write_bytes(b"outside")
        link = self.base / "escape.hkx"
        try:
            link.symlink_to(outside)
        except OSError:
            self.skipTest("Symlink creation is unavailable")
        with self.assertRaises(core.AuthoringError):
            core.contained(self.base.resolve(), "escape.hkx")

    def test_missing_validator_cannot_export(self):
        self.output.write_bytes(b"KEEP EXISTING OUTPUT")
        with self.assertRaisesRegex(core.AuthoringError, "缺少原生校验组件"):
            self.export(native=core.NativeTool(self.directory / "missing.exe"))
        self.assertEqual(self.output.read_bytes(), b"KEEP EXISTING OUTPUT")

    def test_rejection_and_zip_failure_preserve_existing_output(self):
        self.output.write_bytes(b"KEEP EXISTING OUTPUT")
        before = fingerprint(self.base)
        with self.assertRaises(core.AuthoringError):
            self.export(native=FakeNative(fail=True))
        self.assertEqual(self.output.read_bytes(), b"KEEP EXISTING OUTPUT")
        with patch.object(core.zipfile.ZipFile, "write", side_effect=OSError("simulated disk failure")):
            with self.assertRaises(OSError):
                self.export()
        self.assertEqual(self.output.read_bytes(), b"KEEP EXISTING OUTPUT")
        self.assertEqual(fingerprint(self.base), before)
        self.assertFalse(list(self.work.iterdir()))
        self.assertFalse(list(self.output.parent.glob(".freeclimb-*.zip")))

    def test_export_cannot_write_inside_base_or_over_input(self):
        for target in (self.base / "override.zip", self.hkx, self.pack):
            with self.subTest(path=target), self.assertRaises(core.AuthoringError):
                self.export(output=target)

    def test_advanced_config_is_validated_and_cannot_redirect_slot(self):
        config = core.load_template(self.pack).configurations["up"]
        config["file"] = "../escape.hkx"
        with self.assertRaises(core.AuthoringError):
            self.export(core.EditOptions(advanced_json=json.dumps(config)))
        config["file"] = "up.hkx"
        config["height"] = 200
        native = FakeNative(fail=True)
        with self.assertRaises(core.AuthoringError):
            self.export(core.EditOptions(advanced_json=json.dumps(config)), native)
        self.assertEqual(native.selected["height"], 200)
        self.assertEqual(native.calls[-1][0], "validate")
        self.assertFalse(self.output.exists())

    def test_native_command_uses_utf8_no_shell_and_checks_exit(self):
        executable = self.directory / "检查 工具.exe"
        executable.write_bytes(b"not executed")
        native = core.NativeTool(executable)
        response = subprocess.CompletedProcess([], 0, '{"ok":true,"duration":2,"frames":121,"tracks":99}', "")
        with patch.object(core.subprocess, "run", return_value=response) as run:
            result = native.inspect(self.hkx)
            args, kwargs = run.call_args
            self.assertEqual(args[0], [str(executable.resolve()), "inspect", str(self.hkx.resolve())])
            self.assertIs(kwargs["shell"], False)
            self.assertEqual(kwargs["encoding"], "utf-8")
            self.assertEqual(result["duration"], 2)
        response.returncode = 1
        with patch.object(core.subprocess, "run", return_value=response), self.assertRaises(core.AuthoringError):
            native.inspect(self.hkx)
        response.returncode, response.stdout = 0, '{"ok":true,"duration":NaN,"frames":121,"tracks":99}'
        with patch.object(core.subprocess, "run", return_value=response), self.assertRaises(core.AuthoringError):
            native.inspect(self.hkx)

    def test_gui_inspection_uses_exact_duration_and_resets_confirmed_marks(self):
        for duration in (1.1666666269302368, 1.2999999523162842, 1.8666666746139526):
            report = {"ok": True, "duration": duration, "frames": 79, "tracks": 99}
            state = SimpleNamespace(path_vars={"hkx": Value(str(self.hkx))},
                                    intervals=[core.SupportInterval(0, 0, .5, .1)],
                                    inspection=report, inspected_path=self.hkx,
                                    end=Value(), contact_mode=Value("manual"), status=Value(),
                                    metadata=SimpleNamespace(configure=lambda **kwargs: None),
                                    refresh_intervals=lambda: None,
                                    run=lambda message, operation, complete: complete(report))
            with patch.object(app.messagebox, "askyesno", return_value=True):
                app.AuthoringApp.inspect(state)
            self.assertEqual(float(state.end.get()), duration)
            self.assertEqual(state.intervals, [])
            self.assertEqual(state.contact_mode.get(), "template")
            core.generate_contacts(duration, [core.SupportInterval(0, 0, float(state.end.get()), .06)], 79)

    def test_native_rejection_includes_bounded_slot_reasons(self):
        executable = self.directory / "validator.exe"
        executable.write_bytes(b"not executed")
        report = {"ok": False, "loaded": 30, "error": "all 35 active slots must load successfully",
                  "slots": [{"slot": "up", "status": "rejected", "file": "up.hkx", "reason": "Animated joint translation/scale is unsupported"},
                            {"slot": "hang", "status": "loaded", "file": "hang.hkx", "reason": "THIS MUST NOT APPEAR"}] +
                           [{"slot": f"bad-{i}", "status": "missing", "file": f"missing-{i}.hkx", "reason": "X" * 2000} for i in range(6)]}
        response = subprocess.CompletedProcess([], 1, json.dumps(report), "")
        with patch.object(core.subprocess, "run", return_value=response), self.assertRaises(core.AuthoringError) as raised:
            core.NativeTool(executable).validate(self.pack)
        text = str(raised.exception)
        self.assertIn("up [up.hkx]", text)
        self.assertIn("Animated joint translation/scale is unsupported", text)
        self.assertNotIn("THIS MUST NOT APPEAR", text)
        self.assertIn("另有 2 个槽失败", text)
        self.assertLess(len(text), 5000)

    def test_gui_cancelling_new_hkx_restores_previous_selection_and_marks(self):
        previous = self.directory / "previous.hkx"
        marks = [core.SupportInterval(0, 0, .5, .1)]
        state = SimpleNamespace(path_vars={"hkx": Value(str(self.hkx))}, intervals=marks,
                                inspected_path=previous, inspection={"duration": 1})
        with patch.object(app.messagebox, "askyesno", return_value=False):
            app.AuthoringApp.inspect(state)
        self.assertEqual(state.path_vars["hkx"].get(), str(previous))
        self.assertIs(state.intervals, marks)


if __name__ == "__main__":
    unittest.main(verbosity=2)
