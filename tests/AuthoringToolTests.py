import copy
import hashlib
import importlib.util
import json
import os
import shutil
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
        self.configurations = pack.configurations
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
        self.assertEqual(result["loaded"], 31)
        with zipfile.ZipFile(self.output) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(set(archive.namelist()), {core.PREFIX + "up.hkx", core.PREFIX + "configs/up.json"})
            self.assertEqual(archive.read(core.PREFIX + "up.hkx"), self.hkx.read_bytes())
            self.assertEqual(json.loads(archive.read(core.PREFIX + "configs/up.json")), core.read_json(self.base / "configs/up.json"))
        self.assertEqual(len(native.staged_files), 64)
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
        self.assertEqual(len(core.ACTIVE_SLOTS), 31)
        self.assertEqual(core.ACTIVE_SLOTS[-1], "contextMantle")
        self.assertFalse({"mantle", "step", "contextRegrab", "flipUp", "flipLeft", "flipRight", "sprintCatch"} & set(core.ACTIVE_SLOTS))
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

    def test_shared_files_export_only_the_selected_slot(self):
        config = core.read_json(self.base / "configs/down.json")
        config["file"] = "up.hkx"
        write_json(self.base / "configs/down.json", config)
        native = FakeNative()
        self.export(native=native)
        self.assertEqual(native.selected["file"], "converted/up.hkx")
        self.assertEqual(native.configurations["down"]["file"], "up.hkx")
        self.assertEqual(native.staged_files["up.hkx"], fingerprint(self.base)["up.hkx"])
        with zipfile.ZipFile(self.output) as archive:
            self.assertEqual(set(archive.namelist()), {core.PREFIX + "converted/up.hkx", core.PREFIX + "configs/up.json"})
        config["file"] = "UP.HKX"
        write_json(self.base / "configs/down.json", config)
        with self.assertRaisesRegex(core.AuthoringError, "文件路径重复"):
            core.load_template(self.pack)

    def test_grouped_base_exports_preserve_all_other_members(self):
        groups = {"wallRun.hkx": ("runUp", "runLeft", "runRight", "runDiagonalLeft", "runDiagonalRight",
                                  "runLaunch", "runCatch", "runLaunchLeft", "runLaunchRight", "sideBrace"),
                  "contextHop.hkx": ("contextHang", "contextHopLeft", "contextHopRight")}
        for file, slots in groups.items():
            (self.base / file).write_bytes(("ORIGINAL-GROUP-" + file).encode())
            for slot in slots:
                config_path = self.base / f"configs/{slot}.json"
                config = core.read_json(config_path)
                config.update(file=file, member=slot)
                write_json(config_path, config)
                (self.base / f"{slot}.hkx").unlink()
        self.assertEqual(len(core.load_template(self.pack).files), 53)
        before = fingerprint(self.base)
        for slot in ("runUp", "contextHopLeft"):
            native = FakeNative()
            self.export(native=native, slot=slot)
            self.assertEqual(len(native.staged_files), 54)
            self.assertEqual(native.configurations[slot]["file"], f"converted/{slot}.hkx")
            self.assertNotIn("member", native.configurations[slot])
            for file, slots in groups.items():
                self.assertEqual(native.staged_files[file], before[file])
                for other in slots:
                    if other != slot:
                        self.assertEqual(native.configurations[other]["file"], file)
                        self.assertEqual(native.configurations[other]["member"], other)
            with zipfile.ZipFile(self.output) as archive:
                self.assertEqual(set(archive.namelist()), {core.PREFIX + f"converted/{slot}.hkx", core.PREFIX + f"configs/{slot}.json"})
            self.assertEqual(fingerprint(self.base), before)

    def test_shared_override_does_not_reuse_another_slots_converted_path(self):
        config = core.read_json(self.base / "configs/down.json")
        (self.base / "converted").mkdir()
        (self.base / "converted/up.hkx").write_bytes(b"OTHER-SLOT")
        config["file"] = "converted/up.hkx"
        write_json(self.base / "configs/down.json", config)
        up = core.read_json(self.base / "configs/up.json")
        up.update(file="converted/up.hkx", member="up")
        write_json(self.base / "configs/up.json", up)
        native = FakeNative()
        self.export(native=native)
        self.assertEqual(native.selected["file"], "converted/up-1.hkx")
        self.assertEqual(native.staged_files[str(Path("converted/up.hkx"))], fingerprint(self.base)[str(Path("converted/up.hkx"))])

    def test_complete_group_config_delegates_atomic_native_export(self):
        manifest = core.read_json(self.pack)
        for group, slots in core.ACTION_GROUPS.items():
            clips = []
            for slot in slots:
                config = core.read_json(self.base / f"configs/{slot}.json")
                config.update(file=f"{group}.hkx", member=slot)
                clips.append(config)
            write_json(self.base / f"configs/{group}.json", {"format": "FreeClimbActionGroup", "version": 1, "group": group, "clips": clips})
            (self.base / f"{group}.hkx").write_bytes(group.encode())
            for entry in manifest["motions"]:
                if entry["slot"] in slots:
                    entry["config"] = f"configs/{group}.json"
        write_json(self.pack, manifest)
        pack = core.load_template(self.pack)
        self.assertEqual(len(pack.files), 42)
        before = fingerprint(self.base)
        native = FakeNative()
        with patch.object(native, "export_group", create=True, return_value={"files": ["group.hkx", "group.json"], "loaded": 31}) as export:
            self.export(native=native, slot="runUp")
            args = export.call_args.args
            self.assertEqual(args[0], self.pack.resolve())
            self.assertEqual(args[1], self.hkx.resolve())
            self.assertEqual(args[2], pack.configurations["runUp"])
            self.assertEqual(args[3], self.output.resolve())
        self.assertEqual(fingerprint(self.base), before)

    def test_native_group_job_uses_unicode_and_removes_temporary_job(self):
        executable = self.directory / "检查工具.exe"
        executable.write_bytes(b"not executed")
        native = core.NativeTool(executable)
        self.work.mkdir()
        captured = []
        def call(command, path):
            captured.append((command, core.read_json(path)))
            return {"ok": True, "loaded": 31, "files": ["group.hkx", "group.json"]}
        config = core.load_template(self.pack).configurations["up"]
        with patch.object(native, "call", side_effect=call):
            native.export_group(self.pack, self.hkx, config, self.output, self.work)
        self.assertEqual(captured, [("export-group", {"pack": str(self.pack), "input": str(self.hkx), "config": config, "output": str(self.output), "work": str(self.work)})])
        self.assertFalse(list(self.work.iterdir()))

    def test_complete_timeline_group_accepts_ranges_and_rejects_wrong_direction(self):
        source = core.read_json(ROOT / "runtime" / core.PREFIX / "configs/runUp.json")
        core.group_header(source)
        self.assertEqual(source["version"], 2)
        mutations = (
            lambda value: value["sequences"].pop(),
            lambda value: value["sequences"][0]["launch"].update(member="runLeft"),
            lambda value: value["sequences"][0]["launch"].update(frameRange=[0, 1201]),
            lambda value: value["sequences"][0]["catch"].update(frameRange=[0, 1]),
            lambda value: value["sequences"][0]["launch"].update(rootShift=[0, float("nan"), 0]),
            lambda value: value["clips"][0].update(frameRange=[True, 2]),
            lambda value: value["sequences"][0]["launch"].update(file="other.hkx"),
        )
        for mutation in mutations:
            invalid = copy.deepcopy(source)
            mutation(invalid)
            with self.assertRaises(core.AuthoringError):
                core.group_header(invalid)
        clip = copy.deepcopy(source["clips"][0])
        clip.update(version=2, authoredPlayback={"version": 1})
        edited = core.configure_clip(clip, 2, core.EditOptions(stride=81))
        self.assertEqual(edited["stride"], 81)
        self.assertEqual(edited["authoredPlayback"], {"version": 1})
        edited["authoredPlayback"]["version"] = 2
        with self.assertRaises(core.AuthoringError):
            core.clip_header(edited, edited["slot"])

    def contextual_groups(self):
        manifest = core.read_json(self.pack)
        for direction in ("contextHopLeft", "contextHopRight"):
            preparation = core.read_json(self.base / "configs/contextHang.json")
            preparation.update(file=f"{direction}.hkx", member=direction, frameRange=[0, 16])
            main = core.read_json(self.base / f"configs/{direction}.json")
            main.update(file=f"{direction}.hkx", member=direction, frameRange=[16, 94])
            ending = copy.deepcopy(preparation)
            ending["frameRange"] = [94, 105]
            document = {"format": "FreeClimbActionGroup", "version": 2, "group": "contextHop",
                        "direction": direction, "clips": [preparation, main],
                        "sequences": [{"slot": direction, "prepare": preparation, "catch": ending}]}
            write_json(self.base / f"configs/{direction}.json", document)
            for entry in manifest["motions"]:
                if entry["slot"] == direction or entry["slot"] == "contextHang" and direction == "contextHopLeft":
                    entry["config"] = f"configs/{direction}.json"
        write_json(self.pack, manifest)

    def test_private_wall_brace_stays_in_its_direction_file(self):
        template = core.read_json(ROOT / "runtime" / core.PREFIX / "configs/runUp.json")
        for direction in ("runLeft", "runRight", "runDiagonalLeft", "runDiagonalRight"):
            source = copy.deepcopy(template)
            source["direction"] = direction
            source["sequences"][0]["slot"] = direction
            launch = "runLaunchLeft" if "Left" in direction else "runLaunchRight"
            for part in source["clips"] + [source["sequences"][0]["launch"], source["sequences"][0]["catch"]]:
                part.pop("references", None)
                part["slot"] = direction if part["slot"] == "runUp" else launch if part["slot"] == "runLaunch" else part["slot"]
                part.update(file=f"{direction}.hkx", member=direction)
            brace = core.read_json(self.base / "configs/sideBrace.json")
            brace.update(file=f"{direction}.hkx", member=direction + "Brace")
            source["sequences"][0]["brace"] = brace
            if direction == "runLeft":
                source["clips"].append(copy.deepcopy(brace))
            core.group_header(source)
            for field, value in (("file", "sideBrace.hkx"), ("member", "otherBrace"), ("slot", "runUp")):
                invalid = copy.deepcopy(source)
                invalid["sequences"][0]["brace"][field] = value
                with self.assertRaises(core.AuthoringError):
                    core.group_header(invalid)
            if direction == "runLeft":
                invalid = copy.deepcopy(source)
                invalid["sequences"][0].pop("brace")
                with self.assertRaises(core.AuthoringError):
                    core.group_header(invalid)
                invalid = copy.deepcopy(source)
                invalid["clips"][-1]["stride"] += 1
                with self.assertRaises(core.AuthoringError):
                    core.group_header(invalid)

    def test_contextual_directions_select_private_groups_and_keep_legacy_reading(self):
        self.contextual_groups()
        pack = core.load_template(self.pack)
        for side in ("contextHopLeft", "contextHopRight"):
            self.assertEqual(pack.config_paths[side], f"configs/{side}.json")
            self.assertEqual(pack.configurations[side]["file"], f"{side}.hkx")
            self.assertEqual(pack.configurations[side]["member"], side)
            own = pack.groups[pack.config_paths[side]]
            self.assertEqual({clip["slot"] for clip in own["clips"]}, {"contextHang", side})
            self.assertEqual(own["sequences"][0]["prepare"], own["clips"][0])
            self.assertNotEqual(own["sequences"][0]["catch"]["frameRange"], own["clips"][0]["frameRange"])
        self.assertEqual(len(pack.files), 62)
        before = fingerprint(self.base)
        for side in ("contextHopLeft", "contextHopRight"):
            native = FakeNative()
            files = [core.PREFIX + f"{side}.hkx", core.PREFIX + f"configs/{side}.json"]
            with patch.object(native, "export_group", create=True, return_value={"files": files, "loaded": 31}) as export:
                result = self.export(native=native, slot=side)
                self.assertEqual(export.call_args.args[2], pack.configurations[side])
                self.assertEqual(result["files"], files)
        self.assertEqual(fingerprint(self.base), before)

    def test_contextual_timeline_rejects_cross_direction_and_invalid_sections(self):
        self.contextual_groups()
        original = core.read_json(self.base / "configs/contextHopLeft.json")
        mutations = (
            lambda value: value.pop("direction"),
            lambda value: value.update(direction="runLeft"),
            lambda value: value["clips"][1].update(slot="contextHopRight"),
            lambda value: value["clips"][1].update(member="contextHopRight"),
            lambda value: value["clips"][1].update(file="contextHopRight.hkx"),
            lambda value: value["sequences"][0].update(slot="contextHopRight"),
            lambda value: value["sequences"][0]["prepare"].update(stride=5),
            lambda value: value["sequences"][0]["catch"].update(member="contextHopRight"),
            lambda value: value["sequences"][0]["catch"].update(file="contextHopRight.hkx"),
            lambda value: value["sequences"][0]["catch"].update(slot="contextHopRight"),
            lambda value: value["sequences"][0]["catch"].update(frameRange=[15, 104]),
            lambda value: value["sequences"][0]["catch"].update(rootShift=[0, float("nan"), 0]),
            lambda value: value["sequences"][0]["catch"].update(frameRange=[0, 1201]),
            lambda value: value["clips"].append(copy.deepcopy(value["clips"][0])),
        )
        for mutation in mutations:
            invalid = copy.deepcopy(original)
            mutation(invalid)
            with self.assertRaises(core.AuthoringError):
                core.group_header(invalid)

    def test_contextual_full_source_keeps_private_calibration_members(self):
        self.contextual_groups()
        for direction in ("contextHopLeft", "contextHopRight"):
            source = core.read_json(self.base / f"configs/{direction}.json")
            for part in source["clips"] + [source["sequences"][0]["prepare"], source["sequences"][0]["catch"]]:
                part.pop("frameRange", None)
                part.update(version=2, authoredPlayback={"version": 1})
            source["clips"][0]["member"] = direction + "Prepare"
            source["sequences"][0]["prepare"] = copy.deepcopy(source["clips"][0])
            source["sequences"][0]["catch"]["member"] = direction + "Catch"
            core.group_header(source)
            for mutation in (
                lambda value: value["clips"][1].pop("authoredPlayback"),
                lambda value: value["clips"][0].update(member=direction),
                lambda value: value["sequences"][0]["catch"].update(member=direction),
                lambda value: value["sequences"][0]["catch"].update(file="another.hkx"),
                lambda value: value["sequences"][0]["prepare"].update(member=direction + "Catch"),
            ):
                invalid = copy.deepcopy(source)
                mutation(invalid)
                with self.assertRaises(core.AuthoringError):
                    core.group_header(invalid)

    def test_private_playback_references_remain_inside_the_owner_file(self):
        for owner, role, slot in (("runLaunch", "launchApproach", "runUp"),
                                  ("kickLeft", "kickTakeoff", "kickUp"),
                                  ("kickLeft", "kickLanding", "hopLeft"),
                                  ("kickRight", "kickLanding", "hopRight"),
                                  ("kickUp", "kickLanding", "hopUp"),
                                  ("kickLeft", "kickRunBrace", "sideBrace"),
                                  ("kickUp", "kickRunLanding", "runUp")):
            source = core.read_json(self.base / f"configs/{owner}.json")
            reference = core.read_json(self.base / f"configs/{slot}.json")
            reference.update(file=source["file"], member=owner + role)
            source["references"] = {role: reference}
            core.clip_header(source, owner)
            for field, value in (("file", "other.hkx"), ("slot", "hang"), ("member", "bad/name"), ("references", {})):
                invalid = copy.deepcopy(source)
                invalid["references"][role][field] = value
                with self.assertRaises(core.AuthoringError):
                    core.clip_header(invalid, owner)
            invalid = copy.deepcopy(source)
            invalid["references"] = {"unexpected": reference}
            with self.assertRaises(core.AuthoringError):
                core.clip_header(invalid, owner)

    @unittest.skipUnless(os.environ.get("FREECLIMB_AUTHORING_EXE"), "Native group integration requires the built authoring executable")
    def test_native_complete_group_export_roundtrip(self):
        native = core.NativeTool(os.environ["FREECLIMB_AUTHORING_EXE"])
        source = ROOT / "runtime" / core.PREFIX
        before = fingerprint(source)
        hop_packages = []
        for group, slot in (("wallRun", "runUp"), ("wallRun", "runCatch"), ("wallRun", "runLaunchLeft"), ("contextHop", "contextHopLeft"), ("contextHop", "contextHopRight")):
            owner = {"runUp": "runUp", "runCatch": "runUp", "runLaunchLeft": "runLeft", "contextHopLeft": "contextHopLeft", "contextHopRight": "contextHopRight"}[slot]
            output = self.directory / f"{group}-{slot}.zip"
            try:
                result = core.export_override(source / "pack.json", slot, source / "up.hkx", output,
                                              core.EditOptions(stride=91), native, self.work)
            except core.AuthoringError as error:
                self.fail(f"{slot}: {error}")
            self.assertEqual(result["loaded"], 31)
            target = self.directory / f"{group}-{slot}"
            shutil.copytree(source, target / core.PREFIX)
            with zipfile.ZipFile(output) as archive:
                self.assertEqual(len(archive.namelist()), 2)
                configs = [name for name in archive.namelist() if name.endswith(".json")]
                self.assertEqual(configs, [core.PREFIX + f"configs/{owner}.json"])
                document = json.loads(archive.read(configs[0]))
                archive.extractall(target)
            original = core.read_json(source / f"configs/{owner}.json")
            self.assertEqual(len(document["clips"]), len(original["clips"]))
            self.assertEqual(len({clip["file"] for clip in document["clips"]}), 1)
            old = {clip["slot"]: clip for clip in original["clips"]}
            for clip in document["clips"]:
                expected = copy.deepcopy(old[clip["slot"]])
                expected["file"] = clip["file"]
                for reference in expected.get("references", {}).values():
                    reference["file"] = clip["file"]
                if clip["slot"] == slot:
                    expected["stride"] = 91
                self.assertEqual({k: v for k, v in clip.items() if k not in ("frameRange", "rootShift")},
                                 {k: v for k, v in expected.items() if k not in ("frameRange", "rootShift")})
            if group == "wallRun":
                for sequence, old_sequence in zip(document["sequences"], original["sequences"]):
                    for role in ("launch", "catch"):
                        part, expected = sequence[role], copy.deepcopy(old_sequence[role])
                        expected["file"] = part["file"]
                        for reference in expected.get("references", {}).values():
                            reference["file"] = part["file"]
                        if old[slot]["member"] == sequence["slot"] and expected["slot"] == slot:
                            expected["stride"] = 91
                        self.assertEqual({k: v for k, v in part.items() if k not in ("frameRange", "rootShift")},
                                         {k: v for k, v in expected.items() if k not in ("frameRange", "rootShift")})
            self.assertEqual(native.validate(target / core.PREFIX / "pack.json")["loaded"], 31)
            if group == "contextHop":
                opposite = "contextHopRight" if slot == "contextHopLeft" else "contextHopLeft"
                for name in (f"{opposite}.hkx", f"configs/{opposite}.json"):
                    self.assertEqual((target / core.PREFIX / name).read_bytes(), (source / name).read_bytes())
                hop_packages.append(output)
        with zipfile.ZipFile(hop_packages[0]) as left, zipfile.ZipFile(hop_packages[1]) as right:
            self.assertFalse(set(left.namelist()) & set(right.namelist()))
        results = []
        for index, archives in enumerate((hop_packages, list(reversed(hop_packages)))):
            target = self.directory / f"both-directions-{index}"
            shutil.copytree(source, target / core.PREFIX)
            for archive in archives:
                with zipfile.ZipFile(archive) as opened:
                    opened.extractall(target)
            self.assertEqual(native.validate(target / core.PREFIX / "pack.json")["loaded"], 31)
            results.append(fingerprint(target))
        self.assertEqual(results[0], results[1])
        self.assertEqual(fingerprint(source), before)

    def test_action_group_rejects_missing_duplicate_and_unknown_stages(self):
        manifest = core.read_json(self.pack)
        clips = [core.read_json(self.base / f"configs/{slot}.json") for slot in core.ACTION_GROUPS["contextHop"]]
        for entry in manifest["motions"]:
            if entry["slot"] in core.ACTION_GROUPS["contextHop"]:
                entry["config"] = "configs/contextHop.json"
        write_json(self.pack, manifest)
        for invalid in (clips[:-1], clips + [clips[0]], [clips[0], clips[0], clips[2]], [clips[0], clips[1], dict(clips[2], slot="hang")]):
            write_json(self.base / "configs/contextHop.json", {"format": "FreeClimbActionGroup", "version": 1, "group": "contextHop", "clips": invalid})
            with self.assertRaises(core.AuthoringError):
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
        report = {"ok": False, "loaded": 30, "error": "all 31 active slots must load successfully",
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
