from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import subprocess
import tempfile
import zipfile
from dataclasses import dataclass, field


ACTIVE_SLOTS = (
    "hang", "up", "down", "left", "right", "reach",
    "hopLeft", "hopRight", "hopUp", "drop", "jumpCatch", "sprintCatch",
    "dropBack", "ledgeCatch", "runUp", "runLeft", "runRight", "runDiagonalLeft",
    "runDiagonalRight", "runLaunch", "runCatch", "kickUp", "kickLeft", "kickRight",
    "flipUp", "flipLeft", "flipRight", "runLaunchLeft", "runLaunchRight",
    "sideBrace", "backFlipOut", "contextHang", "contextHopLeft", "contextHopRight",
    "contextMantle",
)
PREFIX = "meshes/actors/character/animations/FreeClimb/"
TOOL_DIRECTORY = Path(__file__).resolve().parent
MAX_HKX_BYTES = 64 * 1024 * 1024
MAX_PACK_BYTES = 256 * 1024 * 1024
LIMBS = ("左手", "右手", "左脚", "右脚")


class AuthoringError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise AuthoringError(message)


def number(value, label, low, high):
    require(isinstance(value, (int, float)) and not isinstance(value, bool), f"{label} 必须是数字。")
    require(math.isfinite(value) and low <= value <= high, f"{label} 必须在 {low} 到 {high} 之间。")
    return float(value)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"JSON 包含重复字段：{key}")
        result[key] = value
    return result


def parse_json(text, label="JSON"):
    try:
        result = json.loads(text, object_pairs_hook=unique_object,
                            parse_constant=lambda value: (_ for _ in ()).throw(AuthoringError(f"不支持的数字：{value}")))
    except (json.JSONDecodeError, RecursionError) as error:
        raise AuthoringError(f"{label} 不是有效 JSON：{error}") from error
    require(isinstance(result, dict), f"{label} 必须是 JSON 对象。")
    return result


def read_bytes(path, limit):
    try:
        with Path(path).open("rb") as stream:
            data = stream.read(limit + 1)
    except OSError as error:
        raise AuthoringError(f"无法读取 {path}：{error}") from error
    require(0 < len(data) <= limit, f"文件为空或超过大小限制：{path}")
    return data


def read_json(path, limit=256 * 1024):
    try:
        return parse_json(read_bytes(path, limit).decode("utf-8-sig"), str(path))
    except UnicodeDecodeError as error:
        raise AuthoringError(f"JSON 必须使用 UTF-8：{path}") from error


def relative_file(value):
    require(isinstance(value, str) and 0 < len(value) <= 240, "文件路径必须是非空相对路径。")
    require(not any(ord(c) < 32 for c in value) and ":" not in value, "文件路径包含非法字符。")
    require(not PureWindowsPath(value).is_absolute() and not PureWindowsPath(value).drive,
            "文件路径不能包含盘符或绝对路径。")
    normalized = value.replace("\\", "/")
    require(not normalized.startswith("/") and all(p not in ("", ".", "..") for p in normalized.split("/")),
            "文件路径不能跳出动作包，也不能含空目录或 ..。")
    return PurePosixPath(normalized).as_posix()


def contained(root, relative):
    name = relative_file(relative)
    result = (root / name).resolve()
    require(result.is_relative_to(root), f"路径跳出动作包：{relative}")
    require(result.is_file(), f"缺少基础包文件：{relative}")
    return result


@dataclass
class TemplatePack:
    path: Path
    manifest: dict
    configurations: dict
    config_paths: dict
    files: dict

    @property
    def root(self):
        return self.path.parent


def load_template(pack_path, slot=None):
    path = Path(pack_path).resolve()
    require(path.name.lower() == "pack.json", "请选择完整基础动作包的 pack.json。")
    manifest = read_json(path, 64 * 1024)
    require(manifest.get("format") == "FreeClimbAnimationPack" and manifest.get("version") == 1,
            "不支持的基础动作包格式。")
    motions = manifest.get("motions")
    require(isinstance(motions, list) and len(motions) == len(ACTIVE_SLOTS), f"基础包必须包含全部 {len(ACTIVE_SLOTS)} 个有效动作槽。")
    files = {"pack.json": path}
    file_names = {"pack.json"}
    source_names = {str(path).casefold()}
    configurations, config_paths = {}, {}

    def add_file(relative):
        name = relative_file(relative)
        require(name.casefold() not in file_names, f"基础包中的文件路径重复：{name}")
        source = contained(path.parent, name)
        require(str(source).casefold() not in source_names, f"基础包中的文件路径重复：{name}")
        file_names.add(name.casefold())
        source_names.add(str(source).casefold())
        files[name] = source
        return name

    add_file(manifest.get("skeleton"))
    for entry in motions:
        require(isinstance(entry, dict), "动作槽清单格式不正确。")
        name = entry.get("slot")
        require(name in ACTIVE_SLOTS and name not in configurations, f"未知或重复动作槽：{name}")
        config_path = add_file(entry.get("config"))
        config = read_json(files[config_path])
        require(config.get("format") == "FreeClimbClip" and config.get("version") == 1 and config.get("slot") == name,
                f"动作配置与槽名不匹配：{name}")
        add_file(config.get("file"))
        configurations[name] = config
        config_paths[name] = config_path
    require(set(configurations) == set(ACTIVE_SLOTS), "基础包缺少必需动作槽。")
    if slot is not None:
        require(slot in configurations, f"不存在的动作槽：{slot}")
    return TemplatePack(path, manifest, configurations, config_paths, files)


@dataclass(frozen=True)
class SupportInterval:
    limb: int
    start: float
    end: float
    fade: float = 0.06


def generate_contacts(duration, intervals, samples=None):
    duration = number(duration, "HKX 时长", 0.000001, 10)
    samples = max(2, min(1201, math.ceil(duration * 60) + 1)) if samples is None else samples
    require(isinstance(samples, int) and not isinstance(samples, bool) and 2 <= samples <= 1201, "接触采样行数必须是 2–1201。")
    require(len(intervals) <= 256, "支撑区间最多 256 条。")
    for interval in intervals:
        require(isinstance(interval, SupportInterval), "支撑区间格式不正确。")
        require(type(interval.limb) is int and 0 <= interval.limb < 4, "请选择四肢之一。")
        start = number(interval.start, "支撑开始秒数", 0, duration)
        end = number(interval.end, "支撑结束秒数", 0, duration)
        require(end > start, "支撑结束必须晚于开始。")
        number(interval.fade, "柔和过渡秒数", 0, (end - start) / 2)

    def smooth(value):
        value = min(1.0, max(0.0, value))
        return value * value * (3 - 2 * value)

    result = []
    sampled = set()
    for index in range(samples):
        t = duration * index / (samples - 1)
        row = [0.0] * 4
        for interval_index, interval in enumerate(intervals):
            if interval.start <= t <= interval.end:
                weight = 1.0 if interval.fade == 0 else min(smooth((t - interval.start) / interval.fade),
                                                          smooth((interval.end - t) / interval.fade))
                if weight > 0.000001:
                    sampled.add(interval_index)
                row[interval.limb] = max(row[interval.limb], weight)
        result.append(row)
    require(len(sampled) == len(intervals), "有支撑区间未覆盖任何有效 HKX 采样帧。请增加源动画采样帧，或调整区间和过渡长度。")
    return result


@dataclass
class EditOptions:
    contact_mode: str = "template"
    intervals: list[SupportInterval] = field(default_factory=list)
    stride: float | None = None
    height: float | None = None
    travel: tuple[float | None, float | None, float | None] = (None, None, None)
    advanced_json: str | None = None


def configure_clip(template, duration, options, frames=None):
    config = parse_json(options.advanced_json, "高级配置") if options.advanced_json is not None else copy.deepcopy(template)
    require(config.get("slot") == template["slot"] and config.get("file") == template["file"],
            "高级配置不能更改 slot 或 file；本工具只覆盖所选槽原来的两个文件。")
    require(config.get("format") == "FreeClimbClip" and config.get("version") == 1, "请保留配置的 format 和 version。")
    if options.stride is not None:
        config["stride"] = number(options.stride, "stride", 0, 500)
    if options.height is not None:
        config["height"] = number(options.height, "height", -500, 500)
    require(len(options.travel) == 3, "travel 必须是三分量向量。")
    travel = config.get("travel")
    require(isinstance(travel, list) and len(travel) == 3, "配置中缺少三分量 travel。")
    config["travel"] = [number(new if new is not None else old, "travel", -500, 500)
                        for old, new in zip(travel, options.travel)]
    require(options.contact_mode in ("template", "manual"), "未知接触模式。")
    if options.contact_mode == "manual":
        require(options.intervals, "手工模式至少需要一条支撑区间；未标记的肢体将保持卸载。")
        config["contacts"] = generate_contacts(duration, options.intervals, frames)
    contacts = config.get("contacts")
    require(isinstance(contacts, list) and 2 <= len(contacts) <= 1201, "contacts 需要 2–1201 行。")
    for row in contacts:
        require(isinstance(row, list) and len(row) == 4, "每行 contacts 必须有四个值。")
        for value in row:
            number(value, "接触权重", 0, 1)
    return config


class NativeTool:
    def __init__(self, executable=None):
        self.executable = Path(executable) if executable is not None else TOOL_DIRECTORY / "bin/FreeClimbAuthoring.exe"

    def call(self, command, path):
        require(self.executable.is_file(), f"缺少原生校验组件：{self.executable}。请重新解压完整工具；不能跳过校验导出。")
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            result = subprocess.run([str(self.executable.resolve()), command, str(Path(path).resolve())],
                                    shell=False, capture_output=True, encoding="utf-8", errors="strict",
                                    timeout=120, creationflags=flags)
        except (OSError, subprocess.TimeoutExpired, UnicodeError) as error:
            raise AuthoringError(f"原生校验组件运行失败：{error}") from error
        try:
            report = parse_json(result.stdout, "原生校验结果")
        except AuthoringError as error:
            raise AuthoringError(f"校验组件未返回有效 JSON：{result.stderr[:1200] or str(error)}") from error
        if result.returncode != 0 or report.get("ok") is not True:
            details = []
            slots = report.get("slots", [])
            if isinstance(slots, list):
                for entry in slots[:256]:
                    if isinstance(entry, dict) and entry.get("status") in ("rejected", "missing"):
                        name = str(entry.get("slot", "未知槽"))[:64]
                        file = str(entry.get("file", ""))[:240]
                        reason = str(entry.get("reason") or entry["status"])[:600]
                        details.append(f"{name} [{file}]：{reason}")
            summary = str(report.get("error") or result.stderr or "请检查动作文件和配置。")[:1200]
            message = "原生校验未通过：" + summary
            if details:
                message += "\n\n" + "\n".join(details[:5])
                if len(details) > 5:
                    message += f"\n另有 {len(details) - 5} 个槽失败。"
            raise AuthoringError(message)
        return report

    def inspect(self, path):
        report = self.call("inspect", path)
        duration = number(report.get("duration"), "HKX 时长", 0.000001, 10)
        require(type(report.get("frames")) is int and 2 <= report["frames"] <= 1201, "HKX 帧数不受支持。")
        require(type(report.get("tracks")) is int and 1 <= report["tracks"] <= 256, "HKX 轨道数不受支持。")
        return {"ok": True, "duration": duration, "frames": report["frames"], "tracks": report["tracks"]}

    def validate(self, path):
        report = self.call("validate", path)
        require(report.get("loaded") == len(ACTIVE_SLOTS), f"完整基础包未通过全部 {len(ACTIVE_SLOTS)} 槽校验。")
        return report


def inspect_hkx(path, validator=None):
    target = Path(path).resolve()
    require(target.is_file() and target.suffix.lower() == ".hkx", "请选择已适配的 HKX 文件。")
    require(0 < target.stat().st_size <= MAX_HKX_BYTES, "HKX 为空或超过 64 MiB。")
    return (validator or NativeTool()).inspect(target)


def export_override(pack_path, slot, hkx_path, output_path, options=None, validator=None, work_root=None):
    pack = load_template(pack_path, slot)
    native = validator or NativeTool()
    hkx = Path(hkx_path).resolve()
    output = Path(output_path).resolve()
    require(output.suffix.lower() == ".zip", "导出文件必须使用 .zip 扩展名。")
    require(output != hkx and output not in pack.files.values(), "不能用导出文件覆盖输入文件。")
    require(not output.is_relative_to(pack.root), "导出 ZIP 必须放在基础动作包目录之外，以免修改输入包。")
    require(output.parent.is_dir(), "导出目录不存在。")
    information = inspect_hkx(hkx, native)
    template = pack.configurations[slot]
    config = configure_clip(template, information["duration"], options or EditOptions(), information["frames"])
    config_bytes = (json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    require(len(config_bytes) <= 256 * 1024, "生成的动作配置超过 256 KiB。")
    work = Path(work_root).resolve() if work_root is not None else TOOL_DIRECTORY / "work"
    require(not work.is_relative_to(pack.root), "临时工作目录不能位于基础包内。")
    work.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="candidate-", dir=work) as temporary:
        stage = Path(temporary)
        total = 0
        selected_hkx = relative_file(template["file"])
        selected_config = pack.config_paths[slot]
        for relative, source in pack.files.items():
            if relative == selected_config:
                data = config_bytes
            else:
                source = hkx if relative == selected_hkx else source
                limit = MAX_HKX_BYTES if relative.lower().endswith(".hkx") else 256 * 1024
                data = read_bytes(source, min(limit, MAX_PACK_BYTES - total))
            total += len(data)
            require(total <= MAX_PACK_BYTES, "完整候选包超过 256 MiB。")
            target = stage / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        report = native.validate(stage / "pack.json")
        require(report.get("ok") is True and report.get("loaded") == len(ACTIVE_SLOTS), "完整候选包未通过校验。")
        fd, zip_name = tempfile.mkstemp(prefix=".freeclimb-", suffix=".zip", dir=output.parent)
        os.close(fd)
        try:
            with zipfile.ZipFile(zip_name, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                for relative in (selected_hkx, selected_config):
                    archive.write(stage / relative, PREFIX + relative)
            os.replace(zip_name, output)
        finally:
            if Path(zip_name).exists():
                Path(zip_name).unlink()
    return {"output": str(output), "slot": slot, "duration": information["duration"],
            "files": [PREFIX + selected_hkx, PREFIX + selected_config], "loaded": report["loaded"]}
