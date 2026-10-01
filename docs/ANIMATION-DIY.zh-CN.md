# FreeClimb 动作替换教程

[English](ANIMATION-DIY.md) | 简体中文

替换现有动作无需重新编译 DLL。FreeClimb 有 **35 个有效动作槽**；每个槽对应一个 HKX 和一份 JSON 配置。可只覆盖要改的动作，其余文件继续使用完整默认包。

返回 [README](../README.md) · [中文说明](../README.zh-CN.md)

## 动作制作助手

完整源码包附带 [动作制作助手](../tools/authoring/README.zh-CN.md)。安装带 Tkinter 的 Python 3.10 以上版本后，双击 `tools/authoring/Start.cmd`：

1. 选择默认完整动作包的 `pack.json`、要替换的槽，以及已经适配骨架的 HKX。
2. 保留配套接触配置，或填写“哪只手脚、开始支撑时间、结束时间、过渡时间”。工具自动生成接触曲线。
3. 校验并导出 ZIP，在 MO2 中安装到 FreeClimb 之后覆盖，再脱墙重载、实测。

不必编译工具或手填JSON。骨架适配仍在动画软件中完成；侧跃和登顶的专用路径、抓放窗口可在高级页调整。下面是手动替换和参数说明，按需查阅即可。

## 替换成品，还是制作新动作？

- **已经适配 FreeClimb 的动作**：通常直接覆盖 HKX 和作者提供的配套 JSON，再重载检查。
- **新的 FBX、动捕或其它来源动作**：先在动画软件中重定向到 `skeleton.json` 对应的目标骨架，检查骨名、顺序、局部轴和骨长，再导出受支持的 HKX。还要校准手脚接触和动作参数。不是任意 Skyrim HKX 改名就可使用，也没有一键导入原始 FBX 的功能。

支持的是 Skyrim SE 64 位 Havok 2010.2 动画文件：无压缩 interleaved，或加载器支持的 spline 压缩布局。行为文件、其它骨架或其它 Havok 格式不能直接替代。

## 用 `up` 举例：替换向上攀爬

游戏内动作目录是：

```text
Data/meshes/actors/character/animations/FreeClimb/
  pack.json
  skeleton.json
  up.hkx
  configs/up.json
  ...其余 34 组 HKX 和动作配置
```

1. **放入已经适配的 HKX。** 最简单的方式是覆盖 `up.hkx`，并放入与它配套的 `configs/up.json`。
2. **核对必要参数。** 保持 `slot` 为 `up`；按下表检查步幅和接触曲线。通常不用改 `pack.json` 或 `skeleton.json`，也不需要手填骨架。所有 `file` 路径都相对于动作目录，不是相对于 `configs/`。
3. **脱离墙面后重载并实测。** 在菜单的动作页点击“安全重载动作包 / Safely reload animation pack”，确认整包加载成功，再测试上墙、向上爬、停止和登顶。攀爬中发出的重载请求会等待安全脱墙；重载失败会保留上次有效包。首次加载若缺少必需文件，则不能启用动作包。

只换一个动作也必须保留其余 34 个槽的默认文件。添加新文件名可以，新增槽名或触发方式需要修改插件。

## JSON 主要改什么

从发行包的 `configs/up.json` 或对应动作配置复制，不要从空文件开始。

| 字段 | 作者需要做的事 |
| --- | --- |
| `format`、`version`、`slot` | 保持原值；它们标识配置格式和动作槽 |
| `file` | 指向你的 HKX，例如 `up.hkx` 或 `custom/myUp.hkx` |
| `stride` | 一个循环对应的移动距离，单位是 Skyrim 游戏单位。用于同步手脚节奏，不是角色每秒移动速度 |
| `contacts` | 每行依次是 **左手、右手、左脚、右脚** 的接触权重。`1` 表示请求支撑，`0` 表示松开，中间值用于过渡；应跟随实际抓放手时机 |
| `height`、`travel` | 动作原本的抬升和位移参考，主要供登顶、侧跃等校准。不是攀爬高度上限，也不是直接移动玩家的指令 |

`contacts` 从片段开头到结尾均匀排列，加载时会自动重采样到 HKX 的帧数，所以不必为了帧数相同机械复制行数；**抓放手的时间仍要对应新动作**。把接触全部写成 `1` 容易把应该摆动的手钉在墙上。实际抓点和碰撞检查仍由 FreeClimb 决定。

动作时长来自 HKX，无通用的 JSON `duration` 或 `loop` 开关。普通攀爬循环主要根据移动距离和步幅推进；基础跳跃、上墙和离墙仍有控制器自己的时序，不能仅靠拉长或缩短 HKX 改变所有动作速度。移动速度在菜单或 INI 中调整。

## 侧跃和登顶：要连同配套配置一起改

这些动作与真实抓点、身体路线配合，比普通爬行循环需要更多校准：

- **`contextHang`**：侧跃前后的短准备和接稳动作。首帧的手脚位置用于校准，因此需要和左右侧跃一起检查。
- **`contextHopLeft` / `contextHopRight`**：除了普通字段，还使用 `path`、`sourceHands`、`targetHands` 和 `verticalBlend`。`path` 是身体的路线；另外三项决定两手何时松开、重新抓住，以及何时衔接高度差。时间使用 `0..1` 的归一化相位，要和 HKX 中的实际动作对齐。
- **`contextMantle`**：使用 `unplant`、`replant`、`releaseHands` 和 `replantSamplePhase`，对应第一次撑顶后的卸载、再次撑顶和最终松手。这是唯一的登顶槽。替换时检查攀岩、墙跑和自动越障后的衔接；较低边沿使用其中已松手的后段。

直接查看发行包里对应的 `configs/*.json` 作为完整示例。不要只更换特殊动作的 HKX，却继续沿用不匹配的抓握时间和路径。它们的 HKX 时长参与对应控制段的时长计算，但实际路线仍必须通过支撑和碰撞检查。

## 35 个动作槽对照

| 用途 | 槽名 |
| --- | --- |
| 停驻与四向攀爬 | `hang`、`up`、`down`、`left`、`right` |
| 伸手衔接 | `reach` |
| 手动跃抓 | `hopLeft`、`hopRight`、`hopUp` |
| 离墙、上墙与空中抓墙 | `drop`、`jumpCatch`、`sprintCatch`、`dropBack`、`ledgeCatch` |
| 五向墙跑 | `runUp`、`runLeft`、`runRight`、`runDiagonalLeft`、`runDiagonalRight` |
| 墙跑起步、接入与扶墙 | `runLaunch`、`runCatch`、`runLaunchLeft`、`runLaunchRight`、`sideBrace` |
| 蹬跃与翻转 | `kickUp`、`kickLeft`、`kickRight`、`flipUp`、`flipLeft`、`flipRight` |
| 向外后翻离墙 | `backFlipOut` |
| 情境侧跃准备、侧跃与登顶 | `contextHang`、`contextHopLeft`、`contextHopRight`、`contextMantle` |

## 常见问题

| 问题 | 先检查 |
| --- | --- |
| 换了文件没有变化 | MO2 是否实际覆盖正确路径；是否完成重载；整包是否有其它槽加载失败；当前操作是否选到了这个动作 |
| 骨架或轨道错误 | 目标骨架、骨序、骨名和导出轨道是否完整；不要靠改文件名解决 |
| 手腕扭曲、肘反折 | 重定向的局部轴、参考姿态和左右映射；加载成功不等于姿态自然 |
| 手粘墙、抓空气或脚滑 | 接触列顺序、抓放时机、步幅，以及特殊动作的配套路径和时间窗 |
| 修改时长后速度没变 | 该动作是否仍由固定控制段推进；普通循环的步幅是否匹配 |
| 侧跃或登顶动作没出现 | 输入方向、对应设置、实际抓点和可通行路线；替换文件不会取消这些条件 |

先看动作页的加载结果。需要排查时，在菜单开启详细诊断，重现问题后保存日志；位置和记录方式见 [中文 README](../README.zh-CN.md)。
