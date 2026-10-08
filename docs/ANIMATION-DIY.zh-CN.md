# FreeClimb 动作替换教程

[English](ANIMATION-DIY.md) | 简体中文

替换现有动作无需重新编译 DLL。选择动作，导入或载入基础动作，预览后导出即可。墙跑和情境侧跃各方向都使用自己的完整 HKX 和配套配置，所需过渡包含在对应动作中，不同方向可独立替换。

返回 [README](../README.md) · [中文说明](../README.zh-CN.md)

## 导入、预览、导出

打开源码包内的 **`tools/converter/bin/FreeClimbConverter.exe`**：

1. 选择标准 Skyrim SE 人形 HKX 和要替换的动作，载入预览。
2. 先保留自动适配。播放、拖动时间轴查看；需要时在同一窗口裁剪、微调姿态或调整支撑与路线。
3. 导出 ZIP，安装到 MO2 的 FreeClimb 之后覆盖，离墙后安全重载并进游戏检查。

不需要 Python、编译工具，也不用手写 JSON。基础包自动查找；原文件不会被修改。支持条件、操作和自动适配边界见[动作工具说明](../tools/converter/README.zh-CN.md)。

导入预览显示 HKX 自身的动作和源位移，原地动作不额外套基础路线，完整墙跑按整段播放。基础动作预览保留已有路线适配，无需选择模式。预览不代表真实地图碰撞、蒙皮或服装物理验收。

导入新的完整墙跑时，直接选择目标方向并载入 HKX，标出循环开始和结束。保留裁剪起点 < 循环开始 < 循环结束 < 裁剪终点，工具自动更新整段预览与输出。载入基础动作会带入已有循环范围。侧跃不需要循环标记。见[工具使用步骤](../tools/converter/README.zh-CN.md#怎么用)。

全部槽均可导入外部动作：待机按自身时长循环，攀爬／墙跑循环按移动距离与源步幅推进，起手、跃抓、登顶和离墙等保留完整片段时序。源 AMR 或提取位移只烘焙到 Root 一次，真实行程适配经过碰撞检查的路线；原地动作使用基础参考距离。导入或裁剪时首帧 Root 位置归零，局部姿势不变。工具不把普通地面动作自动变成贴墙动作，也不猜测 COM 中的大幅位移；素材应符合槽位用途，游戏内的碰撞与衔接仍需检查。

工具自动生成 `FreeClimbClip` 版本 2 的 `authoredPlayback` 和适用的 Root 轨迹，作者无需手写 JSON。此类导出和新的墙跑时间轴配置都必须配套本次 0.4.0 DLL，0.3.0 DLL 不支持；基础片段保留版本 1 播放规则。下面的数据字段用于技术查阅，**不是每次替换都要填写的表单**。

外部动作不套用默认包专用的扶墙、起跳／落抓拼接或起手裁剪，也不会被其它默认动作截取作局部参考。整体定位、接触修正与安全限制仍保留。请安装工具生成的 HKX **及 JSON**；只换 HKX 会继续使用旧配置的处理方式。

## 替换成品，还是制作新动作？

- **已经适配 FreeClimb 的动作**：通常直接覆盖 HKX 和作者提供的配套 JSON，再重载检查。
- **标准 Skyrim SE 人形 HKX**：使用上面的转换器，将支持的轨道映射到 FreeClimb 骨架并生成配套配置。
- **新的 FBX、动捕或其它来源动作**：先在动画软件中重定向到标准 Skyrim 人形骨架，导出受支持的 SE HKX，再使用转换器。进游戏检查骨骼局部轴、抓放时机和位移。转换器不负责任意骨架重定向，也不直接导入原始 FBX。

支持的是 Skyrim SE 64 位 Havok 2010.2 动画文件：无压缩 interleaved，或加载器支持的 spline 压缩布局。行为文件、其它骨架或其它 Havok 格式不能直接替代。

## 用 `up` 举例：替换向上攀爬

游戏内动作目录是：

```text
Data/meshes/actors/character/animations/FreeClimb/
  pack.json
  skeleton.json
  up.hkx
  configs/up.json
  runUp.hkx
  configs/runUp.json
  runLeft.hkx
  configs/runLeft.json
  contextHopLeft.hkx
  configs/contextHopLeft.json
  contextHopRight.hkx
  configs/contextHopRight.json
  ...其余动作及 configs 中的配置
```

1. 在动作工具选择 `up`，载入你的 HKX，检查预览并导出。
2. 将 ZIP 安装到 MO2 的 FreeClimb 之后覆盖，保持基础包启用。
3. 脱离墙面，在菜单选择“安全重载动作包”，再测试上墙、向上爬、停止和登顶。失败会保留上次有效包；首次加载仍需要全部必需文件。

保持完整基础包启用。墙跑和情境侧跃各方向都导出自己的 HKX 和 JSON，不同方向的覆盖包可以同时使用。同一动作则由 MO2 后加载的文件覆盖。旧 `wallRun.hkx` 或 `contextHop.hkx` 共享覆盖包需通过新版工具重新导出。新增槽名或触发方式仍需修改插件。

<details>
<summary>配置字段与专用动作时序（技术参考，工具自动写入）</summary>

### 配置字段

从发行包的 `configs/up.json` 或对应动作配置复制，不要从空文件开始。墙跑和情境侧跃使用独立的 `FreeClimbActionGroup` 版本 2 配置，例如 `configs/runLeft.json` 和 `configs/contextHopLeft.json`，只描述此方向。组版本与内部 `FreeClimbClip` 播放版本相互独立。下表描述单个片段字段。

| 字段 | 作者需要做的事 |
| --- | --- |
| `format`、`version`、`slot` | 保持原值；它们标识配置格式和动作槽 |
| `file` | 指向你的 HKX，例如 `up.hkx` 或 `custom/myUp.hkx` |
| `member` | 合并 HKX 内的命名动画。墙跑使用方向名选择完整时间轴，再用 `frameRange` 选片段；单动画文件通常省略 |
| `frameRange` | 完整时间轴内的零起始帧范围 `[first, last]`，两端都包含，至少两帧且不能越界；工具自动填写 |
| `rootShift` | 与 `frameRange` 配套的三分量 Root 偏移。加载片段时从各帧 Root 平移中减去它，恢复该段局部曲线，避免重复计算整条时间轴的累计位移 |
| `stride` | 一个循环对应的移动距离，单位是 Skyrim 游戏单位。用于同步手脚节奏，不是角色每秒移动速度 |
| `contacts` | 每行依次是 **左手、右手、左脚、右脚** 的接触权重。`1` 表示请求支撑，`0` 表示松开，中间值用于过渡；应跟随实际抓放手时机 |
| `height`、`travel` | 动作原本的抬升和位移参考，主要供登顶、侧跃等校准。不是攀爬高度上限，也不是直接移动玩家的指令 |

`contacts` 从片段开头到结尾均匀排列，加载时会自动重采样到 HKX 的帧数，所以不必为了帧数相同机械复制行数；**抓放手的时间仍要对应新动作**。把接触全部写成 `1` 容易把应该摆动的手钉在墙上。实际抓点和碰撞检查仍由 FreeClimb 决定。

动作时长来自 HKX；若使用 `frameRange`，由该范围的帧间距计算片段时长，而非播放整个 member。无通用的 JSON `duration` 或 `loop` 开关。移动循环根据实际移动距离和步幅推进；外部导入的单次动作按完整片段时长播放。基础版本 1 保留原控制器时序与短过渡规则。角色移动速度在菜单或 INI 中调整。

### 独立墙跑时间轴

`runUp.hkx`、`runLeft.hkx`、`runRight.hkx`、`runDiagonalLeft.hkx`、`runDiagonalRight.hkx` 各包含自己的完整命名动画。侧向动作所需的扶墙参考也保存在自己的 HKX 中，不需要共享 `sideBrace.hkx`。

以 `configs/runLeft.json` 为例：

- 外层是 `format: "FreeClimbActionGroup"`、`version: 2`、`group: "wallRun"`、`direction: "runLeft"`。
- `clips` 描述 `runLeft` 时间轴的起步、循环和收尾，并通过内部 `sideBrace` 槽登记该文件自己的扶墙参考。
- `sequences` 仅有一项，`slot` 为 `runLeft`；`launch` 和 `catch` 对应起步和收尾，`brace` 指向同一文件内的 `runLeftBrace`。其它侧向动作各自保留自己的参考。
- `frameRange` 选择每段的闭区间。相邻范围可共享边界帧；`rootShift` 仅恢复该段局部 Root 位移。完整文件限于 10 秒、1201 帧。
- 手脚接触、源时序和轨迹均按对应片段计算。工具负责换算，作者只需确定整段动作及循环范围。

加载器保留对旧组合格式的读取能力；当前默认文件和导出均使用独立方向。安装不同方向的覆盖包，不需要修改 `pack.json`。

`contextHopLeft.hkx` 和 `contextHopRight.hkx` 同样各自包含一整条准备、跃抓、收尾时间轴。配套 JSON 使用 `group: "contextHop"`、自己的 `direction`，以及含 `prepare`、`catch` 范围的 `sequences`。内部 `contextHang` 阶段只属于当前方向。导入的完整侧跃按源动作播放，不额外插入共享准备或落抓动作。

部分基础片段带有可选的 `references`，用于 `launchApproach`、`kickTakeoff`、`kickLanding`、`kickRunLanding`、`kickRunBrace`。每项指向当前 HKX 内的私有命名动画，不读取其它动作文件。工具自动维护这些参考；作者无需手写，外部导入也不会套用基础片段的参考动作。

### 侧跃和登顶的配套配置

以下是基础版本 1 的专用字段。外部导入版本 2 由工具生成源时序、接触和路线，不需要手工填写这些基础时间窗：

- **`contextHang`**：各完整侧跃内部的准备和接稳阶段。手脚位置只标定当前方向，作者直接替换完整左侧或右侧动作。
- **`contextHopLeft` / `contextHopRight`**：除了普通字段，还使用 `path`、`sourceHands`、`targetHands` 和 `verticalBlend`。`path` 是身体的路线；另外三项决定两手何时松开、重新抓住，以及何时衔接高度差。时间使用 `0..1` 的归一化相位，要和 HKX 中的实际动作对齐。
- **`contextMantle`**：基础版本 1 配置使用 `unplant`、`replant`、`releaseHands` 和 `replantSamplePhase`，对应第一次撑顶后的卸载、再次撑顶和最终松手。这是唯一的登顶槽。替换时检查攀岩、墙跑和自动越障后的衔接；较低边沿使用其中已松手的后段。

直接查看发行包里对应的 `configs/*.json` 作为完整示例。不要只更换特殊动作的 HKX，却继续沿用不匹配的抓握时间和路径。它们的 HKX 时长参与对应控制段的时长计算，但实际路线仍必须通过支撑和碰撞检查。

## 动作与内部阶段对照

| 槽名 | 什么时候使用 |
| --- | --- |
| `hang`、`up`、`down`、`left`、`right` | 墙上停驻及四向攀爬。 |
| `reach` | 按现有抓墙组合，仍在地面且目标较近时伸手抓墙。 |
| `jumpCatch`、`ledgeCatch` | 跳起／较远的抓墙起手，以及下落时重新抓墙。 |
| `hopUp`、`hopLeft`、`hopRight` | 攀岩时方向＋跳跃；也用于有实际落点的受阻恢复。 |
| `runUp`、`runLeft`、`runRight`、`runDiagonalLeft`、`runDiagonalRight` | 上墙后按住墙跑键，配合对应方向。 |
| `runLaunch`、`runLaunchLeft`、`runLaunchRight`、`runCatch` | 攀岩与墙跑之间的过渡。默认包作为姿态混合片段使用，不一定完整单独播放；外部版本 2 替换使用完整片段。 |
| `sideBrace` | 保存在各侧向墙跑文件内的扶墙手臂参考，不再单独编辑或安装。 |
| `kickUp`、`kickLeft`、`kickRight` | 墙跑时遇到可跨越障碍，自动选择对应方向蹬跃；要求落点、整条路线和翻身空间通过检查。墙跑手动跳跃仍关闭。 |
| `drop` | 原地松手。 |
| `backFlipOut`、`dropBack` | 后退＋跳跃向外离墙；允许花式且空间足够时后翻，否则普通后蹬。 |
| `contextHang` | 当前方向侧跃自己的准备与落抓阶段，不是独立长期悬挂。 |
| `contextHopLeft`、`contextHopRight` | 自动攀爬动作或手动侧跃找到合适真实支撑与路线时使用。 |
| `contextMantle` | 找到可站立顶部后登顶；所有登顶共用此槽。 |

按键组合不因动作变体而变化。工具按完整动作编辑，内部过渡和参考槽不需要另外制作替换包。碰撞、耐力与开关仍决定动作是否可用。详细日志中的 `Session action` 会列出实际选择的槽名，但不会把局部扶墙参考计作独立动作。

## 常见问题

| 问题 | 先检查 |
| --- | --- |
| 换了文件没有变化 | MO2 是否实际覆盖正确路径；是否完成重载；整包是否有其它槽加载失败；当前操作是否选到了这个动作 |
| 骨架或轨道错误 | 目标骨架、骨序、骨名和导出轨道是否完整；不要靠改文件名解决 |
| 手腕扭曲、肘反折 | 重定向的局部轴、参考姿态和左右映射；加载成功不等于姿态自然 |
| 手粘墙、抓空气或脚滑 | 接触列顺序、抓放时机、步幅，以及特殊动作的配套路径和时间窗 |
| 修改时长后速度没变 | 移动循环按距离和步幅推进；单次动作确认已导出版本 2 并重载 |
| 侧跃或登顶动作没出现 | 输入方向、对应设置、实际抓点和可通行路线；替换文件不会取消这些条件 |

先看动作页的加载结果。需要排查时，在菜单开启详细诊断，重现问题后保存日志；位置和记录方式见 [中文 README](../README.zh-CN.md)。

</details>
