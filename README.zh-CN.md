# FreeClimb

[English](README.md) | 简体中文

FreeClimb 为《上古卷轴5：天际》提供攀岩、墙跑、侧向跃抓和自动登顶功能。

## 安装

安装与完整游戏版本匹配的 [Skyrim Script Extender (SKSE64)](https://skse.silverlock.org/) 和 [Address Library for SKSE Plugins](https://www.nexusmods.com/skyrimspecialedition/mods/32444)，以及 [Microsoft Visual C++ v14 Redistributable（x64）](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist) 系统运行库，然后通过 MO2 安装并启用 `FreeClimb-Nexus.zip`。

设置菜单可选安装 [SKSE Menu Framework](https://www.nexusmods.com/skyrimspecialedition/mods/120352)；其当前版本还要求安装 [ImGui Icons](https://www.nexusmods.com/skyrimspecialedition/mods/114790)。未安装菜单时，可通过 INI 配置 FreeClimb。

## 兼容性

包含以下游戏版本的适配：

| 分支 | 游戏版本 |
| --- | --- |
| SE | 1.5.97.0 |
| AE | 1.6.317.0、1.6.318.0、1.6.323.0、1.6.342.0、1.6.353.0 |
| AE | 1.6.629.0、1.6.640.0、1.6.659.0、1.6.1130.0、1.6.1170.0、1.6.1179.0 |

未列入加载白名单的版本会被拒绝；VR、Game Pass 和 Epic 发行渠道不在支持范围内。

## 操作

| 默认按键 | 功能 |
| --- | --- |
| 同时按 **Space+A+W+D** | 靠近有效墙面后进入攀岩 |
| **WASD** | 沿墙移动 |
| 攀岩中按住 **Shift** | 切换墙跑 |
| 攀岩中 **方向键+Space** | 手动跃抓 |
| **S+Space** | 向外离墙 |
| **A+S+D+Space** | 原地松手，不施加向外推力 |
| 松开移动键 | 贴墙停驻 |

向上到达合适平台时可自动登顶。情境侧跃会在方向、抓点和通路满足条件时自动穿插；基础 `hopLeft/hopRight/hopUp` 默认由手动输入触发。墙跑期间禁用手动跃抓。攀岩或墙跑遇到头顶、侧向障碍时，可沿验证通路自动跃抓。只有确认可站立平台才登顶，统一使用 `contextMantle`。向下输入不执行墙跑。

启用耐力消耗时，墙跑每秒耗耐力为攀岩的 **2 倍**。

### 按键重绑

按键页提供前进、后退、左、右、上墙攀爬、墙跑修饰键和跃抓键。每项支持单键或最多四键组合，例如 `Ctrl+E`。

上墙攀爬使用独立的完整绑定，不需要额外按前进键。上墙后先进入攀岩，再按墙跑修饰键切换墙跑；若上墙时已经按住墙跑修饰键，先松开再按即可。“后退 + 跃抓键”向外蹬离，“左 + 后退 + 右 + 跃抓键”原地松手。

入墙绑定可以包含移动、跃抓或墙跑修饰键；移动与动作绑定之间的冲突会在保存时提示。重绑仅作用于 FreeClimb，未攀爬时使用游戏原生输入。离墙后需松开并重新按下入墙组合才能再次抓墙。

## 设置

界面默认 **English**，附带 **简体中文**。菜单语言从 `Data/Interface/Translations/FreeClimb_<language>.txt` 读取。

| 页面 | 设置内容 |
| --- | --- |
| General / 基础 | 总开关、操作提示、低耐力提醒、自动登顶 |
| Movement / 移动 | 四向攀爬速度、墙跑速度、斜向墙跑倍率 |
| Automatic actions / 自动动作 | 情境侧跃、墙跑自动越障、情境登顶、尝试间隔、左右机会保留率 |
| Stamina / 耐力 | 耐力消耗开关、攀爬与停驻消耗、开始抓墙的耐力门槛 |
| Audio / 音效 | 音效开关与音量 |
| Keys / 按键 | 按键与组合键 |
| Animations and diagnostics / 动作管理与诊断 | 安全重载动作包、文件验证结果、动作统计、详细诊断开关 |

底部仅有“**保存设置**”和“**还原默认设置**”两个按钮，点击后都会写入 `Data/SKSE/Plugins/FreeClimb.ini` 并应用。移动与按键修改等待安全离墙后生效；还原默认设置只恢复并保存当前选项卡上的设置，其它选项卡、界面语言和仅在 INI 中提供的参数保持不变；其它页面未保存的编辑仍保留在编辑区。音效调整会在下次播放攀爬音效时预览。

关闭 **Consume stamina / 消耗耐力** 会同时取消 FreeClimb 的入墙耐力门槛与全部动作扣费。

左右机会保留率 `1` 保留该方向全部合格机会，`0` 禁用该方向的自动情境侧跃。尝试间隔按有效攀爬时间计算；边缘机会也可能提前检查。实际动作仍取决于抓点与碰撞检查，间隔不代表固定播放周期。

## 菜单翻译

翻译文件使用固定键和文本值：复制 `FreeClimb_english.txt`，按语言 ID 命名，例如 `FreeClimb_french.txt`；保留键名，只翻译制表符后的文字，并将 `$FC_LANGUAGE_NAME` 设为语言自身名称。把文件安装到 `Data/Interface/Translations/`，重启游戏后选择该语言，再点击“保存设置”。翻译加载错误记录在 `Data/SKSE/FreeClimb.log` 中。

文件由 FreeClimb 自行读取，菜单使用 ImGui，不依赖游戏 Scaleform 自动发现翻译。框架导航入口保留 **FreeClimb → Settings**。翻译范围为 FreeClimb 菜单文字；HUD 操作提示与原始技术诊断不在接口范围内。

## 动作自定义

动作目录为 `Data/meshes/actors/character/animations/FreeClimb/`，由 `pack.json`、`skeleton.json`、35 个 HKX 和 35 个动作配置组成。

**替换现有槽的动画与配套参数无需重新编译 DLL。** 可用 MO2 覆盖部分文件，保留完整默认包提供其余槽。新增动作槽或新的触发逻辑需要修改插件。

HKX 必须重定向至规定的 99 骨架，并符合受支持的 Skyrim SE 64 位 Havok 2010.2 动画格式。人物移动、碰撞和抓点仍由 FreeClimb 控制；部分动作采用控制器固定时序，修改 HKX 时长不会统一改变所有动作速度。

替换步骤、主要参数、动作槽用途和常见问题见 **[动作 DIY 教程](docs/ANIMATION-DIY.zh-CN.md)**。

源码包附带 **[动作制作助手](tools/authoring/README.zh-CN.md)**。安装带 Tkinter 的 Python 3.10 以上版本后，运行 `tools/authoring/Start.cmd`。选择兼容 HKX 和动作槽，标记手脚支撑时段，即可校验并导出仅含两个文件的 MO2 覆盖包。原生校验组件已随包提供，使用时不用编译；骨架重定向仍在动画软件中完成。

## 日志与问题反馈

日志写入游戏目录下的 **`Data/SKSE/FreeClimb.log`**。通过 MO2 运行时，新建日志通常位于 **`Overwrite/SKSE/FreeClimb.log`**。

详细诊断默认关闭，对应 `Data/SKSE/Plugins/FreeClimb.ini`：

```ini
[General]
Diagnostics=0
```

关闭详细诊断后，基础启动信息、动作包加载摘要、错误/警告以及简短的入墙和结束原因仍会记录。开启后增加动作切换、路线失败、姿态输出观测等信息，适合排查卡住、抓点异常或动作未出现的问题。

有菜单时，在 **Animations and diagnostics / 动作管理与诊断** 中打开 **Detailed diagnostic log / 详细诊断日志**，然后点击“保存设置”。无菜单时，退出游戏，将 `Diagnostics` 改为 `1` 后再启动。自定义 INI 的既有取值会在升级时保留。

**每次启动游戏会覆盖该日志。** 收集问题记录时，先开启详细诊断，再复现问题，并在下次启动前复制日志保存。反馈请附完整游戏版本、FreeClimb 版本、地点、按键、现象及修改过的动作包；画面问题可附视频或截图。崩溃问题还需提供崩溃记录器生成的报告，FreeClimb 日志不包含完整崩溃转储。

动作页的“选择次数”统计控制器选槽；“实际输出次数”仅在开启详细诊断时采样，关闭时显示“未采样”。统计以本次读档后的会话为范围，已累计的输出观测可能保留。

## 源码与编译

编译需要 Git、CMake 3.24 以上、Visual Studio 2022 C++ 桌面开发工具和 Windows SDK。将 Git、CMake 加入 PATH 后，在源码目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1
```

输出为 `build-multiruntime/Release/FreeClimb.dll`。`FreeClimb-source.zip` 源码包包含 CommonLibSSE-NG、spdlog、rapidcsv、DirectXTK、MinHook、nlohmann/json 和菜单 API 头；固定版本与文件校验分别见 `tools/dependencies.json` 和 `DEPENDENCY-SOURCES.json`。DLL 编译不需要游戏目录或动作素材。

需要重新编译制作助手时，在构建命令后加 `-AuthoringTools`，再将 `build-multiruntime/Release/FreeClimbAuthoring.exe` 复制到 `tools/authoring/bin/FreeClimbAuthoring.exe`，与源码包一起分发。

运行不依赖动作文件的测试：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Tests
```

运行包含动作与姿态的测试时，先完整解压 `FreeClimb-Nexus.zip`，再提供动作路径：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Tests -AnimationPack 'D:\Mods\FreeClimb\meshes\actors\character\animations\FreeClimb\pack.json' -HkxDirectory 'D:\Mods\FreeClimb\meshes\actors\character\animations\FreeClimb'
```

`tools/package.py` 与 `tools/validate.py` 用于正式发行包的生成和固定清单验证。`tools/deploy.ps1` 是面向指定 MO2 路径的部署工具，使用前须确认其安装目标；`-ValidateOnly` 只验证暂存包。DIY 动作的加载规则与验证方法见教程。

## 许可与致谢

本项目自有代码采用 GNU GPL 第 3 版许可（`GPL-3.0-only`）。CommonLibSSE-NG 保留其 GPL-3.0-or-later 许可及上游的 Modding/Linking Exception；菜单公开 API 采用 LGPL-2.1，nlohmann/json 采用 MIT。

默认动画使用获授权的 Mixamo/Threepeat 内容，音效来自 Kenney。第三方代码、动画和音效的归属与许可见源码包中的 [LICENSE](LICENSE)、[THIRD-PARTY-NOTICES.txt](THIRD-PARTY-NOTICES.txt) 及授权说明：[Mixamo](docs/MIXAMO-LICENSE.md)、[Threepeat](docs/THREEPEAT-LICENSE.md)、[音效](docs/AUDIO-LICENSE.md)。
