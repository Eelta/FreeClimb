# FreeClimb

English | [简体中文](README.zh-CN.md)

FreeClimb adds climbing, wall running, sideways leap catches and automatic mantling to The Elder Scrolls V: Skyrim.

## Installation

Install [Skyrim Script Extender (SKSE64)](https://skse.silverlock.org/) and [Address Library for SKSE Plugins](https://www.nexusmods.com/skyrimspecialedition/mods/32444) for your exact game version, plus the [Microsoft Visual C++ v14 Redistributable (x64)](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist). Then install and enable `FreeClimb-Nexus.zip` through MO2.

The settings menu optionally uses [SKSE Menu Framework](https://www.nexusmods.com/skyrimspecialedition/mods/120352); its current releases also require [ImGui Icons](https://www.nexusmods.com/skyrimspecialedition/mods/114790). Without the menu, configure FreeClimb through its INI.

## Compatibility

Includes support for these game versions:

| Branch | Game versions |
| --- | --- |
| SE | 1.5.97.0 |
| AE | 1.6.317.0, 1.6.318.0, 1.6.323.0, 1.6.342.0, 1.6.353.0 |
| AE | 1.6.629.0, 1.6.640.0, 1.6.659.0, 1.6.1130.0, 1.6.1170.0, 1.6.1179.0 |

Versions outside the loading whitelist are rejected. VR, Game Pass and Epic releases are not supported.

## Controls

| Default input | Action |
| --- | --- |
| **Space+A+W+D** together | Start climbing near a valid wall |
| **WASD** | Move along the wall |
| Hold **Shift** while climbing | Switch to wall running |
| **Movement key+Space** while climbing | Perform a manual leap catch |
| **S+Space** | Push away from the wall |
| **A+S+D+Space** | Let go in place, without an outward push |
| Release movement keys | Rest against the wall |

Climbing upward toward a suitable platform can trigger an automatic mantle. Contextual sideways leaps may occur when the direction, handholds and route allow them; the basic `hopLeft/hopRight/hopUp` actions use manual input by default. Manual leap catches are disabled while wall running. Climbing and wall running can automatically bypass overhead or side obstacles along verified routes. A mantle requires a standable platform and always uses `contextMantle`. Downward input never causes wall running.

When stamina consumption is enabled, wall running costs **twice** as much stamina per second as climbing.

### Rebinding controls

The Keys page provides Forward, Backward, Left, Right, Wall entry, Wall run modifier and Hop bindings. Each accepts one key or a combination of up to four keys, such as `Ctrl+E`.

Wall entry uses its complete configured binding; no additional Forward key is required. Entry always starts climbing. Press the wall run modifier after grabbing to switch to wall running; if it was already held during entry, release and press it again. Backward + Hop pushes away from the wall, and Left + Backward + Right + Hop lets go in place.

Wall entry may share keys with movement, Hop or the wall run modifier. Conflicting movement and action bindings are reported when saving. Rebinding affects FreeClimb only; native game input remains active outside climbing. Release and press the entry combination again before reattaching.

## Settings

The interface defaults to **English** and includes **Simplified Chinese**. Menu languages are loaded from `Data/Interface/Translations/FreeClimb_<language>.txt`.

| Page | Settings |
| --- | --- |
| General | Main toggle, operation notifications, low stamina warning, automatic mantling |
| Movement | Climbing speeds in all four directions, wall run speed, diagonal wall run multiplier |
| Automatic actions | Contextual sideways leaps, automatic wall run obstacle jumps, contextual mantling, attempt intervals, left/right opportunity rates |
| Stamina | Stamina consumption toggle, climbing and resting costs, stamina required to grab |
| Audio | Sound toggle and volume |
| Keys | Keys and key combinations |
| Animations and diagnostics | Safe animation pack reload, file validation results, action statistics, detailed diagnostic toggle |

The bottom buttons are **Save settings** and **Restore default settings**. Both write `Data/SKSE/Plugins/FreeClimb.ini` and apply the settings; movement and binding changes wait until a safe detached state. Restore default settings resets and saves only the controls on the current tab. Other tabs, the interface language and INI-only options are retained; unsaved edits on other tabs remain in the editor. Audio changes preview on subsequent traversal sounds.

Disabling **Consume stamina** removes both FreeClimb's stamina requirement for grabbing a wall and all traversal stamina costs.

A left/right opportunity rate of `1` retains every eligible opportunity in that direction; `0` disables automatic contextual sideways leaps in that direction. Attempt intervals count active climbing time, and edge opportunities may be checked earlier. Actions still depend on handhold and collision checks; the interval is not a fixed animation playback cycle.

## Menu translation

Translation files contain fixed keys and text values. Copy `FreeClimb_english.txt` and name it after a language ID, such as `FreeClimb_french.txt`. Keep the keys, translate only the text after each tab, and set `$FC_LANGUAGE_NAME` to the language's own name. Install the file in `Data/Interface/Translations/`, restart Skyrim, select the language, then choose Save settings. Translation loading errors are recorded in `Data/SKSE/FreeClimb.log`.

FreeClimb reads these files itself. The menu uses ImGui and does not rely on automatic translation discovery through the game's Scaleform system. The framework navigation entry remains **FreeClimb → Settings**. Translation covers FreeClimb menu text; HUD operation notifications and raw technical diagnostics are outside this interface.

## Custom animations

Animations are stored in `Data/meshes/actors/character/animations/FreeClimb/`: `pack.json`, `skeleton.json`, 35 HKX files and 35 animation configuration files.

**Replacing animations and their parameters in existing slots does not require recompiling the DLL.** You can override selected files through MO2 while keeping the complete default pack for the remaining slots. Adding slots or new trigger logic requires plugin changes.

HKX files must be retargeted to the specified 99-bone skeleton and use a supported Skyrim SE 64-bit Havok 2010.2 animation format. FreeClimb still controls character movement, collisions and handholds. Some actions use fixed controller timing, so changing HKX duration does not change every action's speed in the same way.

See the **[Animation DIY guide](docs/ANIMATION-DIY.md)** for replacement steps, the main configuration fields, slot purposes and troubleshooting.

The source archive includes an **[Animation Authoring Helper](tools/authoring/README.md)** (Chinese interface). Run `tools/authoring/Start.cmd` with Python 3.10+ and Tkinter installed. Select a compatible HKX and slot, mark limb support intervals, then validate and export a two-file MO2 override ZIP. The native validator is included; no compiler is needed to use it. Skeleton retargeting remains an animation-editor task.

## Logs and issue reports

Logs are written to **`Data/SKSE/FreeClimb.log`** under the game directory. When running through MO2, newly created logs usually appear at **`Overwrite/SKSE/FreeClimb.log`**.

Detailed diagnostics are disabled by default in `Data/SKSE/Plugins/FreeClimb.ini`:

```ini
[General]
Diagnostics=0
```

With detailed diagnostics disabled, the log still records basic startup information, animation pack loading summaries, errors and warnings, and brief wall-entry and exit reasons. Enabling diagnostics adds action transitions, route failures and observed pose output, which can help investigate getting stuck, incorrect handholds or missing actions.

If the menu is available, enable **Detailed diagnostic log** under **Animations and diagnostics**, then choose Save settings. Without the menu, exit the game, set `Diagnostics` to `1`, then restart. Existing custom INI values are preserved during upgrades.

**The log is overwritten each time the game starts.** To capture a problem, enable detailed diagnostics, reproduce it, then copy the log before starting the game again. Reports should include the exact game version, FreeClimb version, location, inputs, symptoms and any modified animation pack. For visual issues, include a video or screenshots. Crashes also require a crash logger report; the FreeClimb log does not contain a complete crash dump.

On the animation page, Selected counts controller slot selections. Observed output is sampled only while detailed diagnostics are enabled; otherwise it displays Off. Statistics cover the current game-load session, and previously accumulated output observations may remain visible.

## Source and building

Building requires Git, CMake 3.24 or later, the Visual Studio 2022 Desktop development with C++ workload, and the Windows SDK. Add Git and CMake to PATH, then run from the source directory:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1
```

The output is `build-multiruntime/Release/FreeClimb.dll`. `FreeClimb-source.zip` includes CommonLibSSE-NG, spdlog, rapidcsv, DirectXTK, MinHook, nlohmann/json and the menu API header. Pinned versions and file hashes are recorded in `tools/dependencies.json` and `DEPENDENCY-SOURCES.json`. Compiling the DLL does not require the game directory or animation assets.

To rebuild the helper, add `-AuthoringTools` to the build command. Copy `build-multiruntime/Release/FreeClimbAuthoring.exe` to `tools/authoring/bin/FreeClimbAuthoring.exe` before distributing the source bundle.

To run tests that do not depend on animation files:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Tests
```

To include animation and pose tests, fully extract `FreeClimb-Nexus.zip` and provide the animation paths:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Tests -AnimationPack 'D:\Mods\FreeClimb\meshes\actors\character\animations\FreeClimb\pack.json' -HkxDirectory 'D:\Mods\FreeClimb\meshes\actors\character\animations\FreeClimb'
```

`tools/package.py` and `tools/validate.py` generate release packages and validate their fixed file manifests. `tools/deploy.ps1` targets a specific MO2 path; check its installation target before use. `-ValidateOnly` validates the staged package only. The DIY guide describes custom animation loading rules and validation.

## License and credits

This project's own code is licensed under GNU GPL version 3 only (`GPL-3.0-only`). CommonLibSSE-NG retains its GPL-3.0-or-later license and the upstream Modding/Linking Exception. The public menu API uses LGPL-2.1, and nlohmann/json uses MIT.

The default animations use authorized Mixamo/Threepeat content, and the audio comes from Kenney. For third-party code, animation and audio attribution and licensing, see the source archive's [LICENSE](LICENSE), [THIRD-PARTY-NOTICES.txt](THIRD-PARTY-NOTICES.txt), and license documentation: [Mixamo](docs/MIXAMO-LICENSE.md), [Threepeat](docs/THREEPEAT-LICENSE.md) and [audio](docs/AUDIO-LICENSE.md).
