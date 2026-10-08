# FreeClimb Animation Authoring Assistant

English | [简体中文](README.zh-CN.md)

For importing, previewing and adjusting an HKX in one window, use the [Animation Tool](../converter/README.md). This optional configuration helper is not required by that workflow.

Create a single-animation override package from an HKX already adapted for FreeClimb. Keep the existing contact curve, or generate one by specifying which hand or foot provides support, when support begins, and when it releases.

## Getting started

Requires Windows x64, Python 3.10 or newer, and Python's Tkinter component (select Tcl/Tk when installing Python). The native validation component also requires the Microsoft Visual C++ 2015–2022 x64 runtime. No third-party Python packages or Visual Studio installation are needed.

Extract the complete source package and double-click `Start.cmd` in this directory. The launcher tries `py -3` first, then `python`. The native validation component must be at `bin/FreeClimbAuthoring.exe`. Restore it if it is missing; exporting requires validation to succeed.

## Creating an override package

1. Select `pack.json` from the complete default animation pack, then select the slot to replace, such as `up`.
2. Select an adapted HKX. Inspection shows its duration, frame count, and track count. This only confirms that the file can be read; the complete pack will also be validated later.
3. Keep the template contacts, or add support intervals for individual limbs. Leave the optional `stride`, `height`, and `travel X/Y/Z` fields empty to retain their original values.
4. Choose a ZIP location outside the base pack directory, then click **校验并导出 MO2 覆盖包** (Validate and export the override package). The ZIP is written only after validation succeeds; a failed operation does not replace an existing ZIP.

The ZIP contains an HKX and its action JSON, under `meshes/actors/character/animations/FreeClimb/`. Wall runs and contextual side leaps export only the selected direction, including its transitions and required references. Left and right packages can be installed together without replacing each other. Re-export old shared `contextHop` overrides with the updated Animation Tool. Install after the complete FreeClimb mod in MO2 and keep the base pack enabled. After leaving the wall, safely reload and test entry, movement, stopping, transitions and exit.

## Marking contacts

Times are in seconds and must fall within the new HKX's duration. Outside an interval, contact weight is `0`, meaning released. Inside it, the weight can reach `1`, requesting support.

For example, if the left hand starts contact at `0.20` seconds and releases at `0.90` seconds, with a `0.05`-second transition, it fades in over `0.20–0.25`, maintains support over `0.25–0.85`, and fades out over `0.85–0.90`. The transition must not exceed half the interval length. A value of `0` switches immediately, which is usually less natural than a short transition. Overlapping intervals for the same limb use the highest weight; unmarked limbs remain at `0`.

The assistant generates four columns at the new HKX's actual decoded frame count, up to 1201 rows. Each interval must contain at least one positive-weight sample. If the source has too few frames, an interval is too narrow, or its transitions leave no effective sample, export is rejected. Increase source sampling or adjust the markers. The assistant does not automatically detect when hands or feet touch the wall; authors must mark these times from the animation.

## Special animations and advanced configuration

Each contextual side leap owns its internal `contextHang` preparation and ending; their hand and foot positions calibrate that direction only. For base clips, the `path`, `sourceHands`, `targetHands`, and `verticalBlend` fields in `contextHopLeft/Right`, and the `unplant`, `replant`, `releaseHands`, and `replantSamplePhase` fields in `contextMantle`, must match the animation. The Animation Tool generates the configuration for imported complete actions without requiring separate helper replacements.

These fields are preserved in full by default and shown on the **高级配置与特殊动作** (Advanced configuration and special animations) tab. Enable advanced JSON when you need to edit them; `slot` and `file` cannot be changed. Basic parameter inputs override their corresponding JSON fields, and manual contact mode overrides `contacts`. All advanced changes must also pass complete native validation. Changing limb contact markers alone does not automatically update dedicated grip/release windows or paths.

## Scope and file safety

- The assistant does not automatically turn arbitrary FBX files, animations from other games, or arbitrary Skyrim HKX files into FreeClimb animations. Retargeting, local axes, and animation quality still need to be handled in animation software.
- It supports the Skyrim SE 64-bit Havok 2010.2 interleaved format and certain spline layouts accepted by the native validation component. It does not add animation slots or triggers, or change character movement and collision rules.
- Successful validation does not guarantee perfect visual results. Check wrists, elbows, hand and foot contacts, and transitions in game.

For more animation parameters, see the [Animation replacement guide](../../docs/ANIMATION-DIY.md).
