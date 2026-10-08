# FreeClimb Animation Replacement Guide

English | [简体中文](ANIMATION-DIY.zh-CN.md)

Replacing animations does not require recompiling the DLL. Select an action, import an HKX or load its base animation, preview and export. Each wall-run direction and each contextual side leap has its own complete HKX and configuration. Required transitions stay with that action; directions can be replaced independently.

Back to [README](../README.md) · [Chinese README](../README.zh-CN.md)

## Import, preview and export

Open **`tools/converter/bin/FreeClimbConverter.exe`** in the source archive:

1. Choose a standard Skyrim SE humanoid HKX and the action to replace, then load the preview.
2. Start with automatic adaptation. Play and scrub to inspect it; trim, correct the pose or adjust support and routes in the same window only where needed.
3. Export a ZIP, install it after FreeClimb in MO2, detach, safely reload and check it in-game.

No Python, compiler or hand-written JSON is needed. The base pack is found automatically and original files stay untouched. See the [Animation Tool guide](../tools/converter/README.md) for input requirements, controls and adaptation limits.

Imported previews show the HKX's own animation and source movement; in-place clips receive no added base route, and complete wall runs play through the whole file. Base actions retain their existing route adaptation without mode selection. Preview does not validate real map collision, skins or clothing physics.

To import a complete wall run, select its direction and load the HKX, then mark its loop start and end. Keep trim start < loop start < loop end < trim end; the tool updates the complete preview and output automatically. Base actions load their existing loop range. Side leaps need no loop markers. See the [tool workflow](../tools/converter/README.md#quick-start).

All slots accept external animations: idles loop on their own clock, climb/run cycles follow movement distance and source stride, and entries, grabs, mantles and exits keep complete clip timing. Source AMR or extracted movement is baked into Root once. Actual travel is fitted to a collision-checked route; in-place clips use base reference distances. Importing or trimming resets the first Root position to zero without changing local poses. The tool does not turn ground actions into wall actions or guess large COM displacement. Choose suitable source content and check collision and transitions in-game.

The tool automatically writes `FreeClimbClip` version 2 with `authoredPlayback` and applicable Root trajectories; authors do not need to write JSON. These exports and the new wall-run timeline configurations require the matching 0.4.0 DLL; the 0.3.0 DLL cannot use them. Base clips retain version 1 playback rules. The fields below are a technical reference, **not a form you must complete for every replacement**.

External actions do not inherit base-specific bracing, takeoff/landing splices or entry trimming, and are not sampled as partial references for other base actions. Overall placement, contact adjustment and safety limits still apply. Install the generated HKX **and JSON** together; replacing only the HKX keeps the old configuration's processing.

## Replacing a ready-made animation or creating a new one?

- **Animations already adapted to FreeClimb:** usually replace the HKX and its accompanying JSON, then reload and check the result.
- **Standard Skyrim SE humanoid HKX:** use the converter above to map supported tracks to FreeClimb's rig and create the paired configuration.
- **New FBX files, motion capture or other source animations:** retarget to the standard Skyrim humanoid skeleton in animation software, export a supported SE HKX, then use the converter. Check local bone axes, grip timing and movement in-game. The converter does not retarget arbitrary rigs or import raw FBX.

Supported files are Skyrim SE 64-bit Havok 2010.2 animations: uncompressed interleaved animations or spline-compressed layouts supported by the loader. Behavior files, other skeletons and other Havok formats are not direct replacements.

## Example: replacing upward climbing in the `up` slot

The in-game animation directory is:

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
  ...remaining animations and files under configs
```

1. Select `up` in the Animation Tool, load your HKX, inspect the preview and export.
2. Install the ZIP after FreeClimb in MO2 and keep the base pack enabled.
3. Detach, choose **Safely reload animation pack** in the menu, then test entry, upward climbing, stopping and mantling. Failed reloads preserve the previous valid pack; initial loading still requires all necessary files.

Keep the complete base pack enabled. Each wall-run and contextual-leap direction exports its own HKX and JSON; packages for different directions can be used together. MO2 decides which replacement wins for the same action. Re-export old shared `wallRun.hkx` or `contextHop.hkx` overrides with the updated tool. Adding slot names or triggers still requires plugin changes.

<details>
<summary>Configuration fields and special-action timing (technical reference; written by the tool)</summary>

### Configuration fields

Start with `configs/up.json` or the corresponding release configuration. Wall runs and contextual leaps use independent `FreeClimbActionGroup` version 2 configurations, such as `configs/runLeft.json` and `configs/contextHopLeft.json`, describing that direction only. Group format and contained `FreeClimbClip` playback versions are independent. The table describes individual clip fields.

| Field | What to do |
| --- | --- |
| `format`, `version`, `slot` | Keep the original values; they identify the configuration format and animation slot. |
| `file` | Point to your HKX, such as `up.hkx` or `custom/myUp.hkx`. |
| `member` | Named animation in a grouped HKX. Wall runs select a complete direction timeline, then use `frameRange` to select a section. Usually omitted for single-animation files. |
| `frameRange` | Zero-based `[first, last]` frame indices in the complete timeline, inclusive at both ends. Must contain at least two frames and stay within the animation. Written automatically. |
| `rootShift` | Three-component Root offset paired with `frameRange`. Loading subtracts it from each selected frame's Root translation to restore the local section curve instead of counting the timeline's accumulated movement again. |
| `stride` | Distance covered by one cycle, in Skyrim game units. It synchronizes limb motion with movement; it is not movement speed per second. |
| `contacts` | Each row contains contact weights for the **left hand, right hand, left foot and right foot**, in that order. `1` requests support, `0` releases it, and intermediate values blend between them. Match the actual grip and release timing. |
| `height`, `travel` | Reference lift and displacement from the animation, mainly used to calibrate mantles and sideways leaps. These are not climbing height limits or commands that directly move the player. |

The `contacts` rows are evenly spaced from the beginning to the end of the clip. The loader resamples them to the HKX frame count, so there is no need to duplicate rows merely to match that count. **Grip and release timing must still match the new animation.** Setting every contact to `1` can pin a hand to the wall when it should be swinging. FreeClimb still determines actual handholds and performs collision checks.

Animation duration comes from the HKX. With `frameRange`, the selected range's frame spacing determines the section duration rather than playing the whole member. There is no general JSON `duration` or `loop` switch. Moving cycles advance according to actual movement distance and stride; imported single actions play for their full section duration. Base version 1 retains its controller timing and short transition rules. Adjust character movement speeds in the menu or INI.

### Independent wall-run timelines

`runUp.hkx`, `runLeft.hkx`, `runRight.hkx`, `runDiagonalLeft.hkx` and `runDiagonalRight.hkx` each contain their own complete named animation. Sideways directions also keep their private bracing reference in the same HKX; no shared `sideBrace.hkx` is needed.

For example, `configs/runLeft.json` contains:

- `format: "FreeClimbActionGroup"`, `version: 2`, `group: "wallRun"` and `direction: "runLeft"`.
- `clips` describes this direction's start, loop and ending in the `runLeft` timeline. It also registers that file's private bracing reference under the internal `sideBrace` slot.
- One `sequences` entry with `slot: "runLeft"`; `launch` and `catch` describe the start and ending, and `brace` references the private `runLeftBrace` member in the same file. Other sideways directions have their own reference.
- Inclusive `frameRange` sections, which may share boundary frames. `rootShift` restores each section's local Root translation. The complete file is limited to 10 seconds and 1201 frames.
- Contacts, source timing and trajectories calculated for each section. The tool performs this conversion; authors identify only the complete animation and its loop boundaries.

The loader retains support for older combined formats. Current default files and exports use independent directions. Installing overrides for different directions does not require replacing `pack.json`.

`contextHopLeft.hkx` and `contextHopRight.hkx` likewise contain their own preparation, leap and ending on one timeline. Each paired JSON has `group: "contextHop"`, its own `direction`, and a `sequences` entry with `prepare` and `catch` ranges. The internal `contextHang` stage belongs to that direction only. Imported complete leaps use the full source action without adding another shared preparation or catch.

Some base clips include optional `references` for `launchApproach`, `kickTakeoff`, `kickLanding`, `kickRunLanding` and `kickRunBrace`. Each refers to a private named animation inside the current HKX, without reading another action file. The tool maintains these references automatically; authors do not write them, and external imports do not inherit the base clips' reference animations.

### Matching leap and mantle configuration

The following fields belong to base version 1 configurations. For imported version 2 actions, the tool generates source timing, contacts and routes; authors do not need to fill in these base timing windows manually:

- **`contextHang`:** an internal preparation/settling stage stored separately inside each complete side leap. Its hand and foot positions calibrate that direction only; authors replace the complete left or right action.
- **`contextHopLeft` / `contextHopRight`:** in addition to the ordinary fields, these use `path`, `sourceHands`, `targetHands` and `verticalBlend`. `path` describes the body's route; the other fields determine when the hands release, regrip and blend across a height difference. Timing uses normalized phases from `0..1` and must align with the HKX animation.
- **`contextMantle`:** the base version 1 configuration uses `unplant`, `replant`, `releaseHands` and `replantSamplePhase` for unloading after the first support on the top surface, planting again and finally releasing the hands. This is the sole mantle slot. Check its transitions from climbing, wall running and automatic obstacle jumps; low ledges use its final unloaded segment.

Use the release package's matching `configs/*.json` files as complete examples. Do not replace a special action's HKX while retaining grip timing and a path that no longer match it. Their HKX durations contribute to the corresponding controller segment durations, but the actual route must still pass support and collision checks.

## Actions and internal stages

| Slots | When they are used |
| --- | --- |
| `hang`, `up`, `down`, `left`, `right` | Resting on a wall and four-way climbing. |
| `reach` | The existing entry combination while grounded and close to the target. |
| `jumpCatch`, `ledgeCatch` | Jumping or longer entry approaches, and catching a wall while falling. |
| `hopUp`, `hopLeft`, `hopRight` | Direction + jump while climbing; also checked obstacle recovery with an actual landing. |
| `runUp`, `runLeft`, `runRight`, `runDiagonalLeft`, `runDiagonalRight` | Hold the wall-run modifier after attaching, with the corresponding direction. |
| `runLaunch`, `runLaunchLeft`, `runLaunchRight`, `runCatch` | Transitions between climbing and wall running. The base pack blends these as pose bridges rather than always playing them as complete separate actions; external version 2 replacements use the full clip. |
| `sideBrace` | An internal inner-arm reference stored inside each sideways wall-run file, not a separately edited or installed action. |
| `kickUp`, `kickLeft`, `kickRight` | Automatic directional kicks over wall-run obstacles, after checking the landing, complete route and body clearance. Manual wall-run jumping remains disabled. |
| `drop` | Letting go in place. |
| `backFlipOut`, `dropBack` | Backward + jump to leave the wall: a backflip when fancy jumps and clearance allow it, otherwise an ordinary backward kick. |
| `contextHang` | The selected side leap's own preparation and settling stages, not a separate sustained hanging mode. |
| `contextHopLeft`, `contextHopRight` | Automatic climbing actions or manual side leaps with suitable measured support and a clear route. |
| `contextMantle` | Mantling onto a verified standable top; all mantles use this slot. |

Animation variants do not change the controls. The tool presents complete actions; internal transition and reference slots do not require separate replacement packages. Collision, stamina and settings still determine availability. Detailed `Session action` logs identify selected slots by name; partial arm references are not counted as separate actions.

## Troubleshooting

| Problem | Check first |
| --- | --- |
| Replaced files have no visible effect | Whether MO2 overrides the correct paths, the pack was reloaded, another slot failed to load, and the current action actually selects this slot. |
| Skeleton or track errors | Target skeleton, bone order, bone names and complete exported tracks. Renaming a file does not fix these. |
| Twisted wrists or reversed elbows | Retargeting axes, reference pose and left/right mapping. Successful loading does not guarantee a natural pose. |
| Hands stick to the wall, grab air or feet slide | Contact column order, grip timing, stride, and the paths and timing windows for special actions. |
| Changing duration does not change speed | Moving loops advance by distance and stride; for single actions, confirm that version 2 was exported and reloaded. |
| Sideways leaps or mantles do not appear | Input direction, related settings, actual handholds and a clear route. Replacing files does not remove these conditions. |

Start with the loading results on the animation page. For further troubleshooting, enable detailed diagnostics in the menu, reproduce the issue and save the log. See the [README](../README.md) for its location and recording instructions.

</details>
