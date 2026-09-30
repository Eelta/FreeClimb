# FreeClimb Animation Replacement Guide

English | [简体中文](ANIMATION-DIY.zh-CN.md)

Replacing existing animations does not require recompiling the DLL. FreeClimb has **35 active animation slots**, each with an HKX file and a JSON configuration. Override only the animations you want to change, keeping the complete default pack for everything else.

Back to [README](../README.md) · [Chinese README](../README.zh-CN.md)

## Animation Authoring Helper

The complete source archive includes the [Animation Authoring Helper](../tools/authoring/README.md). Install Python 3.10 or later with Tkinter, then double-click `tools/authoring/Start.cmd`:

1. Select the complete default pack's `pack.json`, the slot to replace, and an HKX already adapted to the skeleton.
2. Keep the matching contact configuration, or specify each limb's support start, end and fade times. The tool generates the contact curves.
3. Validate and export a ZIP. Install it after FreeClimb in MO2 so it takes priority, then detach from the wall, reload and test.

There is no need to compile the tool or enter JSON manually. Skeleton adaptation still takes place in animation software; the advanced tab lets you edit the dedicated paths and grip timing windows for sideways leaps and mantles. The following sections cover manual replacement and parameters for reference.

## Replacing a ready-made animation or creating a new one?

- **Animations already adapted to FreeClimb:** usually replace the HKX and its accompanying JSON, then reload and check the result.
- **New FBX files, motion capture or other source animations:** first retarget to the skeleton defined by `skeleton.json` in animation software. Check bone names, order, local axes and lengths, then export a supported HKX. Contact timing and animation parameters also need calibration. Renaming an arbitrary Skyrim HKX is not enough, and raw FBX files cannot be imported with one click.

Supported files are Skyrim SE 64-bit Havok 2010.2 animations: uncompressed interleaved animations or spline-compressed layouts supported by the loader. Behavior files, other skeletons and other Havok formats are not direct replacements.

## Example: replacing upward climbing in the `up` slot

The in-game animation directory is:

```text
Data/meshes/actors/character/animations/FreeClimb/
  pack.json
  skeleton.json
  up.hkx
  configs/up.json
  ...the other 34 HKX and configuration pairs
```

1. **Add the adapted HKX.** The simplest approach is to replace `up.hkx` together with its matching `configs/up.json`.
2. **Check the required parameters.** Keep `slot` set to `up`, and check stride and contact curves using the table below. You normally do not need to edit `pack.json` or `skeleton.json`, or enter skeleton data manually. All `file` paths are relative to the animation directory, not to `configs/`.
3. **Detach, reload and test.** On the menu's animation page, choose **Safely reload animation pack**, confirm that the complete pack loaded successfully, then test wall entry, upward climbing, stopping and mantling. Reload requests made while climbing wait until the player is safely detached; a failed reload preserves the last valid pack. On initial load, missing required files prevent the pack from being enabled.

Even if you replace only one animation, keep the default files for the other 34 slots. You may use new filenames; adding slot names or trigger behavior requires changes to the plugin.

## Which JSON fields matter?

Start with `configs/up.json` or the corresponding configuration from the release package, rather than an empty file.

| Field | What to do |
| --- | --- |
| `format`, `version`, `slot` | Keep the original values; they identify the configuration format and animation slot. |
| `file` | Point to your HKX, such as `up.hkx` or `custom/myUp.hkx`. |
| `stride` | Distance covered by one cycle, in Skyrim game units. It synchronizes limb motion with movement; it is not movement speed per second. |
| `contacts` | Each row contains contact weights for the **left hand, right hand, left foot and right foot**, in that order. `1` requests support, `0` releases it, and intermediate values blend between them. Match the actual grip and release timing. |
| `height`, `travel` | Reference lift and displacement from the animation, mainly used to calibrate mantles and sideways leaps. These are not climbing height limits or commands that directly move the player. |

The `contacts` rows are evenly spaced from the beginning to the end of the clip. The loader resamples them to the HKX frame count, so there is no need to duplicate rows merely to match that count. **Grip and release timing must still match the new animation.** Setting every contact to `1` can pin a hand to the wall when it should be swinging. FreeClimb still determines actual handholds and performs collision checks.

Animation duration comes from the HKX; there is no general JSON `duration` or `loop` switch. Ordinary climbing cycles advance mainly according to movement distance and stride. Basic jumps, wall entry and wall departure retain their own controller timing, so stretching or shortening an HKX does not change every action's speed. Adjust movement speeds in the menu or INI.

## Sideways leaps and mantles: update the matching configuration

These actions coordinate real handholds with the body's route and need more calibration than ordinary climbing loops:

- **`contextHang`:** the brief preparation and settling animation around a sideways leap. Its first-frame hand and foot positions are used for calibration, so check it together with both sideways leaps.
- **`contextHopLeft` / `contextHopRight`:** in addition to the ordinary fields, these use `path`, `sourceHands`, `targetHands` and `verticalBlend`. `path` describes the body's route; the other fields determine when the hands release, regrip and blend across a height difference. Timing uses normalized phases from `0..1` and must align with the HKX animation.
- **`contextMantle`:** uses `unplant`, `replant`, `releaseHands` and `replantSamplePhase` for unloading after the first support on the top surface, planting again and finally releasing the hands. This is the sole mantle slot. Check its transitions from climbing, wall running and automatic obstacle jumps; low ledges use its final unloaded segment.

Use the release package's matching `configs/*.json` files as complete examples. Do not replace a special action's HKX while retaining grip timing and a path that no longer match it. Their HKX durations contribute to the corresponding controller segment durations, but the actual route must still pass support and collision checks.

## The 35 animation slots

| Purpose | Slots |
| --- | --- |
| Resting and four-way climbing | `hang`, `up`, `down`, `left`, `right` |
| Reach transition | `reach` |
| Manual leap catches | `hopLeft`, `hopRight`, `hopUp` |
| Wall departure, entry and airborne catches | `drop`, `jumpCatch`, `sprintCatch`, `dropBack`, `ledgeCatch` |
| Five-direction wall running | `runUp`, `runLeft`, `runRight`, `runDiagonalLeft`, `runDiagonalRight` |
| Wall run starts, catches and bracing | `runLaunch`, `runCatch`, `runLaunchLeft`, `runLaunchRight`, `sideBrace` |
| Kicks and flips | `kickUp`, `kickLeft`, `kickRight`, `flipUp`, `flipLeft`, `flipRight` |
| Outward backflip departure | `backFlipOut` |
| Contextual leap preparation, sideways leaps and mantling | `contextHang`, `contextHopLeft`, `contextHopRight`, `contextMantle` |

## Troubleshooting

| Problem | Check first |
| --- | --- |
| Replaced files have no visible effect | Whether MO2 overrides the correct paths, the pack was reloaded, another slot failed to load, and the current action actually selects this slot. |
| Skeleton or track errors | Target skeleton, bone order, bone names and complete exported tracks. Renaming a file does not fix these. |
| Twisted wrists or reversed elbows | Retargeting axes, reference pose and left/right mapping. Successful loading does not guarantee a natural pose. |
| Hands stick to the wall, grab air or feet slide | Contact column order, grip timing, stride, and the paths and timing windows for special actions. |
| Changing duration does not change speed | Whether the action uses a fixed controller segment, and whether the stride matches ordinary loops. |
| Sideways leaps or mantles do not appear | Input direction, related settings, actual handholds and a clear route. Replacing files does not remove these conditions. |

Start with the loading results on the animation page. For further troubleshooting, enable detailed diagnostics in the menu, reproduce the issue and save the log. See the [README](../README.md) for its location and recording instructions.
