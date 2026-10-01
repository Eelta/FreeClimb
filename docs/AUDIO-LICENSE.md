# FreeClimb audio provenance and license

Audio by **Kenney**, licensed **CC0 1.0 Universal**. FreeClimb adapts selected
sounds from Impact Sounds 1.0 and RPG Audio for its climbing system. The
original notices permit personal and commercial use and make attribution
optional. FreeClimb retains Kenney attribution and reproduces both notices
below.

- [Impact Sounds 1.0 — official asset page](https://kenney.nl/assets/impact-sounds)
- [RPG Audio — official asset page](https://kenney.nl/assets/rpg-audio)
- [Kenney support — usage and attribution](https://kenney.nl/support)
- [CC0 1.0 deed](https://creativecommons.org/publicdomain/zero/1.0/)
- [CC0 1.0 legal code](https://creativecommons.org/publicdomain/zero/1.0/legalcode)

## Included sounds

All installed paths are relative to Data. Each WAV is uncompressed PCM16,
mono, 44,100 Hz. The sound controller plays grip contacts during climbing,
boot contacts during wall running, push effects on release, and top-out
contacts when reaching supported ground.

| Runtime WAV under `Sound/fx/FreeClimb/` | Intended group | Original archive members | Duration (s) | Peak (dBFS) |
| --- | --- | --- | ---: | ---: |
| `step01.wav` | step | `impact-sounds/Audio/footstep_concrete_000.ogg` | 0.103 | -5.00 |
| `step02.wav` | step | `impact-sounds/Audio/footstep_concrete_001.ogg` | 0.105 | -5.00 |
| `step03.wav` | step | `impact-sounds/Audio/footstep_concrete_002.ogg` | 0.110 | -5.00 |
| `step04.wav` | step | `impact-sounds/Audio/footstep_concrete_003.ogg` | 0.108 | -5.00 |
| `grip01.wav` | grip | `impact-sounds/Audio/impactSoft_medium_000.ogg` + `rpg-audio/Audio/handleSmallLeather.ogg` | 0.298 | -8.00 |
| `grip02.wav` | grip | `impact-sounds/Audio/impactSoft_medium_001.ogg` + `rpg-audio/Audio/handleSmallLeather2.ogg` | 0.307 | -8.00 |
| `grip03.wav` | grip | `impact-sounds/Audio/impactSoft_medium_002.ogg` + `rpg-audio/Audio/handleSmallLeather.ogg` | 0.298 | -8.00 |
| `push01.wav` | push | `impact-sounds/Audio/footstep_concrete_004.ogg` + `rpg-audio/Audio/cloth1.ogg` | 0.649 | -6.00 |
| `push02.wav` | push | `rpg-audio/Audio/footstep09.ogg` + `rpg-audio/Audio/cloth2.ogg` | 0.406 | -6.00 |
| `top01.wav` | top | `rpg-audio/Audio/footstep00.ogg` | 0.243 | -5.00 |
| `top02.wav` | top | `rpg-audio/Audio/footstep01.ogg` | 0.282 | -5.00 |
| `top03.wav` | top | `rpg-audio/Audio/footstep08.ogg` | 0.256 | -5.00 |

Grip effects combine soft impacts with leather handling. Push-off effects
combine boot contacts with clothing movement. Variants use the source takes
listed above.

## Adaptation

The selected source audio is decoded to floating-point mono, trimmed around
near-silence, adjusted for gain and mixed where listed above. The outputs use
2 ms attack and 15 ms tail fades. Peak targets are -5 dBFS for steps and
top-outs, -8 dBFS for grips and -6 dBFS for push-offs. The final mixes are
encoded as PCM16 WAV.

The runtime package contains the 12 processed effects. Source archive members
and output files are identified below by SHA-256. The original license
notices are reproduced without modification.

## Source archives

- Impact Sounds 1.0: [kenney_impact-sounds.zip](https://kenney.nl/media/pages/assets/impact-sounds/87b4ddecda-1677589768/kenney_impact-sounds.zip)
  SHA-256: `029d734af1582474edf3a694d1b0cebc97c1c152f2f39fa34d4c2bafc5de77f8`
- RPG Audio: [kenney_rpg-audio.zip](https://kenney.nl/media/pages/assets/rpg-audio/8e99002d76-1677590336/kenney_rpg-audio.zip)
  SHA-256: `6dbeaf8544da958d8f2adcb4a4a4b76c1ade34a05f8ab9edccd327da7375f38b`

## Output SHA-256

| File | SHA-256 |
| --- | --- |
| `step01.wav` | `6e77a4a3ac7b42344224417bbafbf889cc55b0fef3ec0b836bd7cf80a3d4b5c1` |
| `step02.wav` | `bdffe9d82a87c46e51b81cf839fd5838a207f3f71da8f8383c8fffb44f92c257` |
| `step03.wav` | `8839c477d560a1c35b0daf5c3882513dedbff3bceaea6da6f78ee12f7c8a3809` |
| `step04.wav` | `43e585ee25aa1fa0567116248fcd8317bc27a1d581ff328bfd9ecf588e5c08ed` |
| `grip01.wav` | `c5a8826d9f99e6b801c369df4a8c529f4bddb781dd6a0182ce006bb04f9de4dd` |
| `grip02.wav` | `00b275c176566962518b50e7718a68fd1ce89a5b772b3222835c978cc9c7472e` |
| `grip03.wav` | `2dcecf5485144a8abbb80ed32801a499180d6dabe63ace1cbfd0e1a6249a6907` |
| `push01.wav` | `bc3c8079b6e6bbc60eeb3c6695669d3315530123b97914c6679c7328d25133c8` |
| `push02.wav` | `d9d9ecd6185a8db94236e98dd49ecf60ac8c3beab1131116a22af4a702d7d05e` |
| `top01.wav` | `0dfee7c05f06e94ac96ad08f5fd827c5197d8929f2aac0ad964f8c092efa5588` |
| `top02.wav` | `064ff5f1ebbb903dca0285cb2c83bbaebe0808030f06b123f37e19f3288abd4e` |
| `top03.wav` | `3ae115412a201750e73ca7a87bea0f8195df093978a33d8f66b09770d7cc5231` |

## Original license notices

### Impact Sounds 1.0

```text
Impact Sounds (1.0)

	Created/distributed by Kenney (www.kenney.nl)
	Creation date: 19-12-2019

			------------------------------

	License: (Creative Commons Zero, CC0)
	http://creativecommons.org/publicdomain/zero/1.0/

	This content is free to use in personal, educational and commercial projects.

	Support us by crediting Kenney or www.kenney.nl (this is not mandatory)

			------------------------------

	Donate:   http://support.kenney.nl
	Request:  http://request.kenney.nl
	Patreon:  http://patreon.com/kenney/

	Follow on Twitter for updates:
	http://twitter.com/KenneyNL
```

### RPG Audio

```text
RPG Audio

	by  Kenney Vleugels (Kenney.nl)

			------------------------------

	License (Creative Commons Zero, CC0)
	http://creativecommons.org/publicdomain/zero/1.0/

	You may use these assets in personal and commercial projects.
	Credit (Kenney or www.kenney.nl) would be nice but is not mandatory.

			------------------------------

	Donate:   http://support.kenney.nl
	Request:  http://request.kenney.nl

	Follow on Twitter for updates:
	@KenneyNL
```
