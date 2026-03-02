# Point Motion LiDAR Importer

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

*This project and its outputs are shared under Creative Commons — non-commercial use only. See license below.*

An ongoing Blender add-on by **JibuFilm** for importing Ouster LiDAR PCAP recordings into Blender as animated point cloud sequences.

This tool is part of a larger artistic research project exploring LiDAR as a cinematic medium. More at [jibujin.com](https://jibujin.com) *(site under construction)*

---

## Overview

Point Motion LiDAR Importer enables importing raw Ouster LiDAR recordings into Blender for time-based point cloud visualization. It supports:

- Free camera navigation
- Lighting and rendering inside Blender
- Frame-by-frame animated point cloud playback
- Just-in-time loading of decoded geometry

```
PCAP + JSON
     ↓
Ouster SDK decode
     ↓
Per-frame PLY export
     ↓
Blender animated point cloud sequence
```

The add-on decodes PCAP recordings using the Ouster SDK, exports per-frame PLY files, and loads them as an animated sequence inside Blender.

---

## ⚠️ Development Status

**Early experimental / alpha development. Breaking changes may occur.**

Currently only tested on:
- macOS 11.0+ (Apple Silicon / ARM64)
- CPython 3.11
- Blender 4.2+
- Ouster SDK 0.16.1

---

## Sensor Support

Tested with Ouster LiDAR sensors: OS0, OS1, OS2, OSDome.

Decoding is Ouster-specific via the Ouster SDK. Other manufacturers (Velodyne, Livox, Hesai) use different packet formats and require separate SDK integration. Contributions welcome.

---

## Requirements

- Blender 4.2+
- Ouster SDK 0.16.1
- Matching `.pcap` and `.json` files — both must share the same filename and be in the same folder

---

## Installation

**1. Download all required wheels** (macOS ARM64, Python 3.11):

```bash
pip download ouster-sdk==0.16.1 rosbags blinker certifi \
  charset-normalizer click flask idna ifaddr itsdangerous jinja2 \
  laspy lz4 markupsafe more-itertools numpy packaging pillow \
  prettytable psutil requests ruamel.yaml threadpoolctl \
  typing_extensions urllib3 waitress wcwidth werkzeug zeroconf zstandard \
  --dest ./wheels --only-binary=:all: \
  --python-version=3.11 --platform=macosx_11_0_arm64
```

**2.** Place all downloaded `.whl` files inside `point_motion_lidar_importer/wheels/`

**3.** Ensure `blender_manifest.toml` lists all wheel filenames

**4.** Zip the entire `point_motion_lidar_importer/` folder

**5.** Blender → Edit → Preferences → Get Extensions → Install from Disk → select ZIP → Enable

**6.** Open the **N Panel** → **LiDAR** tab

---

## Usage

**Import from PCAP:**
1. LiDAR panel → **Import PCAP + JSON**
2. Select your `.pcap` file — matching `.json` is auto-detected
3. Frames are decoded and exported to PLY
4. Timeline adjusts automatically to match recording length

**Load existing PLY sequence:**
- Use **Load PLY Sequence** → select any file in the folder
- Loads without re-decoding

**SDK status:**
The LiDAR panel displays ✓ or ✗ depending on whether ouster-sdk was detected successfully.

---

## Contributions

Support for Windows, Linux, Intel macOS, and additional LiDAR manufacturers is not yet implemented. Issues and pull requests are welcome.

---

## Credits

Sequence loading approach inspired by
[blender-sequence-loader](https://github.com/InteractiveComputerGraphics/blender-sequence-loader) (MIT License)
© InteractiveComputerGraphics

LiDAR decoding powered by [Ouster SDK](https://github.com/ouster-lidar/ouster-ros)

---

## License

[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/) — Creative Commons Attribution-NonCommercial 4.0 International

Free to use, modify, and share for non-commercial purposes with credit to JibuFilm.
Commercial use requires explicit permission from JibuFilm.

> Note: `blender_manifest.toml` declares `GPL-3.0-or-later` for Blender extension platform compatibility. The actual license governing this project and its use is CC BY-NC 4.0 as stated above.

---

*Part of an ongoing cinematic research project by JibuFilm — [jibujin.com](https://jibujin.com)*
