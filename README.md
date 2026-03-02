# Point Motion LiDAR Importer

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

*This project and its outputs are shared under Creative Commons — non-commercial use only. See license below.*

An on-go Blender add-on project by **JibuFilm** that imports Ouster LiDAR PCAP recordings into Blender as animated point cloud sequences.

*See description below for Contribution and currently supports*

This tool is part of an ongoing art project. More at [jibujin.com](https://jibujin.com) *(site under construction)*

---

## About

Point Motion LiDAR Importer enables the importing of raw LiDAR sensor recordings into Blender's environment. It enables free camera work, lighting, and rendering.

The pipeline decodes Ouster PCAP recordings frame by frame using the Ouster SDK, exports per-frame PLY geometry, and loads them as a just-in-time animated sequence inside Blender.

```
PCAP + JSON
     ↓
Ouster SDK decode
     ↓
Per-frame PLY export
     ↓
Blender animated point cloud sequencer

```

---

## ⚠️ Disclaimer

**This project is in early experimental / alpha development. Expect breaking changes.**

**Currently testing on:**
- macOS 11.0+ (Apple Silicon / ARM64)
- CPython 3.11
- Blender 4.2+
- Ouster SDK 0.16.1

**Sensor support:**
This add-on uses the Ouster SDK for decoding and has only been tested with Ouster LiDAR sensors (OS0, OS1, OS2, OSDome). The PCAP format is universal, but the decoding pipeline is Ouster-specific. Other manufacturers use different packet formats and would require their own SDK integration.

**Contributions welcome:**
Support for additional platforms (Windows, Linux, Intel macOS) and other LiDAR manufacturers is not yet implemented. Pull requests and issues are encouraged.

Use at your own risk.

---

## Requirements

- Blender 4.2+
- Ouster SDK 0.16.1 wheel (see Installation)
- PCAP and JSON metadata files from an Ouster sensor *Both files must share the same name and be in the same folder

---

## Installation

1. Download the Ouster SDK wheel for your platform:
```bash
pip download ouster-sdk==0.16.1 --dest ./wheels --only-binary=:all: \
  --python-version=3.11 --platform=macosx_11_0_arm64
```

2. Place the `.whl` file inside the `wheels/` folder

3. Uncomment the wheels line in `blender_manifest.toml`:
```toml
wheels = [
  "./wheels/ouster_sdk-0.16.1-cp311-cp311-macosx_11_0_arm64.whl",
]
```

4. Zip the entire `point_motion_lidar_importer/` folder

5. Blender → Edit → Preferences → Add-ons → Install → select the zip → Enable

6. Open the **N panel** in the 3D viewport → **LiDAR** tab

---

## Usage

**Import from PCAP:**
1. LiDAR panel → **Import PCAP + JSON**
2. Select your `.pcap` file — the matching `.json` is detected automatically
3. The add-on decodes all frames, exports PLYs, and animates the sequence
4. Timeline is set automatically to match your recording length

**Load existing PLY sequence:**
- If you've already decoded a PCAP, use **Load PLY Sequence** to load the folder directly without re-decoding

**SDK status:**
The LiDAR panel shows a ✓ or ✗ indicator confirming whether ouster-sdk was found correctly.

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

---

*Part of an ongoing art project by JibuFilm — [jibujin.com](https://jibujin.com)*
