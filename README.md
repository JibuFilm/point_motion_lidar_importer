# Point Motion LiDAR Importer
### A Blender Add-on by JibuFilm

Import LiDAR PCAP recordings directly into Blender as animated point cloud sequences.

---

## What it does

- Select a `.pcap` file — JSON metadata is auto-detected from the same folder
- Decodes all frames using Ouster SDK
- Exports PLY frames to a folder named after the PCAP
- Loads them as a just-in-time animated sequence in Blender
- Applies Point Cloud geometry nodes automatically (renderable in Cycles)
- Timeline is set to match your recording length

---

## Installation

1. Download `ouster-sdk` wheel for macOS arm64:
```bash
pip download ouster-sdk==0.16.1 --dest ./wheels --only-binary=:all: \
  --python-version=3.11 --platform=macosx_11_0_arm64
```

2. Place the `.whl` file in the `wheels/` folder and uncomment the line in `blender_manifest.toml`

3. Zip the entire `point_motion_lidar_importer/` folder

4. Blender → Edit → Preferences → Add-ons → Install → select the zip → Enable

---

## Usage

- Open the **N panel** in the 3D viewport → **LiDAR** tab
- Click **Import PCAP + JSON** → select your `.pcap` file
- The add-on decodes and animates automatically
- Or use **Load PLY Sequence** if you've already decoded

---

## Credits

Sequence loading approach inspired by:
[blender-sequence-loader](https://github.com/InteractiveComputerGraphics/blender-sequence-loader) (MIT License)
© InteractiveComputerGraphics

LiDAR decoding powered by [Ouster SDK](https://github.com/ouster-lidar/ouster-ros)

---

## License

MIT
