# Point Motion LiDAR Importer
# Copyright (C) 2026 JibuFilm
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Part of an ongoing cinematic research project by JibuFilm
# https://www.jibujin.com


bl_info = {
    "name": "Point Motion LiDAR Importer",
    "author": "JibuFilm",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > LiDAR Beta",
    "description": "Import Ouster LiDAR PCAP/OSF captures as animated point-cloud sequences.",
    "category": "Import-Export",
}

import bpy
import os
import json
import tarfile
import itertools
import numpy as np
from collections import OrderedDict
from mathutils import Vector, Matrix
from bpy.props import (StringProperty, IntProperty, BoolProperty,
                       FloatProperty, FloatVectorProperty, EnumProperty)
from bpy.types import Operator, Panel
from bpy_extras.io_utils import ImportHelper
from bpy.app.handlers import persistent

# ===========================================================================
# Ouster SDK
# ===========================================================================

def get_ouster_sdk():
    try:
        from ouster.sdk import open_source
        import ouster.sdk.core as core
        return open_source, core
    except ImportError:
        return None, None


def read_sensor_info(json_path):
    """Parse a sensor JSON without opening the PCAP."""
    _, core = get_ouster_sdk()
    if core is None:
        return None
    try:
        with open(json_path, 'r') as f:
            return core.SensorInfo(f.read())
    except Exception as e:
        print(f"[PointMotion] Could not read sensor info from "
              f"{os.path.basename(json_path)}: {e}")
        return None


def sensor_label(json_path):
    """
    Readable label: "OS-1-128  ·  SN: 122448000402  ·  1024x10"
    Falls back to filename if SDK unavailable.
    """
    info = read_sensor_info(json_path)
    if info is None:
        return os.path.basename(json_path)
    try:
        mode = str(info.config.lidar_mode).split(".")[-1]
        return f"{info.prod_line}  ·  SN: {info.sn}  ·  {mode}"
    except Exception:
        return os.path.basename(json_path)


def sensor_label_from_info(info):
    """Readable label from a SensorInfo object (for OSF)."""
    if info is None:
        return "Unknown sensor"
    try:
        mode = str(info.config.lidar_mode).split(".")[-1]
        return f"{info.prod_line}  ·  SN: {info.sn}  ·  {mode}"
    except Exception:
        try:
            return f"{info.prod_line}  ·  SN: {info.sn}"
        except Exception:
            return "Unknown sensor"


# ===========================================================================
# TAR extraction
# ===========================================================================

def extract_metadata_tar(tar_path, pcap_stem):
    """
    Extract a metadata-only TAR into <pcap_stem>_metadata/ next to the tar.
    Only top-level JSONs are returned as sensor calibration candidates.
    Subfolders like settings/ are extracted but ignored for sensor matching.

    Returns: (metadata_dir: str, sensor_jsons: list[str])
    """
    tar_dir      = os.path.dirname(os.path.abspath(tar_path))
    metadata_dir = os.path.join(tar_dir, f"{pcap_stem}_metadata")
    os.makedirs(metadata_dir, exist_ok=True)

    sensor_jsons = []

    with tarfile.open(tar_path, 'r:*') as tar:
        for member in tar.getmembers():
            if not member.isfile():
                continue
            safe_name = member.name.replace("\\", "/")
            out_path = os.path.join(metadata_dir, safe_name)
            out_path = os.path.realpath(out_path)
            if not out_path.startswith(
                    os.path.realpath(metadata_dir) + os.sep):
                print(f"[PointMotion] Skipping unsafe tar entry: "
                      f"{member.name}")
                continue
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            if not os.path.exists(out_path):
                # F1 hardening (2026-04-18 audit): size-cap + chunked copy
                # guards against crafted tar members causing memory spikes.
                # Real sensor sidecar JSONs are ~50 KB; 50 MB is generous.
                if member.size is not None and \
                        member.size > 50 * 1024 * 1024:
                    print(f"[PointMotion] Skipping oversized tar member "
                          f"({member.size} bytes): {member.name}")
                    continue
                with tar.extractfile(member) as src, \
                     open(out_path, 'wb') as dst:
                    while True:
                        chunk = src.read(65536)
                        if not chunk:
                            break
                        dst.write(chunk)
            depth = len(member.name.replace("\\", "/").split("/"))
            if depth == 1 and member.name.lower().endswith(".json"):
                sensor_jsons.append(out_path)
                print(f"[PointMotion] Sensor JSON: {member.name}")
            elif member.name.lower().endswith(".json"):
                print(f"[PointMotion] Extra config: {member.name}")

    if not sensor_jsons:
        raise ValueError(
            f"No top-level .json found in {os.path.basename(tar_path)}.\n"
            f"Contents extracted to: {metadata_dir}"
        )

    return metadata_dir, sorted(sensor_jsons)


def find_metadata_for_pcap(pcap_path):
    """
    Locate sensor JSON(s) for a PCAP. Detection order:
      1. Exact name match:  BIE.pcap + BIE.json
      2. Pre-extracted:     <pcap_stem>_metadata/ folder already exists
      3. TAR in folder:     extract → <pcap_stem>_metadata/
      4. Single JSON:       only one .json in folder

    Returns: list[str]
    """
    folder = os.path.dirname(os.path.abspath(pcap_path))
    stem   = os.path.splitext(os.path.basename(pcap_path))[0]

    # 1. Exact match
    exact = os.path.join(folder, stem + ".json")
    if os.path.exists(exact):
        print(f"[PointMotion] Metadata: exact match → {stem}.json")
        return [exact]

    # 2. Pre-extracted metadata folder
    metadata_dir = os.path.join(folder, f"{stem}_metadata")
    if os.path.isdir(metadata_dir):
        jsons = sorted([
            os.path.join(metadata_dir, f)
            for f in os.listdir(metadata_dir)
            if f.lower().endswith(".json")
               and not f.startswith(".")            # skip hidden
               and os.path.dirname(f) == ""         # top-level only
        ])
        # Fallback: if the above is empty, grab all JSONs directly in dir
        if not jsons:
            jsons = sorted([
                os.path.join(metadata_dir, f)
                for f in os.listdir(metadata_dir)
                if f.lower().endswith(".json") and not f.startswith(".")
            ])
        if jsons:
            print(f"[PointMotion] Metadata: cached {stem}_metadata/ "
                  f"({len(jsons)} sensor(s))")
            return jsons

    # 3. TAR in same folder
    tars = [
        os.path.join(folder, f) for f in os.listdir(folder)
        if f.lower().endswith(".tar") or f.lower().endswith(".tar.gz")
    ]
    if tars:
        print(f"[PointMotion] TAR found: {os.path.basename(tars[0])} "
              f"— extracting...")
        _, sensor_jsons = extract_metadata_tar(tars[0], stem)
        return sensor_jsons

    # 4. Single JSON fallback
    jsons = [os.path.join(folder, f) for f in os.listdir(folder)
             if f.lower().endswith(".json")]
    if len(jsons) == 1:
        print(f"[PointMotion] Metadata: single json → "
              f"{os.path.basename(jsons[0])}")
        return jsons
    if len(jsons) > 1:
        raise ValueError(
            f"Multiple .json files found — cannot auto-match:\n"
            + "\n".join(f"  {os.path.basename(j)}" for j in sorted(jsons))
            + "\nPlace them in a TAR or rename to match the PCAP stem."
        )

    raise FileNotFoundError(
        f"No metadata found for {os.path.basename(pcap_path)}.\n"
        f"Folder checked: {folder}"
    )


def find_extrinsics_file(pcap_path, metadata_dir=None):
    """
    Look for Ouster extrinsic calibration.

    Search order:
      1. metadata_dir:  extrinsic*.json files (recursive)
      2. Source folder:  extrinsic*.json files
      3. Source folder:  *_configuration.tar  →  extrinsics_parameters.json
         (Ouster Perception standard TAR structure)

    Supported JSON formats:
      A) Ouster standard (SDK / Detect / Percept):
         { "transforms": [ { "source_frame": "SN",
           "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z",
           "destination_frame": "world" } ] }
      B) Simple per-SN:
         { "SN": { "transform": [16 floats column-major] } }
         { "SN": [16 floats] }

    Returns: dict { serial_number_str: np.array(4,4) } or None
    """
    def _quat_pos_to_matrix(px, py, pz, qw, qx, qy, qz):
        """Quaternion (w,x,y,z) + position → 4x4 homogeneous matrix."""
        # Rotation from quaternion
        r00 = 1 - 2*(qy*qy + qz*qz)
        r01 = 2*(qx*qy - qz*qw)
        r02 = 2*(qx*qz + qy*qw)
        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx*qx + qz*qz)
        r12 = 2*(qy*qz - qx*qw)
        r20 = 2*(qx*qz - qy*qw)
        r21 = 2*(qy*qz + qx*qw)
        r22 = 1 - 2*(qx*qx + qy*qy)
        return np.array([
            [r00, r01, r02, px],
            [r10, r11, r12, py],
            [r20, r21, r22, pz],
            [0,   0,   0,   1 ],
        ], dtype=np.float64)

    def _reject_nonfinite(token):
        """Raise on NaN / Infinity / -Infinity JSON literals.

        Audit Finding 3 MEDIUM (v1.6.14): prevents malformed or
        adversarial extrinsics files from silently producing a
        NaN-filled 4x4 that would then corrupt every frame via
        `_apply_transform`.
        """
        raise ValueError(f"non-finite JSON constant: {token}")

    def _parse_extrinsics_json(data):
        """Parse extrinsics JSON data in any supported format."""
        result = {}

        # Format A: Ouster standard  { "transforms": [ ... ] }
        if "transforms" in data and isinstance(data["transforms"], list):
            for t in data["transforms"]:
                sn = str(t.get("source_frame", ""))
                if not sn:
                    continue
                try:
                    # Coerce to float and validate finiteness before
                    # feeding into the rotation matrix.
                    _vals = [
                        float(t["p_x"]), float(t["p_y"]),
                        float(t["p_z"]), float(t["q_w"]),
                        float(t["q_x"]), float(t["q_y"]),
                        float(t["q_z"])]
                    if not all(np.isfinite(v) for v in _vals):
                        raise ValueError(
                            f"non-finite quat/pos value in "
                            f"transform: {_vals}")
                    mat = _quat_pos_to_matrix(*_vals)
                    result[sn] = mat
                except (KeyError, ValueError) as e:
                    print(f"[PointMotion] Skipping transform for "
                          f"{sn}: {e}")
            return result

        # Format B: per-SN dict
        for key, val in data.items():
            if isinstance(val, dict) and "transform" in val:
                mat = np.array(val["transform"],
                               dtype=np.float64).reshape(4, 4)
                result[str(key)] = mat
            elif isinstance(val, list) and len(val) == 16:
                mat = np.array(val, dtype=np.float64).reshape(4, 4)
                result[str(key)] = mat
        return result

    # ── Collect candidate files ───────────────────────────────────────
    candidates = []
    if metadata_dir and os.path.isdir(metadata_dir):
        for root, _dirs, files in os.walk(metadata_dir):
            for f in files:
                if f.lower().startswith("extrinsic") \
                        and f.lower().endswith(".json"):
                    candidates.append(os.path.join(root, f))

    folder = os.path.dirname(os.path.abspath(pcap_path))
    if os.path.isdir(folder):
        for f in os.listdir(folder):
            if f.lower().startswith("extrinsic") \
                    and f.lower().endswith(".json"):
                candidates.append(os.path.join(folder, f))

    # ── Try _configuration.tar (Ouster Perception standard) ───────────
    if not candidates and os.path.isdir(folder):
        for f in os.listdir(folder):
            if f.lower().endswith("_configuration.tar") \
                    or f.lower().endswith("_configuration.tar.gz"):
                tar_path = os.path.join(folder, f)
                try:
                    with tarfile.open(tar_path, 'r:*') as tar:
                        for member in tar.getmembers():
                            if member.isfile() and \
                                    "extrinsic" in member.name.lower() \
                                    and member.name.lower() \
                                    .endswith(".json"):
                                extracted = tar.extractfile(member)
                                if extracted:
                                    # parse_constant rejects NaN/Inf
                                    # literals (audit Finding 3, v1.6.14)
                                    data = json.load(
                                        extracted,
                                        parse_constant=_reject_nonfinite)
                                    result = _parse_extrinsics_json(
                                        data)
                                    if result:
                                        print(
                                            f"[PointMotion] Extrinsics "
                                            f"from {f}/"
                                            f"{member.name} "
                                            f"({len(result)} sensor(s))"
                                        )
                                        return result
                except Exception as e:
                    print(f"[PointMotion] Could not read "
                          f"configuration tar {f}: {e}")

    # ── Parse candidate JSON files ────────────────────────────────────
    for cpath in candidates:
        try:
            with open(cpath, 'r') as f:
                # parse_constant rejects NaN/Inf literals
                # (audit Finding 3, v1.6.14)
                data = json.load(f, parse_constant=_reject_nonfinite)
            result = _parse_extrinsics_json(data)
            if result:
                print(f"[PointMotion] Extrinsics loaded from "
                      f"{os.path.basename(cpath)} "
                      f"({len(result)} sensor(s))")
                return result
        except Exception as e:
            print(f"[PointMotion] Could not parse extrinsics "
                  f"{os.path.basename(cpath)}: {e}")
    return None


# ===========================================================================
# Module-level state for sensor picker dialog
# ===========================================================================

_pending_source_path   = ""
_pending_sensor_jsons  = []      # populated for PCAP, empty for OSF
_pending_sensor_labels = []
_pending_is_osf        = False
_pending_sensor_infos  = []      # populated for OSF


def _sensor_enum_items(self, context):
    if not _pending_sensor_labels:
        return [("NONE", "No sensors found", "")]
    items = [
        (str(i), _pending_sensor_labels[i],
         f"Sensor {i}: {_pending_sensor_labels[i]}")
        for i in range(len(_pending_sensor_labels))
    ]
    # "All Sensors" is only available for OSF — PCAP multi-sensor
    # decode is unreliable in SDK 0.16.x (v1.4.0: disabled for PCAP)
    if len(items) > 1 and _pending_is_osf:
        items.insert(0, ("ALL", "All Sensors",
                         "Import every sensor as a separate sequence"))
    return items


# ===========================================================================
# Extrinsic helpers
# ===========================================================================

def _get_extrinsic_from_info(sensor_info):
    """
    Extract 4x4 extrinsic from sensor metadata.
    Returns np.array(4,4) or None if identity / unavailable.
    """
    try:
        ext = np.array(sensor_info.extrinsic, dtype=np.float64).reshape(4, 4)
        if np.allclose(ext, np.eye(4)):
            return None
        return ext
    except Exception:
        return None


def _apply_transform(points, matrix_4x4):
    """Apply a 4x4 homogeneous transform to Nx3 points."""
    rot   = matrix_4x4[:3, :3]
    trans = matrix_4x4[:3, 3]
    return (rot @ points.T).T + trans


_PM_SEQUENCE_MANIFEST = "_pm_sequence_manifest.json"
_PM_MANIFEST_VERSION = 1
_PM_MANIFEST_LAYOUT = "point_motion_binary_float32_v1"
_PM_MANIFEST_SAMPLE_EVERY = 10
_PM_MANIFEST_SAMPLE_VALUES = 4096
_PM_MANIFEST_MAX_VALUES = 262144
_sequence_manifest_cache = {}


def _sequence_manifest_path(output_dir):
    return os.path.join(output_dir, _PM_SEQUENCE_MANIFEST)


def _load_sequence_manifest(ply_dir):
    """Load and memoize a sequence manifest if present."""
    if ply_dir in _sequence_manifest_cache:
        return _sequence_manifest_cache[ply_dir]
    manifest_path = _sequence_manifest_path(ply_dir)
    manifest = None
    if os.path.exists(manifest_path):
        # F2 hardening (2026-04-18 audit): reject NaN/Inf literals.
        def _reject_nonfinite(token):
            raise ValueError(f"non-finite JSON constant: {token}")
        try:
            with open(manifest_path, 'r') as mf:
                manifest = json.load(
                    mf, parse_constant=_reject_nonfinite)
        except Exception:
            manifest = None
    _sequence_manifest_cache[ply_dir] = manifest
    return manifest


def _sample_attr_values(values, max_values):
    """Deterministically subsample finite values to a bounded size."""
    finite = values[np.isfinite(values)]
    if len(finite) == 0:
        return None
    finite = finite.astype(np.float32, copy=False)
    if len(finite) <= max_values:
        return finite
    idx = np.linspace(0, len(finite) - 1, max_values, dtype=np.int64)
    return finite[idx]


class _SequenceManifestTracker:
    """Accumulate lightweight sequence metadata during frame export."""

    def __init__(self, sample_every=_PM_MANIFEST_SAMPLE_EVERY,
                 sample_values=_PM_MANIFEST_SAMPLE_VALUES,
                 max_values=_PM_MANIFEST_MAX_VALUES):
        self.sample_every = max(1, int(sample_every))
        self.sample_values = max(1, int(sample_values))
        self.max_values = max(self.sample_values, int(max_values))
        self.frame_count = 0
        self.sampled_frames = 0
        self.schema = []
        self.attr_samples = {}
        self.total_frame_bytes = 0
        self.max_frame_bytes = 0

    def update(self, points, attrs=None):
        frame_bytes = int(points.nbytes)
        if attrs:
            frame_bytes += sum(arr.nbytes for arr in attrs.values())
            for col in _ATTR_COLS:
                if col in attrs and col not in self.schema:
                    self.schema.append(col)
        self.total_frame_bytes += frame_bytes
        if frame_bytes > self.max_frame_bytes:
            self.max_frame_bytes = frame_bytes

        should_sample = (self.frame_count % self.sample_every) == 0
        if should_sample and attrs:
            self.sampled_frames += 1
            for name, values in attrs.items():
                if name in ("nx", "ny", "nz"):
                    continue
                sample = _sample_attr_values(values, self.sample_values)
                if sample is None:
                    continue
                existing = self.attr_samples.get(name)
                if existing is None:
                    self.attr_samples[name] = sample
                    continue
                combined = np.concatenate([existing, sample])
                if len(combined) > self.max_values:
                    idx = np.linspace(
                        0, len(combined) - 1, self.max_values,
                        dtype=np.int64)
                    combined = combined[idx]
                self.attr_samples[name] = combined

        self.frame_count += 1

    def build_manifest(self, extra=None):
        attr_ranges = {}
        for name, sample in self.attr_samples.items():
            vmin, vmax = _compute_attr_display_range(name, sample)
            attr_ranges[name] = [float(vmin), float(vmax)]
        manifest = {
            "version": _PM_MANIFEST_VERSION,
            "layout": _PM_MANIFEST_LAYOUT,
            "frame_count": int(self.frame_count),
            "schema": list(self.schema),
            "attr_ranges": attr_ranges,
            "sample_every": int(self.sample_every),
            "sample_values_per_frame": int(self.sample_values),
            "sampled_frames": int(self.sampled_frames),
            "avg_frame_bytes": int(
                self.total_frame_bytes / self.frame_count)
            if self.frame_count > 0 else 0,
            "max_frame_bytes": int(self.max_frame_bytes),
        }
        if extra:
            manifest.update(extra)
        return manifest


def _write_sequence_manifest(output_dir, tracker, extra=None):
    """Write the sequence manifest produced during export/build."""
    manifest = tracker.build_manifest(extra=extra)
    manifest_path = _sequence_manifest_path(output_dir)
    with open(manifest_path, 'w') as mf:
        json.dump(manifest, mf, indent=2)
    _sequence_manifest_cache[output_dir] = manifest
    return manifest


# ===========================================================================
# Decode — unified PCAP / OSF, fixed multi-sensor
# ===========================================================================

def decode_to_ply(source_path, sensor_jsons, sensor_idx, output_dir,
                  apply_extrinsic=False, extrinsic_overrides=None,
                  include_attributes=False, max_frames=0):
    """
    Decode one sensor stream from a PCAP or OSF to per-frame PLY files.

    source_path:         path to .pcap or .osf
    sensor_jsons:        list of JSON paths (PCAP) or None/[] (OSF)
    sensor_idx:          which sensor to decode (index)
    output_dir:          where to write frame_NNNN.ply files
    apply_extrinsic:     if True, apply the sensor's extrinsic transform
    extrinsic_overrides: dict { sn_str: np.array(4,4) } from external file
    include_attributes:  if True, extract per-point attributes
                         (signal, reflectivity, near_ir, range, normals,
                          dual-return, window, timestamps, IMU, poses)

    Returns: (output_dir, frame_count)
    """
    open_source, core = get_ouster_sdk()
    if open_source is None:
        raise ImportError("ouster-sdk not available — check wheels/ folder.")

    os.makedirs(output_dir, exist_ok=True)
    manifest_tracker = _SequenceManifestTracker()

    # ── Log file next to the source ───────────────────────────────────────
    log_path = os.path.join(os.path.dirname(source_path),
                            "pointmotion_decode.log")

    def log(msg):
        print(f"[PointMotion] {msg}")
        with open(log_path, 'a') as lf:
            lf.write(msg + "\n")

    with open(log_path, 'w') as lf:
        lf.write("=== Point Motion Decode Log — v1.5.6 ===\n")

    is_osf = source_path.lower().endswith(".osf")
    log(f"Source: {os.path.basename(source_path)}  "
        f"({'OSF' if is_osf else 'PCAP'})")
    log(f"Output: {output_dir}")
    log(f"Requested sensor_idx: {sensor_idx}")
    log(f"Apply extrinsic: {apply_extrinsic}")
    log(f"Include attributes: {include_attributes}")

    # ── Open source ───────────────────────────────────────────────────────
    try:
        if is_osf or not sensor_jsons:
            source = open_source(source_path)
            log("Opened OSF (metadata embedded)")
        else:
            log(f"Metadata files ({len(sensor_jsons)}):")
            for j in sensor_jsons:
                info_pre = read_sensor_info(j)
                if info_pre:
                    log(f"  {os.path.basename(j)}  →  "
                        f"{info_pre.prod_line}  SN:{info_pre.sn}")
                else:
                    log(f"  {os.path.basename(j)}  (could not parse)")
            source = open_source(source_path, meta=sensor_jsons)
    except Exception as e:
        log(f"ERROR opening source: {e}")
        raise

    # ── Resolve sensor info list from the source ──────────────────────────
    try:
        all_info = source.sensor_info
        n_sensors = len(all_info)
        log(f"SDK resolved {n_sensors} sensor stream(s):")
        for i, si in enumerate(all_info):
            log(f"  [{i}] {si.prod_line}  SN:{si.sn}")
    except Exception as e:
        log(f"Could not read sensor_info: {e}")
        raise

    # ── Map sensor_idx to the correct source slot ─────────────────────────
    resolved_idx = min(sensor_idx, n_sensors - 1)

    if sensor_jsons and sensor_idx < len(sensor_jsons):
        target_info = read_sensor_info(sensor_jsons[sensor_idx])
        target_sn   = str(target_info.sn) if target_info else None
        log(f"Target SN (from JSON): {target_sn}")

        if target_sn:
            for i, si in enumerate(all_info):
                if str(si.sn) == target_sn:
                    resolved_idx = i
                    log(f"Target SN:{target_sn} → "
                        f"source.sensor_info[{i}] ✓")
                    break
            else:
                log(f"WARNING: Target SN:{target_sn} not found in "
                    f"source.sensor_info — using slot {resolved_idx}")
    else:
        log(f"Direct index mapping → slot {resolved_idx}")

    info = all_info[resolved_idx]
    log(f"Decoding slot {resolved_idx}: "
        f"{info.prod_line}  SN:{info.sn}")

    # ── Build XYZ look-up table ───────────────────────────────────────────
    use_sdk_extrinsic = False
    manual_transform  = None

    if apply_extrinsic:
        sn_str = str(info.sn)
        if extrinsic_overrides and sn_str in extrinsic_overrides:
            manual_transform = extrinsic_overrides[sn_str]
            log(f"Using EXTERNAL extrinsic for SN:{sn_str}")
        else:
            embedded = _get_extrinsic_from_info(info)
            if embedded is not None:
                use_sdk_extrinsic = True
                log(f"Using EMBEDDED extrinsic from sensor metadata")
            else:
                log(f"No extrinsic found for SN:{sn_str} — "
                    f"points in sensor frame")

    xyz_lut = core.XYZLut(info, use_extrinsics=use_sdk_extrinsic)

    # ── Choose iteration strategy ─────────────────────────────────────────
    #
    # Strategy A: open UNCOLLATED source → single_source(idx)
    #   The SDK error "Cannot get a single stream from an already collated
    #   source" means we must open with collate=False first, THEN call
    #   .single_source(idx).  This yields individual LidarScan objects
    #   correctly isolated per sensor.
    #
    # Strategy C: open_source with explicit sensor_idx
    #   Re-opens the file targeting one sensor.  The source is used
    #   directly — we do NOT pre-consume with next() because that can
    #   break some SDK iterator implementations.
    #
    # Strategy B: collated scanset[idx]  — last fallback
    #   Index into the collated scanset per frame.  May yield identical
    #   data for both sensors on some PCAPs (SDK port-demux issue).
    #
    scan_iter       = None
    strategy_name   = None
    is_single_scan  = False   # True → iterator yields LidarScan directly
    _strategy_src   = None    # keep reference alive for GC

    # ── Strategy A: uncollated source → single_source() ───────────────────
    if n_sensors > 1:
        try:
            if is_osf or not sensor_jsons:
                uncollated = open_source(source_path, collate=False)
            else:
                uncollated = open_source(source_path, meta=sensor_jsons,
                                         collate=False)
            _strategy_src = uncollated   # prevent GC

            # Try single_source / single methods
            for method_name in ("single_source", "single"):
                method = getattr(uncollated, method_name, None)
                if method is None:
                    continue
                try:
                    single = method(resolved_idx)
                    scan_iter      = iter(single)
                    strategy_name  = (f"uncollated.{method_name}"
                                      f"({resolved_idx})")
                    is_single_scan = True

                    # Update info + xyz_lut from the single source
                    if hasattr(single, 'sensor_info'):
                        si_list = single.sensor_info
                        if si_list:
                            info = si_list[0]
                            xyz_lut = core.XYZLut(
                                info,
                                use_extrinsics=use_sdk_extrinsic)

                    log(f"Strategy A: {strategy_name}")
                    break
                except Exception as e:
                    log(f"Strategy A ({method_name}) failed: {e}")
        except Exception as e:
            log(f"Strategy A (uncollated open) failed: {e}")

    # ── Strategy C: re-open with explicit sensor_idx ──────────────────────
    if scan_iter is None and n_sensors > 1:
        try:
            if is_osf or not sensor_jsons:
                src2 = open_source(source_path,
                                   sensor_idx=resolved_idx)
            else:
                src2 = open_source(source_path, meta=sensor_jsons,
                                   sensor_idx=resolved_idx)
            _strategy_src = src2   # prevent GC

            # Use the source directly — do NOT pre-consume with next()
            scan_iter      = iter(src2)
            strategy_name  = (f"open_source(sensor_idx="
                              f"{resolved_idx})")
            is_single_scan = True

            # Update info + xyz_lut from this source
            if hasattr(src2, 'sensor_info'):
                si_list = src2.sensor_info
                if si_list:
                    info = si_list[0] if len(si_list) == 1 \
                        else si_list[min(resolved_idx,
                                         len(si_list) - 1)]
                    xyz_lut = core.XYZLut(
                        info, use_extrinsics=use_sdk_extrinsic)

            log(f"Strategy C: {strategy_name}")
        except Exception as e:
            log(f"Strategy C failed: {e}")
            scan_iter = None

    # ── Strategy B: collated with verification ────────────────────────────
    if scan_iter is None:
        scan_iter       = iter(source)
        strategy_name   = (f"Collated scanset[{resolved_idx}] "
                           f"(fallback)")
        is_single_scan  = False
        log(f"Strategy B: {strategy_name}")

        if n_sensors > 1 and resolved_idx > 0:
            log("Verifying slot data differs between sensors...")
            try:
                probe_source = open_source(
                    source_path, meta=sensor_jsons
                ) if sensor_jsons else open_source(source_path)
                probe_iter = iter(probe_source)
                probe_set  = next(probe_iter, None)
                if probe_set is not None:
                    s0 = probe_set[0]
                    sN = probe_set[resolved_idx]
                    if s0 is not None and sN is not None:
                        xyz0 = core.XYZLut(all_info[0])(s0)
                        xyzN = xyz_lut(sN)
                        fp0  = float(np.sum(np.abs(
                            xyz0.reshape(-1, 3)[:100])))
                        fpN  = float(np.sum(np.abs(
                            xyzN.reshape(-1, 3)[:100])))
                        if abs(fp0 - fpN) < 1e-3:
                            log("⚠ WARNING: Slot 0 and slot "
                                f"{resolved_idx} frame-0 data is "
                                f"IDENTICAL (fp0={fp0:.2f}, "
                                f"fpN={fpN:.2f})")
                            log("  The SDK collated source is not "
                                "separating sensors for this PCAP.")
                            log("  Points will be from sensor 0 "
                                "regardless of slot selected.")
                            log("  To fix: convert PCAP → OSF with "
                                "'ouster-cli source <pcap> save "
                                "<output.osf>' then import the OSF.")
                        else:
                            log(f"Slot data verified DIFFERENT "
                                f"(fp0={fp0:.2f}, fpN={fpN:.2f}) ✓")
                    elif sN is None:
                        log(f"⚠ WARNING: Slot {resolved_idx} "
                            f"returned None on frame 0")
                del probe_source
            except Exception as e:
                log(f"Verification probe error (non-fatal): {e}")

    log(f"Decode strategy: {strategy_name}")

    # ── Iterate & write PLY frames ────────────────────────────────────────
    frame_count = 0
    none_count  = 0

    # For single_source / open_source(sensor_idx=N):
    #   the LidarScanSet has 1 entry → index 0
    # For collated source:
    #   the LidarScanSet has N entries → index resolved_idx
    scanset_idx = 0 if is_single_scan else resolved_idx

    try:
        for item in scan_iter:
            if item is None:
                none_count += 1
                continue

            # SDK 0.16.x breaking change: ALL iterators yield
            # LidarScanSet, never bare LidarScan.  We must index
            # into the scanset to get the actual LidarScan.
            scan = None
            try:
                scan = item[scanset_idx]
            except (IndexError, TypeError, KeyError):
                # Older SDK or unexpected type — try it as-is if it
                # looks like a LidarScan (has .field attribute)
                if hasattr(item, 'field') or hasattr(item, 'astype'):
                    scan = item

            if frame_count == 0:
                log(f"  Item type: {type(item).__name__}, "
                    f"scanset_idx: {scanset_idx}")
                if scan is not None:
                    log(f"  Scan type: {type(scan).__name__}")
                else:
                    log(f"  Scan is None after indexing")

            if scan is None:
                none_count += 1
                continue

            xyz    = xyz_lut(scan)
            points = xyz.reshape(-1, 3)
            mask   = np.sum(points * points, axis=1) > 0.01  # 0.1² avoids sqrt

            # Refine mask with FLAGS if available
            try:
                flags = scan.field(core.ChanField.FLAGS).reshape(-1)
                mask &= (flags == 0)
            except Exception:
                pass

            points = points[mask]

            if manual_transform is not None:
                points = _apply_transform(points, manual_transform)

            # ── Attribute extraction ──────────────────────────────
            frame_attrs = None
            frame_meta = None
            if include_attributes:
                frame_attrs = {}

                # Cache scan.field() results to avoid repeat SDK calls
                _field_cache = {}
                def _get_field(chan_name):
                    if chan_name not in _field_cache:
                        try:
                            key = getattr(core.ChanField, chan_name)
                            _field_cache[chan_name] = scan.field(key)
                        except Exception:
                            _field_cache[chan_name] = None
                    return _field_cache[chan_name]

                # Standard scalar fields
                for field_name, chan in (
                    ("signal",         "SIGNAL"),
                    ("reflectivity",   "REFLECTIVITY"),
                    ("near_ir",        "NEAR_IR"),
                    ("range",          "RANGE"),
                    ("signal2",        "SIGNAL2"),
                    ("reflectivity2",  "REFLECTIVITY2"),
                    ("range2",         "RANGE2"),
                    ("window",         "WINDOW"),
                ):
                    raw = _get_field(chan)
                    if raw is not None:
                        frame_attrs[field_name] = raw.reshape(-1) \
                            .astype(np.float32)[mask]

                # FLAGS2 — filter dual-return fields if present
                flags2_raw = _get_field("FLAGS2")
                if flags2_raw is not None:
                    flags2 = flags2_raw.reshape(-1)
                    dual_valid = (flags2 == 0)
                    for dr_field in ("signal2", "reflectivity2",
                                     "range2"):
                        if dr_field in frame_attrs:
                            frame_attrs[dr_field] = np.where(
                                dual_valid[mask],
                                frame_attrs[dr_field], 0.0)

                # AutoExposure + BUC on NEAR_IR (reuse cached field)
                if "near_ir" in frame_attrs:
                    try:
                        from ouster.sdk.core._utils import (
                            AutoExposure, BeamUniformityCorrector)
                        if not hasattr(decode_to_ply, '_ae'):
                            decode_to_ply._ae = AutoExposure()
                            decode_to_ply._buc = \
                                BeamUniformityCorrector()
                        nir_2d = _get_field("NEAR_IR")
                        if nir_2d is not None:
                            nir_2d = nir_2d.astype(np.float64)
                            decode_to_ply._buc(nir_2d)
                            decode_to_ply._ae(nir_2d)
                            frame_attrs["near_ir"] = \
                                nir_2d.reshape(-1)[mask].astype(
                                    np.float32)
                    except Exception:
                        pass

                # Surface normals from structured scan
                try:
                    range_raw = _get_field("RANGE")
                    range_2d = range_raw.astype(np.float64) \
                        if range_raw is not None else None
                    xyz_full = xyz  # reuse already-computed result; avoids duplicate SDK call
                    # Estimate normals via cross-product of
                    # adjacent row/col vectors in the 2D grid
                    H, W = xyz_full.shape[:2]
                    if H > 1 and W > 1:
                        normals_3d = np.cross(
                            np.diff(xyz_full, axis=0, append=xyz_full[-1:, :, :]).reshape(-1, 3),
                            np.diff(xyz_full, axis=1, append=xyz_full[:, -1:, :]).reshape(-1, 3))
                        norms = np.linalg.norm(
                            normals_3d, axis=1, keepdims=True)
                        norms[norms == 0] = 1.0
                        normals_3d /= norms
                        # Fix boundary normals (last row/col are
                        # degenerate from diff append of duplicates)
                        n2d = normals_3d.reshape(H, W, 3)
                        n2d[-1, :, :] = n2d[-2, :, :]
                        n2d[:, -1, :] = n2d[:, -2, :]
                        normals_3d = n2d.reshape(-1, 3)
                        frame_attrs["nx"] = \
                            normals_3d[mask, 0].astype(np.float32)
                        frame_attrs["ny"] = \
                            normals_3d[mask, 1].astype(np.float32)
                        frame_attrs["nz"] = \
                            normals_3d[mask, 2].astype(np.float32)
                except Exception:
                    pass

                # Per-frame metadata sidecar (timestamps, IMU,
                # poses) — stored as JSON alongside PLY files
                frame_meta = {}

                # Column timestamps (nanoseconds)
                try:
                    ts = scan.timestamp
                    if ts is not None and len(ts) > 0:
                        frame_meta["timestamps_ns"] = {
                            "first": int(ts[0]),
                            "last": int(ts[-1]),
                            "count": len(ts),
                        }
                except Exception:
                    pass

                # SLAM poses (per-column 4x4 transforms)
                try:
                    poses = scan.pose
                    if poses is not None and len(poses) > 0:
                        # Store first and last pose for the frame
                        frame_meta["poses"] = {
                            "first": poses[0].tolist(),
                            "last": poses[-1].tolist(),
                        }
                except Exception:
                    pass

                # Column status (validity)
                try:
                    status = scan.status
                    if status is not None:
                        valid_cols = int(np.sum(status & 0x01))
                        frame_meta["column_status"] = {
                            "valid": valid_cols,
                            "total": len(status),
                        }
                except Exception:
                    pass

                # IMU data (FW 3.2+ ACCEL32_GYRO32_NMEA profile)
                try:
                    imu_acc = scan.field(core.ChanField.IMU_ACC)
                    imu_gyro = scan.field(core.ChanField.IMU_GYRO)
                    imu_ts = scan.field(core.ChanField.IMU_TIMESTAMP)
                    if imu_acc is not None and len(imu_acc) > 0:
                        frame_meta["imu"] = {
                            "accel": imu_acc.tolist(),
                            "gyro": imu_gyro.tolist(),
                            "timestamps_ns": imu_ts.tolist(),
                            "sample_count": len(imu_acc),
                        }
                except Exception:
                    pass

                if not frame_attrs:
                    frame_attrs = None

            ply_path = os.path.join(output_dir,
                                    f"frame_{frame_count:04d}.ply")
            _write_ply(ply_path, points, attrs=frame_attrs)
            manifest_tracker.update(points, frame_attrs)

            # Write sidecar metadata JSON if present
            if frame_meta:
                meta_path = os.path.join(
                    output_dir,
                    f"frame_{frame_count:04d}.json")
                with open(meta_path, 'w') as mf:
                    json.dump(frame_meta, mf)
            frame_count += 1

            if frame_count == 1:
                attr_names = list(frame_attrs.keys()) \
                    if frame_attrs else []
                log(f"  First frame OK — {len(points)} points"
                    f"{' + ' + ','.join(attr_names) if attr_names else ''}"
                    f" ✓")
            elif frame_count % 100 == 0:
                log(f"  {frame_count} frames decoded...")

            if max_frames > 0 and frame_count >= max_frames:
                log(f"  Reached max_frames limit ({max_frames})")
                break

    except Exception as e:
        log(f"ERROR during decode loop at frame {frame_count}: "
            f"{type(e).__name__}: {e}")
        import traceback
        log(traceback.format_exc())

    log(f"Done. {frame_count} frames total. "
        f"({none_count} None scans skipped)")
    log(f"Log: {log_path}")

    # Extract sensor frame rate from lidar_mode (e.g. "1024x10" → 10)
    sensor_fps = 10  # fallback
    try:
        mode_str = str(info.config.lidar_mode).split(".")[-1]
        sensor_fps = int(mode_str.split("x")[-1])
        log(f"Sensor frame rate: {sensor_fps} Hz (from {mode_str})")
    except Exception:
        log(f"Could not parse sensor frame rate, using {sensor_fps} Hz")

    try:
        _write_sequence_manifest(
            output_dir, manifest_tracker,
            extra={
                "producer": "decode_to_ply",
                "source_fps": sensor_fps,
                "has_attributes": bool(include_attributes),
            })
    except Exception as e:
        log(f"WARNING writing sequence manifest: {e}")

    return output_dir, frame_count, sensor_fps


# ===========================================================================
# Merge two PLY sequences
# ===========================================================================

def merge_ply_sequences(dir_a, dir_b, output_dir,
                        transform_a=None, transform_b=None):
    """
    Merge two PLY sequence directories frame-by-frame.
    Points + float attributes from dir_a and dir_b are concatenated.
    Optional 4x4 transforms applied to XYZ before merging.

    Returns: (output_dir, frame_count)
    """
    os.makedirs(output_dir, exist_ok=True)
    manifest_tracker = _SequenceManifestTracker()
    frames_a = sorted([
        os.path.join(dir_a, f) for f in os.listdir(dir_a)
        if f.startswith("frame_") and f.endswith(".ply")
    ]) if os.path.isdir(dir_a) else []
    frames_b = sorted([
        os.path.join(dir_b, f) for f in os.listdir(dir_b)
        if f.startswith("frame_") and f.endswith(".ply")
    ]) if os.path.isdir(dir_b) else []

    n = min(len(frames_a), len(frames_b))
    if n == 0:
        raise ValueError("One or both PLY directories are empty.")

    for i in range(n):
        pts_a, attrs_a, _ = load_ply_with_attributes(frames_a[i])
        pts_b, attrs_b, _ = load_ply_with_attributes(frames_b[i])

        if transform_a is not None and len(pts_a) > 0:
            pts_a = _apply_transform(pts_a, transform_a)
        if transform_b is not None and len(pts_b) > 0:
            pts_b = _apply_transform(pts_b, transform_b)

        combined_pts = np.vstack([pts_a, pts_b]) \
            if len(pts_a) and len(pts_b) \
            else (pts_a if len(pts_a) else pts_b)

        # Merge attribute columns
        combined_attrs = {}
        all_keys = set(list(attrs_a.keys()) + list(attrs_b.keys()))
        for key in all_keys:
            a = attrs_a.get(key, np.zeros(len(pts_a), dtype=np.float32))
            b = attrs_b.get(key, np.zeros(len(pts_b), dtype=np.float32))
            combined_attrs[key] = np.concatenate([a, b])

        _write_ply(os.path.join(output_dir, f"frame_{i:04d}.ply"),
                   combined_pts,
                   attrs=combined_attrs if combined_attrs else None)
        manifest_tracker.update(
            combined_pts, combined_attrs if combined_attrs else None)

        if i % 100 == 0:
            print(f"[PointMotion] Merge {i:04d}/{n} — "
                  f"{len(combined_pts)} pts")

    try:
        _write_sequence_manifest(
            output_dir, manifest_tracker,
            extra={
                "producer": "merge_ply_sequences",
                "source_frame_count": int(n),
                "source_dirs": [
                    os.path.basename(os.path.normpath(dir_a)),
                    os.path.basename(os.path.normpath(dir_b)),
                ],
            })
    except Exception as e:
        print(f"[PointMotion] WARNING writing sequence manifest: {e}")

    print(f"[PointMotion] Merge complete: {n} frames → {output_dir}")
    return output_dir, n


# ===========================================================================
# Ego-motion + classification helpers for Structured Interpolation v2
# ===========================================================================









# ===========================================================================
# Structured grid decode  (PCAP/OSF → .npz grids, preserving H×W layout)
# ===========================================================================

def _decode_to_grids(source_path, sensor_jsons, sensor_idx, output_dir,
                     include_attributes=True, max_frames=0):
    """
    Decode PCAP/OSF to structured grids (.npz) preserving the (H, W)
    beam layout. Used for structured interpolation where beam[r,c]
    correspondence between frames is needed.

    Each .npz contains:
      xyz:    (H, W, 3) float32
      range:  (H, W) float32
      refl:   (H, W) float32
      signal: (H, W) float32
      near_ir:(H, W) float32
      flags:  (H, W) uint8

    Returns: (output_dir, frame_count, sensor_fps, grid_shape)
    """
    open_source, core = get_ouster_sdk()
    if open_source is None:
        raise ImportError("ouster-sdk not available.")

    os.makedirs(output_dir, exist_ok=True)

    is_osf = source_path.lower().endswith(".osf")
    if is_osf or not sensor_jsons:
        source = open_source(source_path)
    else:
        source = open_source(source_path, meta=sensor_jsons)

    all_info = source.sensor_info
    resolved_idx = min(sensor_idx, len(all_info) - 1)

    if sensor_jsons and sensor_idx < len(sensor_jsons):
        target_info = read_sensor_info(sensor_jsons[sensor_idx])
        target_sn = str(target_info.sn) if target_info else None
        if target_sn:
            for i, si in enumerate(all_info):
                if str(si.sn) == target_sn:
                    resolved_idx = i
                    break

    info = all_info[resolved_idx]
    xyz_lut = core.XYZLut(info)

    frame_count = 0
    grid_shape = None

    try:
        source_iter = iter(source)
        for item in source_iter:
            scan = None
            if hasattr(item, 'field'):
                scan = item
            elif hasattr(item, '__getitem__'):
                try:
                    scan = item[resolved_idx]
                except (IndexError, KeyError):
                    for sub in item:
                        if hasattr(sub, 'field'):
                            scan = sub
                            break
            elif hasattr(item, 'field') or hasattr(item, 'astype'):
                scan = item

            if scan is None:
                continue

            xyz_2d = xyz_lut(scan)  # (H, W, 3)
            H, W = xyz_2d.shape[:2]
            if grid_shape is None:
                grid_shape = (H, W)
                print(f"[PointMotion] Grid shape: {H}x{W}")

            grid_data = {
                'xyz': xyz_2d.astype(np.float32),
            }

            try:
                grid_data['range'] = scan.field(
                    core.ChanField.RANGE).astype(np.float32)
            except Exception:
                grid_data['range'] = np.zeros((H, W), dtype=np.float32)

            try:
                grid_data['refl'] = scan.field(
                    core.ChanField.REFLECTIVITY).astype(np.float32)
            except Exception:
                grid_data['refl'] = np.zeros((H, W), dtype=np.float32)

            try:
                grid_data['signal'] = scan.field(
                    core.ChanField.SIGNAL).astype(np.float32)
            except Exception:
                grid_data['signal'] = np.zeros((H, W), dtype=np.float32)

            try:
                grid_data['near_ir'] = scan.field(
                    core.ChanField.NEAR_IR).astype(np.float32)
            except Exception:
                grid_data['near_ir'] = np.zeros((H, W), dtype=np.float32)

            try:
                grid_data['flags'] = scan.field(
                    core.ChanField.FLAGS).astype(np.uint8)
            except Exception:
                grid_data['flags'] = np.zeros((H, W), dtype=np.uint8)

            np.savez_compressed(
                os.path.join(output_dir, f"grid_{frame_count:04d}.npz"),
                **grid_data)

            frame_count += 1
            if frame_count == 1:
                print(f"[PointMotion] First grid saved")
            elif frame_count % 100 == 0:
                print(f"[PointMotion] {frame_count} grids decoded...")

            if max_frames > 0 and frame_count >= max_frames:
                break

    except Exception as e:
        print(f"[PointMotion] Grid decode error: {e}")
        import traceback
        traceback.print_exc()

    # Extract sensor fps
    sensor_fps = 10
    try:
        mode_str = str(info.config.lidar_mode).split(".")[-1]
        sensor_fps = int(mode_str.split("x")[-1])
    except Exception:
        pass

    print(f"[PointMotion] Grid decode complete: {frame_count} grids "
          f"@ {sensor_fps} Hz")
    return output_dir, frame_count, sensor_fps, grid_shape

# ===========================================================================
# PLY I/O  (binary little-endian, with optional float attributes)
# ===========================================================================

# Attribute columns written/read alongside x,y,z
_ATTR_COLS = ("signal", "reflectivity", "near_ir", "range",
              "signal2", "reflectivity2", "range2",
              "nx", "ny", "nz", "window", "confidence",
              "freeze_frame", "freeze_resolution",
              "native_density", "vertex_id")

# ---------------------------------------------------------------------------
# Colormap presets  (position, R, G, B)
# ---------------------------------------------------------------------------
_COLORMAPS = {
    "VIRIDIS": [
        (0.00, 0.267, 0.004, 0.329),
        (0.25, 0.282, 0.140, 0.458),
        (0.50, 0.127, 0.566, 0.551),
        (0.75, 0.544, 0.774, 0.247),
        (1.00, 0.993, 0.906, 0.144),
    ],
    "MAGMA": [
        (0.00, 0.001, 0.000, 0.014),
        (0.25, 0.271, 0.051, 0.431),
        (0.50, 0.716, 0.215, 0.475),
        (0.75, 0.994, 0.541, 0.345),
        (1.00, 0.987, 0.991, 0.750),
    ],
    "GREYSCALE": [
        (0.00, 0.015, 0.015, 0.015),
        (1.00, 1.000, 1.000, 1.000),
    ],
    "OUSTER": [
        (0.00, 0.050, 0.000, 0.150),
        (0.25, 0.100, 0.200, 0.600),
        (0.50, 0.000, 0.600, 0.600),
        (0.75, 0.200, 0.800, 0.200),
        (1.00, 1.000, 1.000, 0.000),
    ],
    "CALREF": [
        (0.00, 0.000, 0.000, 0.000),
        (0.25, 0.200, 0.000, 0.400),
        (0.50, 0.000, 0.400, 0.800),
        (0.75, 0.000, 0.800, 0.400),
        (1.00, 1.000, 1.000, 1.000),
    ],
}

_PLY_FLOAT_TYPES = {"float", "float32"}


def _write_ply(filepath, points, attrs=None, colors=None):
    """
    Write binary little-endian PLY with XYZ + optional float attributes.
    points: Nx3 float32
    attrs:  dict { "signal": N-array, ... } or None
    colors: Nx3 uint8 (legacy compat, unused in binary path) or None
    """
    n = len(points)
    # Determine which attribute columns to write
    attr_cols = []
    if attrs is not None:
        for col in _ATTR_COLS:
            if col in attrs and len(attrs[col]) == n:
                attr_cols.append(col)

    # Build header (ASCII)
    header_lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
    ]
    for col in attr_cols:
        header_lines.append(f"property float {col}")
    header_lines.append("end_header")
    header = "\n".join(header_lines) + "\n"

    # Build data array: x y z [signal reflectivity near_ir]
    global _write_buffer
    n_cols = 3 + len(attr_cols)
    if _write_buffer is None or _write_buffer.shape[0] != n or _write_buffer.shape[1] != n_cols:
        _write_buffer = np.empty((n, n_cols), dtype=np.float32)
    _write_buffer[:, :3] = points
    for i, col in enumerate(attr_cols):
        _write_buffer[:, 3 + i] = attrs[col]
    data = _write_buffer[:n]

    # Write: ASCII header + binary data
    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        data.tofile(f)


def load_ply_points(filepath):
    """Load XYZ only (backwards compatible)."""
    pts, _, _ = _load_ply_full(filepath, want_attrs=False)
    return pts


def load_ply_with_attributes(filepath):
    """
    Load XYZ + named float attributes.
    Returns: (Nx3 float32, dict {name: N-array}, None)
    """
    return _load_ply_full(filepath, want_attrs=True)


def _manifest_schema_for_ply_dir(ply_dir):
    """Return the canonical Point Motion schema for a sequence folder."""
    manifest = _load_sequence_manifest(ply_dir)
    if not manifest or manifest.get("layout") != _PM_MANIFEST_LAYOUT:
        return None
    schema = manifest.get("schema")
    if not isinstance(schema, list):
        return None
    result = []
    seen = set()
    for name in schema:
        name = str(name).lower()
        if name in _ATTR_COLS and name not in seen:
            result.append(name)
            seen.add(name)
    return result


def _point_motion_schema_from_properties(properties, manifest_schema=None):
    """Detect the strict Point Motion float32 property layout."""
    if len(properties) < 3:
        return None

    prop_names = [name for name, _dtype in properties]
    if prop_names[:3] != ["x", "y", "z"]:
        return None

    if any(dtype not in _PLY_FLOAT_TYPES for _name, dtype in properties):
        return None

    attr_names = prop_names[3:]
    expected = [col for col in _ATTR_COLS if col in attr_names]
    if attr_names != expected:
        return None

    if manifest_schema is not None and attr_names != manifest_schema:
        return None

    return attr_names


def _load_ply_full(filepath, want_attrs=True):
    """
    Parse PLY — auto-detects binary_little_endian or ascii format.
    Returns: (points Nx3, attrs dict, colors Nx3 or None)
    """
    properties   = []
    vertex_count = 0
    is_binary    = False
    header_bytes = 0

    _MAX_VERTICES = 50_000_000   # safety cap for PLY loading

    with open(filepath, 'rb') as f:
        header_line_count = 0
        while True:
            line = f.readline()
            header_line_count += 1
            if not line or header_line_count > 500:
                break
            text = line.decode('ascii', errors='ignore').strip()

            if text.startswith("format binary_little_endian"):
                is_binary = True
            elif text.startswith("element vertex"):
                try:
                    vertex_count = min(int(text.split()[-1]),
                                       _MAX_VERTICES)
                except (ValueError, IndexError):
                    vertex_count = 0
            elif text.startswith("property"):
                parts = text.split()
                if len(parts) >= 3:
                    properties.append((parts[-1].lower(),
                                       parts[1].lower()))
            elif text == "end_header":
                header_bytes = f.tell()
                break

        if header_bytes == 0:
            print(f"[PointMotion] WARNING: end_header not found in "
                  f"{filepath}")
            return (np.zeros((0, 3), dtype=np.float32), {}, None)

        if vertex_count == 0 or not properties:
            return (np.zeros((0, 3), dtype=np.float32), {}, None)

        n_props = len(properties)
        manifest_schema = _manifest_schema_for_ply_dir(
            os.path.dirname(os.path.abspath(filepath)))
        point_motion_schema = _point_motion_schema_from_properties(
            properties, manifest_schema)

        # ── Read data ─────────────────────────────────────────────────
        if is_binary:
            raw_data = np.fromfile(
                f, dtype='<f4', count=vertex_count * n_props)
            if len(raw_data) < vertex_count * n_props:
                # Truncated file — use what we have
                vertex_count = len(raw_data) // n_props
                raw_data = raw_data[:vertex_count * n_props]
            raw_data = raw_data.reshape(vertex_count, n_props)
        else:
            # ASCII fallback for older PLY files
            raw_data = np.zeros((vertex_count, n_props), dtype=np.float32)
            parsed = 0
            for line in f:
                if parsed >= vertex_count:
                    break
                vals = line.split()
                for j in range(min(len(vals), n_props)):
                    raw_data[parsed, j] = float(vals[j])
                parsed += 1
            raw_data = raw_data[:parsed]
            vertex_count = parsed

    if vertex_count == 0:
        return (np.zeros((0, 3), dtype=np.float32), {}, None)

    points = raw_data[:, :3]
    if not want_attrs:
        return points, {}, None

    if is_binary and point_motion_schema is not None:
        attrs = {
            name: raw_data[:, j + 3]
            for j, name in enumerate(point_motion_schema)
        }
        return points, attrs, None

    # ── Extract columns ───────────────────────────────────────────────
    attrs = {}
    color_r = color_g = color_b = None
    for j, (name, dtype) in enumerate(properties):
        if j < 3:
            continue  # skip x, y, z
        if name in _ATTR_COLS:
            attrs[name] = raw_data[:, j]
        elif name == "red":
            color_r = raw_data[:, j]
        elif name == "green":
            color_g = raw_data[:, j]
        elif name == "blue":
            color_b = raw_data[:, j]

    colors = None
    if color_r is not None and color_g is not None \
            and color_b is not None:
        colors = np.stack([color_r, color_g, color_b], axis=1) \
            .astype(np.uint8)

    return points, attrs, colors


def load_ply_as_mesh_verts(filepath):
    return [tuple(p) for p in load_ply_points(filepath)]


# ===========================================================================
# Geometry nodes
# ===========================================================================

def apply_point_cloud_geometry_nodes(obj, has_color=False):
    mod = obj.modifiers.new(name="PointCloud", type='NODES')
    ng  = bpy.data.node_groups.new(name="LiDAR_PointCloud",
                                   type='GeometryNodeTree')
    mod.node_group = ng
    nodes = ng.nodes
    links = ng.links
    gi  = nodes.new('NodeGroupInput')
    go  = nodes.new('NodeGroupOutput')
    ng.interface.new_socket('Geometry', in_out='INPUT',
                            socket_type='NodeSocketGeometry')
    ng.interface.new_socket('Geometry', in_out='OUTPUT',
                            socket_type='NodeSocketGeometry')
    m2p = nodes.new('GeometryNodeMeshToPoints')
    m2p.inputs['Radius'].default_value = 0.002  # native beam size

    gi.location   = (-400, 0)
    m2p.location  = (-100, 0)

    if has_color:
        # Set Material node — auto-assigns PM_LiDAR_Viz
        setmat = nodes.new('GeometryNodeSetMaterial')
        setmat.location = (200, 0)
        go.location     = (500, 0)
        # Find or create the material
        mat = bpy.data.materials.get("PM_LiDAR_Viz")
        if mat is not None:
            setmat.inputs['Material'].default_value = mat

        links.new(gi.outputs['Geometry'],    m2p.inputs['Mesh'])
        links.new(m2p.outputs['Points'],     setmat.inputs['Geometry'])
        links.new(setmat.outputs['Geometry'], go.inputs['Geometry'])
    else:
        go.location = (200, 0)
        links.new(gi.outputs['Geometry'], m2p.inputs['Mesh'])
        links.new(m2p.outputs['Points'],  go.inputs['Geometry'])

# ===========================================================================
# Sequence registry + persistence
# ===========================================================================

_sequence_registry = {}


def _register_sequence(obj, ply_dir, total_frames, start_frame,
                       has_color=False):
    # Runtime registry uses absolute path
    _sequence_registry[obj.name] = {
        "ply_dir": ply_dir, "total": total_frames,
        "start_frame": start_frame, "has_color": has_color,
    }
    # Persistent property: store relative to .blend if possible
    # Blender's // convention means "relative to .blend file"
    if bpy.data.filepath:
        try:
            rel = bpy.path.relpath(ply_dir)
            obj["pm_ply_dir"] = rel
        except ValueError:
            # Different drive on Windows — can't make relative
            obj["pm_ply_dir"] = ply_dir
    else:
        # .blend not saved yet — store absolute, will be
        # converted on next save+load cycle
        obj["pm_ply_dir"] = ply_dir
    obj["pm_total_frames"] = total_frames
    obj["pm_start_frame"]  = start_frame
    obj["pm_has_color"]    = has_color


def _restore_registry_from_scene():
    _sequence_registry.clear()

    for obj in bpy.data.objects:
        if "pm_ply_dir" not in obj or "pm_total_frames" not in obj:
            continue

        stored_path = obj["pm_ply_dir"]

        # Resolve: bpy.path.abspath handles // relative paths,
        # and returns absolute paths unchanged
        ply_dir = bpy.path.abspath(stored_path)

        # Fallback: if resolved path doesn't exist, try searching
        if not os.path.exists(ply_dir):
            basename = os.path.basename(ply_dir.rstrip("/\\"))
            blend_dir = os.path.dirname(bpy.data.filepath) \
                if bpy.data.filepath else None

            found = False
            if blend_dir:
                # Check next to .blend
                candidate = os.path.join(blend_dir, basename)
                if os.path.isdir(candidate):
                    ply_dir = candidate
                    found = True
                else:
                    # Check one level up
                    parent = os.path.dirname(blend_dir)
                    candidate = os.path.join(parent, basename)
                    if os.path.isdir(candidate):
                        ply_dir = candidate
                        found = True

            if found:
                # Update stored path to new relative
                try:
                    obj["pm_ply_dir"] = bpy.path.relpath(ply_dir)
                except ValueError:
                    obj["pm_ply_dir"] = ply_dir
                print(f"[PointMotion] Path re-linked: {basename}")

        if os.path.exists(ply_dir):
            _sequence_registry[obj.name] = {
                "ply_dir":     ply_dir,
                "total":       obj["pm_total_frames"],
                "start_frame": obj.get("pm_start_frame", 1),
                "has_color":   obj.get("pm_has_color", False),
            }
            print(f"[PointMotion] Restored: {obj.name} "
                  f"(color: {obj.get('pm_has_color', False)})")
        else:
            print(f"[PointMotion] WARNING: PLY folder not found "
                  f"— {stored_path}")


# ===========================================================================
# Optimized mesh loading + frame cache
# ===========================================================================

_frame_cache = OrderedDict()
_frame_cache_bytes = 0
_sequence_ranges = {}
_CACHE_MAX_BYTES = 2 * 1024 * 1024 * 1024
_CACHE_MAX_RENDER_BYTES = 1024 * 1024 * 1024


def _clear_frame_cache():
    """Clear frame and sequence caches and reset byte accounting."""
    global _frame_cache_bytes
    _frame_cache.clear()
    _sequence_ranges.clear()
    _sequence_manifest_cache.clear()
    _frame_cache_bytes = 0


def _estimate_frame_data_bytes(data):
    """Estimate the memory footprint of cached frame data."""
    pts, attrs = data
    total = pts.nbytes
    if attrs:
        total += sum(arr.nbytes for arr in attrs.values())
    return total


def _cache_evict():
    """Remove oldest entries until cache fits within the byte budget."""
    global _frame_cache_bytes
    is_rendering = hasattr(bpy.app, 'is_job_running') and \
        bpy.app.is_job_running('RENDER')
    limit = (_CACHE_MAX_RENDER_BYTES if is_rendering
             else _CACHE_MAX_BYTES)
    while _frame_cache and _frame_cache_bytes > limit:
        _, evicted = _frame_cache.popitem(last=False)
        _frame_cache_bytes -= _estimate_frame_data_bytes(evicted)


def _resolve_sequence_total_frames(ply_dir):
    """Count frame_*.ply files when sequence metadata is unavailable."""
    manifest = _load_sequence_manifest(ply_dir)
    if manifest:
        try:
            count = int(manifest.get("frame_count", 0))
            if count > 0:
                return count
        except (TypeError, ValueError):
            pass
    if not os.path.isdir(ply_dir):
        return 0
    return sum(
        1 for name in os.listdir(ply_dir)
        if name.startswith("frame_") and name.endswith(".ply"))


def _compute_attr_display_range(name, finite_values):
    """Compute a stable 0-1 display range from sampled attribute values."""
    vmin = 0.0
    if len(finite_values) == 0:
        return (vmin, 1.0)
    vmax = float(np.percentile(finite_values, 99.5))
    if vmax <= vmin:
        vmax = float(np.max(finite_values))
    if vmax <= vmin:
        vmax = vmin + 1.0
    return (vmin, vmax)




def _get_sequence_attr_ranges(ply_dir, total_frames=0, auto_levels=True):
    """Compute fixed sampled ranges for a sequence and cache them by folder."""
    cached = _sequence_ranges.get(ply_dir)
    if cached is None:
        cached = {}
        manifest = _load_sequence_manifest(ply_dir)
        if manifest:
            ranges = manifest.get("attr_ranges", {})
            if isinstance(ranges, dict):
                for name, bounds in ranges.items():
                    if not isinstance(bounds, (list, tuple)) or \
                            len(bounds) != 2:
                        continue
                    try:
                        vmin = float(bounds[0])
                        vmax = float(bounds[1])
                    except (TypeError, ValueError):
                        continue
                    if vmax <= vmin:
                        vmax = vmin + 1.0
                    cached[name] = (vmin, vmax)

        if not cached:
            total = (int(total_frames) if total_frames
                     else _resolve_sequence_total_frames(ply_dir))
            sample_count = max(1, min(total, 64))
            if sample_count == 1:
                indices = [0] if total > 0 else []
            else:
                indices = np.linspace(
                    0, total - 1, sample_count, dtype=np.int64).tolist()
                indices = sorted(set(int(idx) for idx in indices))
            sampled = {}
            for idx in indices:
                fp = os.path.join(ply_dir, f"frame_{idx:04d}.ply")
                if not os.path.exists(fp):
                    continue
                try:
                    _, attrs, _ = load_ply_with_attributes(fp)
                except Exception:
                    continue
                for name, values in attrs.items():
                    if name in ("nx", "ny", "nz"):
                        continue
                    finite = values[np.isfinite(values)]
                    if len(finite) == 0:
                        continue
                    sampled.setdefault(name, []).append(
                        finite.astype(np.float32, copy=False))
            for name, chunks in sampled.items():
                finite_values = (np.concatenate(chunks)
                                 if len(chunks) > 1 else chunks[0])
                cached[name] = _compute_attr_display_range(
                    name, finite_values)
        _sequence_ranges[ply_dir] = cached

    if auto_levels:
        return cached

    fixed = {}
    for name, value_range in cached.items():
        fixed[name] = _KNOWN_RANGES.get(name, value_range)
    return fixed


def _get_frame_data(ply_dir, idx, has_color):
    """Load frame data with caching. Returns (pts, attrs_or_None)."""
    global _frame_cache_bytes
    key = (ply_dir, idx, has_color)
    if key in _frame_cache:
        _frame_cache.move_to_end(key)  # LRU touch
        return _frame_cache[key]

    fp = os.path.join(ply_dir, f"frame_{idx:04d}.ply")
    if not os.path.exists(fp):
        return None

    if has_color:
        pts, attrs, _ = load_ply_with_attributes(fp)
        data = (pts, attrs)
    else:
        pts = load_ply_points(fp)
        data = (pts, None)

    _frame_cache[key] = data
    _frame_cache_bytes += _estimate_frame_data_bytes(data)
    _cache_evict()
    return data


def _load_frame_into_mesh(mesh, filepath_or_data, has_color=False,
                          ply_dir=None, total_frames=0, scene=None):
    """
    Fast mesh population from PLY data or filepath.
    Uses vertices.add + foreach_set instead of slow from_pydata.
    """
    scene = scene or bpy.context.scene
    if isinstance(filepath_or_data, str):
        # filepath
        if has_color:
            pts, attrs, _ = load_ply_with_attributes(filepath_or_data)
        else:
            pts = load_ply_points(filepath_or_data)
            attrs = None
    else:
        # (pts, attrs) tuple from cache
        pts, attrs = filepath_or_data

    n = len(pts)
    if n == 0:
        return

    # Fast vertex creation — ~3-5x faster than from_pydata for large N
    mesh.vertices.add(n)
    mesh.vertices.foreach_set("co", pts.ravel())
    mesh.update()

    if has_color and attrs:
        seq_dir = ply_dir or (
            os.path.dirname(filepath_or_data)
            if isinstance(filepath_or_data, str) else "")
        attr_ranges = _get_sequence_attr_ranges(
            seq_dir, total_frames, getattr(scene, "pm_auto_levels", True))
        _apply_float_attributes(
            mesh, attrs,
            attr_ranges=attr_ranges,
            active_channel=getattr(scene, "pm_active_channel",
                                   "reflectivity"),
            push_all=getattr(scene, "pm_push_all_attributes", False))


# ===========================================================================
# Frame change handler
# ===========================================================================



def _on_frame_change(scene, depsgraph=None):
    cf = scene.frame_current
    for obj_name, seq in list(_sequence_registry.items()):
        obj = bpy.data.objects.get(obj_name)
        if obj is None:
            _sequence_registry.pop(obj_name, None)
            continue
        idx = cf - seq["start_frame"]
        if idx < 0 or idx >= seq["total"]:
            obj.data.clear_geometry()
            obj.update_tag()
            continue

        has_color = seq.get("has_color", False)
        data = _get_frame_data(seq["ply_dir"], idx, has_color)
        if data is None:
            continue

        pts, attrs = data
        mesh = obj.data
        n_new = len(pts)
        n_old = len(mesh.vertices)
        attr_ranges = None
        if has_color and attrs:
            attr_ranges = _get_sequence_attr_ranges(
                seq["ply_dir"], seq.get("total", 0),
                getattr(scene, "pm_auto_levels", True))
        active_channel = getattr(scene, "pm_active_channel", "reflectivity")
        push_all = getattr(scene, "pm_push_all_attributes", False)

        if n_new == n_old and n_old > 0:
            # Fast path: same vertex count — just overwrite positions
            mesh.vertices.foreach_set("co", pts.ravel())
            mesh.update()
            if has_color and attrs:
                _apply_float_attributes(
                    mesh, attrs, reuse=True,
                    attr_ranges=attr_ranges,
                    active_channel=active_channel,
                    push_all=push_all)
        else:
            # Full rebuild: different vertex count
            mesh.clear_geometry()
            if n_new > 0:
                mesh.vertices.add(n_new)
                mesh.vertices.foreach_set("co", pts.ravel())
                mesh.update()
                if has_color and attrs:
                    _apply_float_attributes(
                        mesh, attrs, reuse=False,
                        attr_ranges=attr_ranges,
                        active_channel=active_channel,
                        push_all=push_all)

        # Force depsgraph to re-evaluate modifier stack (GeoNodes,
        # PM FX, etc.) before the render samples this frame.
        # Without this, modifiers may evaluate on stale mesh data
        # during bpy.ops.render.render(animation=True).
        obj.update_tag()

        # Periodic GC during render to prevent memory accumulation
        if hasattr(bpy.app, 'is_job_running') and \
                bpy.app.is_job_running('RENDER'):
            if cf % 50 == 0:
                import gc
                gc.collect()


def _prefetch_sequence_frame(scene, frame):
    """Prime the cache for the next render frame without touching Blender data."""
    for seq in _sequence_registry.values():
        idx = frame - seq["start_frame"]
        if 0 <= idx < seq["total"]:
            _get_frame_data(seq["ply_dir"], idx, seq.get("has_color", False))


def _prefetch_sequence_window(scene, frames):
    """Prime a small forward render window into the frame cache."""
    for frame in frames:
        _prefetch_sequence_frame(scene, frame)


_rgba_buffer = None   # pre-allocated RGBA buffer, reused across frames
_normals_buffer = None  # pre-allocated normal buffer, reused across frames
_write_buffer = None  # pre-allocated write buffer for _write_ply, reused across frames

_KNOWN_RANGES = {
    "reflectivity": (0.0, 255.0),
    "signal": (0.0, 10000.0),
    "near_ir": (0.0, 10000.0),
    "range": (0.0, 100000.0),
    "confidence": (0.0, 1.0),
    "window": (0.0, 255.0),
}


def _apply_float_attributes(mesh, attrs, reuse=False, attr_ranges=None,
                            active_channel="reflectivity",
                            push_all=False):
    """
    Set per-vertex float attributes on a faceless mesh.
    attrs: dict { "signal": N-float-array, "reflectivity": ..., ... }

    Each field becomes a POINT-domain FLOAT_COLOR attribute named
    e.g. "signal" stored as (value, value, value, 1.0) greyscale RGBA.

    reuse=True: assume existing attributes have the right size,
    just overwrite data (avoids alloc/dealloc cycle during render).
    """
    global _rgba_buffer, _normals_buffer
    n = len(mesh.vertices)
    if n == 0:
        return

    # Reuse or allocate RGBA buffer
    if _rgba_buffer is None or _rgba_buffer.shape[0] != n:
        _rgba_buffer = np.empty((n, 4), dtype=np.float32)
    if _normals_buffer is None or _normals_buffer.shape[0] != n:
        _normals_buffer = np.empty((n, 3), dtype=np.float32)

    # Extract normal components (use .get to avoid mutating cache)
    nx = attrs.get("nx", None)
    ny = attrs.get("ny", None)
    nz = attrs.get("nz", None)
    has_normals = (nx is not None and ny is not None and nz is not None
                   and len(nx) == n)

    if push_all:
        color_names = [
            name for name, values in attrs.items()
            if name not in ("nx", "ny", "nz") and len(values) == n]
    else:
        color_names = []
        for name in (active_channel, "reflectivity"):
            values = attrs.get(name)
            if values is None or len(values) != n or name in color_names:
                continue
            color_names.append(name)

    if not push_all:
        needed = set(color_names)
        for attr in list(mesh.color_attributes):
            if attr.name not in needed:
                mesh.color_attributes.remove(attr)
        lidar_normal = mesh.attributes.get("lidar_normal")
        if lidar_normal is not None and not has_normals:
            mesh.attributes.remove(lidar_normal)

    for name in color_names:
        values = attrs[name]
        if attr_ranges is not None and name in attr_ranges:
            vmin, vmax = attr_ranges[name]
        else:
            finite = values[np.isfinite(values)]
            vmin, vmax = _compute_attr_display_range(name, finite)
        values32 = values.astype(np.float32, copy=False)
        denom = float(vmax) - float(vmin)
        if (not np.isfinite(denom)) or denom <= 0.0:
            _rgba_buffer[:, 0] = 0.5
        else:
            np.subtract(values32, vmin, out=_rgba_buffer[:, 0])
            _rgba_buffer[:, 0] /= denom
            np.clip(_rgba_buffer[:, 0], 0.0, 1.0, out=_rgba_buffer[:, 0])

        # Get or create attribute
        attr = mesh.color_attributes.get(name)
        if reuse and attr is not None and len(attr.data) == n:
            # Fast path: reuse existing attribute object
            pass
        else:
            # Need to create/recreate
            if attr is not None:
                mesh.color_attributes.remove(attr)
            attr = mesh.color_attributes.new(
                name=name, type='FLOAT_COLOR', domain='POINT')

        # Fill pre-allocated buffer
        _rgba_buffer[:, 1] = _rgba_buffer[:, 0]   # G
        _rgba_buffer[:, 2] = _rgba_buffer[:, 0]   # B
        _rgba_buffer[:, 3] = 1.0    # A
        attr.data.foreach_set("color", _rgba_buffer.ravel())

    # Apply normals as a FLOAT_VECTOR attribute (not color)
    if has_normals:
        _normals_buffer[:, 0] = nx.astype(np.float32, copy=False)
        _normals_buffer[:, 1] = ny.astype(np.float32, copy=False)
        _normals_buffer[:, 2] = nz.astype(np.float32, copy=False)
        attr = mesh.attributes.get("lidar_normal")
        if not (reuse and attr is not None and len(attr.data) == n):
            if attr is not None:
                mesh.attributes.remove(attr)
            attr = mesh.attributes.new(
                name="lidar_normal", type='FLOAT_VECTOR',
                domain='POINT')
        attr.data.foreach_set("vector", _normals_buffer.ravel())


def _apply_colormap_to_ramp(color_ramp, cmap_key):
    """Apply a colormap preset to a Blender ColorRamp."""
    stops = _COLORMAPS.get(cmap_key)
    if stops is None:
        return
    cr = color_ramp
    # Remove extra stops (Blender requires min 2)
    while len(cr.elements) > 2:
        cr.elements.remove(cr.elements[-1])
    for i, (pos, r, g, b) in enumerate(stops):
        if i < len(cr.elements):
            cr.elements[i].position = pos
            cr.elements[i].color = (r, g, b, 1.0)
        else:
            el = cr.elements.new(pos)
            el.color = (r, g, b, 1.0)


def _on_viz_setting_changed(self, context):
    """Update PM_LiDAR_Viz material when channel/colormap/auto-levels change."""
    mat = bpy.data.materials.get("PM_LiDAR_Viz")
    if mat is None or mat.node_tree is None:
        return

    channel  = context.scene.pm_active_channel
    cmap_key = context.scene.pm_colormap

    for node in mat.node_tree.nodes:
        if node.type == 'ATTRIBUTE' and node.label == "LiDAR Channel":
            node.attribute_name = channel
        elif node.type == 'VALTORGB':
            _apply_colormap_to_ramp(node.color_ramp, cmap_key)

    try:
        _on_frame_change(context.scene)
    except Exception:
        pass


def _ensure_lidar_material(obj, default_attr="reflectivity"):
    """
    Create / assign a material for LiDAR attribute visualization.

    Pure Principled BSDF — no emission.  Points behave like real
    surfaces: fully responsive to scene lights, HDRI, shadows.

    Attribute → ColorRamp → Base Color (Principled BSDF)

    Always recreates the material to pick up shader changes
    between add-on versions.  User edits to the ramp will be
    lost on re-import — rename the material to keep edits.
    """
    mat_name = "PM_LiDAR_Viz"
    _MAT_VERSION = 9    # bump this when shader setup changes

    mat = bpy.data.materials.get(mat_name)

    # Check if material needs rebuilding
    # Blender 5.0: mat.use_nodes deprecated (always True)
    needs_rebuild = (mat is None
                     or mat.node_tree is None
                     or mat.get("pm_mat_version", 0) < _MAT_VERSION)

    if needs_rebuild:
        # Remove old version if it exists
        if mat is not None:
            bpy.data.materials.remove(mat, do_unlink=True)

        mat = bpy.data.materials.new(name=mat_name)
        # Blender 5.0: use_nodes deprecated, node tree created
        # automatically. Keep for 4.x compat (no-op on 5.0).
        try:
            mat.use_nodes = True
        except Exception:
            pass
        mat["pm_mat_version"] = _MAT_VERSION
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links
        nodes.clear()

        # ── Read scene settings (fall back to defaults) ──────────
        scene = bpy.context.scene
        channel  = getattr(scene, "pm_active_channel", None) \
            or default_attr
        cmap_key = getattr(scene, "pm_colormap", None) or "VIRIDIS"

        # ── Attribute node  ───────────────────────────────────────
        attr_node = nodes.new('ShaderNodeAttribute')
        attr_node.location = (-400, 0)
        attr_node.attribute_name = channel
        attr_node.attribute_type = 'GEOMETRY'
        attr_node.label = "LiDAR Channel"

        # ── ColorRamp — preset palette ────────────────────────────
        ramp = nodes.new('ShaderNodeValToRGB')
        ramp.location = (-100, 0)
        ramp.label = "LiDAR Palette"
        _apply_colormap_to_ramp(ramp.color_ramp, cmap_key)

        # ── Reflectivity → physical material properties ──────────
        #    High reflectivity = shiny (low roughness, high specular)
        #    Low  reflectivity = matte (high roughness, low specular)
        refl_node = nodes.new('ShaderNodeAttribute')
        refl_node.location = (-400, -250)
        refl_node.attribute_name = "reflectivity"
        refl_node.attribute_type = 'GEOMETRY'
        refl_node.label = "Reflectivity (physical)"

        # Invert reflectivity for roughness: high refl → low roughness
        invert = nodes.new('ShaderNodeMath')
        invert.location = (-100, -250)
        invert.operation = 'SUBTRACT'
        invert.inputs[0].default_value = 1.0
        invert.label = "Invert → Roughness"

        # Scale specular: reflectivity × 0.5 → specular range
        spec_scale = nodes.new('ShaderNodeMath')
        spec_scale.location = (-100, -400)
        spec_scale.operation = 'MULTIPLY'
        spec_scale.inputs[1].default_value = 0.5
        spec_scale.label = "Scale → Specular"

        # ── Surface normals from LiDAR ─────────────────────────────
        normal_node = nodes.new('ShaderNodeAttribute')
        normal_node.location = (-400, -550)
        normal_node.attribute_name = "lidar_normal"
        normal_node.attribute_type = 'GEOMETRY'
        normal_node.label = "LiDAR Normals"

        # ── Principled BSDF — physically driven by reflectivity ──
        bsdf = nodes.new('ShaderNodeBsdfPrincipled')
        bsdf.location = (250, 0)
        bsdf.inputs['Emission Strength'].default_value = 0.0

        # ── Output ────────────────────────────────────────────────
        out = nodes.new('ShaderNodeOutputMaterial')
        out.location = (550, 0)

        # ── Links ─────────────────────────────────────────────────
        # Color channel → ColorRamp → Base Color + Emission Color
        links.new(attr_node.outputs['Fac'], ramp.inputs['Fac'])
        links.new(ramp.outputs['Color'], bsdf.inputs['Base Color'])
        links.new(ramp.outputs['Color'], bsdf.inputs['Emission Color'])

        # Reflectivity → inverted → Roughness
        links.new(refl_node.outputs['Fac'], invert.inputs[1])
        links.new(invert.outputs['Value'], bsdf.inputs['Roughness'])

        # Reflectivity → scaled → Specular IOR Level
        links.new(refl_node.outputs['Fac'], spec_scale.inputs[0])
        links.new(spec_scale.outputs['Value'],
                  bsdf.inputs['Specular IOR Level'])

        # Normal: NOT connected. Let Cycles use default sphere
        # normals. Works for both SLAM (no lidar_normal) and
        # non-SLAM (has lidar_normal but sphere normals are fine).
        # The lidar_normal attribute node is kept for reference
        # but not linked to BSDF.

        links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])

    # Always assign to this object
    obj.data.materials.clear()
    obj.data.materials.append(mat)


@persistent
def _on_load_post(filepath):
    _clear_frame_cache()
    _restore_registry_from_scene()
    _register_frame_handler()

    # Check for crash recovery marker from interrupted interpolated render
    if bpy.data.filepath:
        marker_path = bpy.data.filepath + ".pm_interp_recovery"
        if os.path.exists(marker_path):
            # F2 hardening (2026-04-18 audit): reject NaN/Inf literals.
            def _reject_nonfinite(token):
                raise ValueError(f"non-finite JSON constant: {token}")
            try:
                with open(marker_path, 'r') as f:
                    info = json.load(f, parse_constant=_reject_nonfinite)
                print(f"[PointMotion] WARNING: Previous interpolated "
                      f"render was interrupted. Scene keyframes may "
                      f"be at {info.get('target_fps', '?')} fps "
                      f"instead of {info.get('source_fps', '?')} fps. "
                      f"Reload the auto-saved .blend to recover.")
            except Exception as e:
                print(f"[PointMotion] WARNING: Found crash recovery "
                      f"marker at {marker_path} but could not read it "
                      f"({e}). Reload the auto-saved .blend if "
                      f"keyframes look wrong.")


@persistent
def _on_save_pre(filepath):
    """Convert any absolute PLY paths to relative before saving."""
    if not bpy.data.filepath:
        return
    for obj in bpy.data.objects:
        if "pm_ply_dir" in obj:
            stored = obj["pm_ply_dir"]
            # Skip if already relative (starts with //)
            if stored.startswith("//"):
                continue
            try:
                obj["pm_ply_dir"] = bpy.path.relpath(stored)
            except ValueError:
                pass  # different drive, keep absolute


def _register_frame_handler():
    if _on_frame_change not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(_on_frame_change)


def _unregister_frame_handler():
    if _on_frame_change in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(_on_frame_change)


# ===========================================================================
# Scene defaults — applied once on first import into a fresh scene
# ===========================================================================

def _apply_lidar_scene_defaults(context, sensor_fps=None):
    """Apply LiDAR-friendly scene defaults on first import.
    Skips if the scene already uses Cycles or has a custom world."""
    scene = context.scene

    # Only apply to scenes still using the default EEVEE engine
    if 'EEVEE' in scene.render.engine:
        scene.render.engine = 'CYCLES'
        prefs = bpy.context.preferences.addons.get('cycles')
        if prefs:
            import sys as _sys
            if _sys.platform == 'darwin':
                try:
                    prefs.preferences.compute_device_type = 'METAL'
                except Exception:
                    pass
            else:
                for _dev in ('CUDA', 'OPTIX', 'HIP', 'ONEAPI'):
                    try:
                        prefs.preferences.compute_device_type = _dev
                        break
                    except Exception:
                        continue
        scene.cycles.device = 'GPU'

        # Cycles sampling — point clouds don't need high samples
        # (no caustics, no subsurface, mostly direct illumination)
        scene.cycles.preview_samples = 64
        scene.cycles.samples = 128

    # Set FPS from sensor Hz (only if still at Blender's default 24)
    if scene.render.fps == 24:
        if sensor_fps and sensor_fps in (5, 10, 20):
            scene.render.fps = sensor_fps
        else:
            scene.render.fps = 10  # most common Ouster default

    # Black world background (only if still default grey)
    world = scene.world
    if world and world.use_nodes:
        bg = world.node_tree.nodes.get('Background')
        if bg:
            current = bg.inputs['Color'].default_value[:3]
            if all(c < 0.1 for c in current):
                bg.inputs['Color'].default_value = (0, 0, 0, 1)
                bg.inputs['Strength'].default_value = 1.0


# ===========================================================================
# Scene object helper
# ===========================================================================

def _create_sequence_object(context, base_name, ply_dir, total_frames,
                            has_color=False, sensor_fps=None):
    _apply_lidar_scene_defaults(context, sensor_fps=sensor_fps)

    START_FRAME = 1

    existing = bpy.data.objects.get(base_name)
    if existing and "pm_ply_dir" in existing:
        bpy.data.objects.remove(existing, do_unlink=True)

    mesh = bpy.data.meshes.new(name=base_name)
    obj  = bpy.data.objects.new(name=base_name, object_data=mesh)
    context.collection.objects.link(obj)
    context.view_layer.objects.active = obj

    # Material MUST be created before GeoNodes so the
    # Set Material node can reference it
    if has_color:
        _ensure_lidar_material(obj)

    apply_point_cloud_geometry_nodes(obj, has_color=has_color)
    _register_sequence(obj, ply_dir, total_frames, START_FRAME,
                       has_color=has_color)
    _register_frame_handler()

    context.scene.frame_start = START_FRAME
    context.scene.frame_end = max(
        context.scene.frame_end,
        START_FRAME + total_frames - 1
    )

    first = os.path.join(ply_dir, "frame_0000.ply")
    if os.path.exists(first):
        _load_frame_into_mesh(
            mesh, first, has_color,
            ply_dir=ply_dir,
            total_frames=total_frames,
            scene=context.scene)

    return obj


def _run_decode(context, source_path, sensor_jsons, sensor_idx,
                base_name, auto_load,
                apply_extrinsic=False, extrinsic_overrides=None,
                include_attributes=False, max_frames=0):
    output_dir = os.path.join(
        os.path.dirname(source_path), base_name + "_ply_frames"
    )
    ply_dir, total, sensor_fps = decode_to_ply(
        source_path, sensor_jsons, sensor_idx, output_dir,
        apply_extrinsic=apply_extrinsic,
        extrinsic_overrides=extrinsic_overrides,
        include_attributes=include_attributes,
        max_frames=max_frames,
    )
    if total == 0:
        log_path = os.path.join(os.path.dirname(source_path),
                                "pointmotion_decode.log")
        raise ValueError(
            f"No frames decoded.\n\n"
            f"Diagnostic log written to:\n{log_path}\n\n"
            f"Open that file in TextEdit to see what the SDK matched."
        )
    has_color = include_attributes
    if auto_load:
        _create_sequence_object(context, base_name, ply_dir, total,
                                has_color=has_color,
                                sensor_fps=sensor_fps)
    return ply_dir, total


# ===========================================================================
# Sequence editing helpers
# ===========================================================================

def _get_crop_box_bounds(box_obj):
    corners = [box_obj.matrix_world @ Vector(c) for c in box_obj.bound_box]
    xs = [v.x for v in corners]
    ys = [v.y for v in corners]
    zs = [v.z for v in corners]
    return (np.array([min(xs), min(ys), min(zs)]),
            np.array([max(xs), max(ys), max(zs)]))


def _apply_box_crop(ply_dir, total, bmin, bmax, mode):
    modified = 0
    for i in range(total):
        fp = os.path.join(ply_dir, f"frame_{i:04d}.ply")
        if not os.path.exists(fp):
            continue
        pts    = load_ply_points(fp)
        inside = np.all((pts >= bmin) & (pts <= bmax), axis=1)
        mask   = inside if mode == 'KEEP_INSIDE' else ~inside
        _write_ply(fp, pts[mask])
        modified += 1
        if i % 50 == 0:
            print(f"[PointMotion] Crop {i:04d} — {int(mask.sum())} pts")
    return modified



# ===========================================================================
# OPERATOR — Import PCAP / OSF  (unified)
# ===========================================================================

class LIDAR_OT_ImportSource(Operator, ImportHelper):
    """
    Import Ouster PCAP or OSF.
    PCAP: metadata auto-detected from .json / .tar / _metadata/ folder.
    OSF:  metadata is embedded — no external files needed.
    Multi-sensor: picker shows model + serial number.
    """
    bl_idname  = "lidar.import_source"
    bl_label   = "Import PCAP / OSF"
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(default="*.pcap;*.osf", options={'HIDDEN'})
    filename_ext = ".pcap"

    show_advanced: BoolProperty(name="Advanced", default=False)
    start_frame: IntProperty(name="Start Frame", default=1, min=0)
    max_frames: IntProperty(
        name="Max Frames",
        description="Stop decoding after this many frames. "
                    "0 = decode all. Useful for quick testing.",
        default=0, min=0
    )
    auto_load_sequence: BoolProperty(
        name="Auto-load sequence after decode",
        description="Load PLY sequence into Blender immediately after "
                    "decoding. Disable to decode to disk only.",
        default=True
    )
    apply_extrinsic: BoolProperty(
        name="Apply extrinsic transform",
        description="Apply the sensor's extrinsic calibration to place "
                    "points in world frame. Recommended for multi-sensor.",
        default=False
    )
    include_attributes: BoolProperty(
        name="Include Attributes",
        description="Decode per-point attributes (Signal, Reflectivity, "
                    "Near-IR, Range). Uncheck to import geometry only.",
        default=True
    )
    auto_merge_sensors: BoolProperty(
        name="Auto-merge multi-sensor",
        description="When importing All Sensors, merge both into one "
                    "combined PLY sequence. Recommended for most workflows.",
        default=True
    )
    selected_sensor: EnumProperty(
        name="Sensor",
        description="Select which sensor stream to import",
        items=_sensor_enum_items
    )

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        global _pending_source_path, _pending_sensor_jsons
        global _pending_sensor_labels, _pending_is_osf
        global _pending_sensor_infos

        source_path = self.filepath
        if not os.path.exists(source_path):
            self.report({'ERROR'}, f"File not found: {source_path}")
            return {'CANCELLED'}

        is_osf = source_path.lower().endswith(".osf")

        # ── OSF: probe sensor_info from the file directly ─────────────
        if is_osf:
            open_source, core = get_ouster_sdk()
            if open_source is None:
                self.report({'ERROR'}, "ouster-sdk not available.")
                return {'CANCELLED'}
            try:
                probe = open_source(source_path)
                infos = probe.sensor_info
            except Exception as e:
                self.report({'ERROR'}, f"Cannot open OSF: {e}")
                return {'CANCELLED'}

            sensor_jsons = []   # not needed for OSF

            if len(infos) == 1:
                return self._decode(context, source_path, sensor_jsons,
                                    0, infos)

            # Multi-sensor OSF — show picker
            if self.selected_sensor and self.selected_sensor != "NONE":
                if self.selected_sensor == "ALL":
                    return self._decode_all(context, source_path,
                                            sensor_jsons, infos)
                idx = int(self.selected_sensor)
                return self._decode(context, source_path, sensor_jsons,
                                    idx, infos)

            _pending_source_path   = source_path
            _pending_sensor_jsons  = []
            _pending_sensor_labels = [sensor_label_from_info(si)
                                      for si in infos]
            _pending_is_osf        = True
            _pending_sensor_infos  = infos

            print("[PointMotion] Multi-sensor OSF:")
            for i, lbl in enumerate(_pending_sensor_labels):
                print(f"  Sensor {i}: {lbl}")
            return context.window_manager.invoke_props_dialog(
                self, width=480)

        # ── PCAP: find metadata JSON(s) ───────────────────────────────
        try:
            sensor_jsons = find_metadata_for_pcap(source_path)
        except (FileNotFoundError, ValueError) as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}

        if len(sensor_jsons) == 1:
            return self._decode(context, source_path, sensor_jsons, 0)

        # Multi-sensor PCAP — if picker was already shown, decode
        if self.selected_sensor and self.selected_sensor != "NONE":
            if self.selected_sensor == "ALL":
                return self._decode_all(context, source_path,
                                        sensor_jsons)
            idx = int(self.selected_sensor)
            return self._decode(context, source_path, sensor_jsons, idx)

        # Show picker dialog
        _pending_source_path   = source_path
        _pending_sensor_jsons  = sensor_jsons
        _pending_sensor_labels = [sensor_label(j) for j in sensor_jsons]
        _pending_is_osf        = False
        _pending_sensor_infos  = []

        print("[PointMotion] Multi-sensor PCAP:")
        for i, lbl in enumerate(_pending_sensor_labels):
            print(f"  Sensor {i}: {lbl}")

        return context.window_manager.invoke_props_dialog(self, width=480)

    def draw(self, context):
        layout = self.layout

        if _pending_sensor_labels and len(_pending_sensor_labels) > 1:
            box = layout.box()
            box.label(
                text=f"Multi-sensor — {len(_pending_sensor_labels)} "
                     f"sensors found:",
                icon='OUTLINER_DATA_POINTCLOUD'
            )
            for i, lbl in enumerate(_pending_sensor_labels):
                box.label(text=f"Sensor {i}:  {lbl}", icon='MESH_DATA')
            layout.separator()
            layout.prop(self, "selected_sensor", text="Import sensor")
            if _pending_is_osf:
                layout.label(
                    text="'All Sensors' imports every sensor "
                         "simultaneously.",
                    icon='INFO')
                layout.prop(self, "auto_merge_sensors")
            else:
                layout.label(
                    text="PCAP multi-sensor: select one at a time.",
                    icon='INFO')
                layout.label(
                    text="For 'All Sensors', convert to OSF first.",
                    icon='INFO')
            layout.separator()

        layout.prop(self, "auto_load_sequence")
        layout.prop(self, "apply_extrinsic")
        layout.prop(self, "include_attributes")
        if not self.include_attributes:
            sub = layout.column(align=True)
            sub.scale_y = 0.75
            sub.label(text="  Geometry only — no color attributes",
                      icon='INFO')

        layout.separator()
        row = layout.row()
        row.prop(self, "show_advanced",
                 icon='TRIA_DOWN' if self.show_advanced else 'TRIA_RIGHT',
                 emboss=False)
        if self.show_advanced:
            box = layout.box()
            box.prop(self, "start_frame")
            box.label(text="Which Blender frame scan 0 maps to.",
                      icon='INFO')
            box.separator()
            box.prop(self, "max_frames")
            box.label(text="0 = all frames. Try 50 for quick tests.",
                      icon='INFO')

    # ── Decode one sensor ─────────────────────────────────────────────

    def _decode(self, context, source_path, sensor_jsons, sensor_idx,
                osf_infos=None):
        stem = os.path.splitext(os.path.basename(source_path))[0]

        # Build a Readable base name with SN
        import re as _re
        if osf_infos and sensor_idx < len(osf_infos):
            sn = _re.sub(r'[^a-zA-Z0-9_-]', '_',
                         str(osf_infos[sensor_idx].sn))
            base_name = f"{stem}__{sn}"
        elif sensor_jsons and len(sensor_jsons) > 1:
            info = read_sensor_info(sensor_jsons[sensor_idx])
            sn = _re.sub(r'[^a-zA-Z0-9_-]', '_',
                         str(info.sn) if info else str(sensor_idx))
            base_name = f"{stem}__{sn}"
        else:
            base_name = stem

        # Look for external extrinsics file
        ext_overrides = None
        if self.apply_extrinsic:
            md_dir = None
            folder = os.path.dirname(os.path.abspath(source_path))
            md_candidate = os.path.join(folder, f"{stem}_metadata")
            if os.path.isdir(md_candidate):
                md_dir = md_candidate
            ext_overrides = find_extrinsics_file(source_path, md_dir)

        try:
            ply_dir, total = _run_decode(
                context, source_path, sensor_jsons, sensor_idx,
                base_name, self.auto_load_sequence,
                apply_extrinsic=self.apply_extrinsic,
                extrinsic_overrides=ext_overrides,
                include_attributes=self.include_attributes,
                max_frames=self.max_frames,
            )
        except Exception as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}

        msg = (f"Loaded {total} frames — {base_name}"
               if self.auto_load_sequence
               else f"Decoded {total} frames to disk — {base_name}")
        self.report({'INFO'}, msg)
        return {'FINISHED'}

    # ── Decode ALL sensors ────────────────────────────────────────────

    def _decode_all(self, context, source_path, sensor_jsons,
                    osf_infos=None):
        stem = os.path.splitext(os.path.basename(source_path))[0]

        if osf_infos:
            n = len(osf_infos)
        else:
            n = len(sensor_jsons)

        ext_overrides = None
        if self.apply_extrinsic:
            md_dir = None
            folder = os.path.dirname(os.path.abspath(source_path))
            md_candidate = os.path.join(folder, f"{stem}_metadata")
            if os.path.isdir(md_candidate):
                md_dir = md_candidate
            ext_overrides = find_extrinsics_file(source_path, md_dir)

        # Decode each sensor to disk (don't auto-load yet if merging)
        should_merge = self.auto_merge_sensors and n > 1
        per_sensor_dirs = []
        errors = []

        for idx in range(n):
            if osf_infos:
                sn = str(osf_infos[idx].sn)
            elif sensor_jsons:
                info = read_sensor_info(sensor_jsons[idx])
                sn = str(info.sn) if info else str(idx)
            else:
                sn = str(idx)
            import re as _re
            sn = _re.sub(r'[^a-zA-Z0-9_-]', '_', sn)
            base_name = f"{stem}__{sn}"

            # If merging: decode to disk only, don't load individual
            load_this = self.auto_load_sequence and not should_merge

            try:
                ply_dir, total = _run_decode(
                    context, source_path, sensor_jsons, idx,
                    base_name, load_this,
                    apply_extrinsic=self.apply_extrinsic,
                    extrinsic_overrides=ext_overrides,
                    include_attributes=self.include_attributes,
                    max_frames=self.max_frames,
                )
                per_sensor_dirs.append((ply_dir, total, sn))
            except Exception as e:
                errors.append(f"Sensor {idx} (SN:{sn}): {e}")

        # Auto-merge if requested
        if should_merge and len(per_sensor_dirs) >= 2:
            merged_name = f"{stem}__merged"
            merged_dir  = os.path.join(
                os.path.dirname(source_path),
                merged_name + "_ply_frames")
            try:
                _, merged_total = merge_ply_sequences(
                    per_sensor_dirs[0][0],
                    per_sensor_dirs[1][0],
                    merged_dir)
                if self.auto_load_sequence:
                    _create_sequence_object(
                        context, merged_name, merged_dir,
                        merged_total, has_color=self.include_attributes)
                self.report(
                    {'INFO'},
                    f"Merged {n} sensors → {merged_total} frames"
                    f" — {merged_name}")
            except Exception as e:
                errors.append(f"Merge failed: {e}")
                # Fallback: load individual sequences
                for ply_dir, total, sn in per_sensor_dirs:
                    _create_sequence_object(
                        context, f"{stem}__{sn}", ply_dir,
                        total, has_color=self.include_attributes)
        elif not should_merge:
            total_loaded = sum(t for _, t, _ in per_sensor_dirs)
            self.report({'INFO'},
                        f"All {n} sensors loaded — "
                        f"{total_loaded} total frames")

        if errors:
            self.report({'WARNING'},
                        f"Errors: {'; '.join(errors)}")
        return {'FINISHED'}


# ===========================================================================
# OPERATOR — Legacy Import PCAP (alias for backwards compatibility)
# ===========================================================================

class LIDAR_OT_ImportPCAP(Operator, ImportHelper):
    """Import Ouster PCAP (legacy shortcut — redirects to Import Source)."""
    bl_idname  = "lidar.import_pcap"
    bl_label   = "Import PCAP"
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(default="*.pcap", options={'HIDDEN'})
    filename_ext = ".pcap"

    def invoke(self, context, event):
        # Redirect to the unified importer
        return bpy.ops.lidar.import_source('INVOKE_DEFAULT')

    def execute(self, context):
        return bpy.ops.lidar.import_source('INVOKE_DEFAULT')


# ===========================================================================
# OPERATOR — Load PLY folder
# ===========================================================================

class LIDAR_OT_LoadPLYFolder(Operator, ImportHelper):
    """Load an already-decoded PLY sequence folder."""
    bl_idname  = "lidar.load_ply_folder"
    bl_label   = "Load PLY Sequence"
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(default="*.ply", options={'HIDDEN'})
    filename_ext = ".ply"

    def execute(self, context):
        ply_dir      = os.path.dirname(self.filepath)
        plys         = sorted([f for f in os.listdir(ply_dir)
                               if f.endswith(".ply")])
        total_frames = len(plys)
        if total_frames == 0:
            self.report({'ERROR'}, "No PLY files found in folder.")
            return {'CANCELLED'}
        base_name = os.path.basename(ply_dir)
        _create_sequence_object(context, base_name, ply_dir, total_frames)
        self.report({'INFO'}, f"Loaded {total_frames} frames — {base_name}")
        return {'FINISHED'}


# ===========================================================================
# OPERATORS — Sequence management
# ===========================================================================

class LIDAR_OT_RemoveSequence(Operator):
    bl_idname  = "lidar.remove_sequence"
    bl_label   = "Remove Sequence"
    bl_options = {'REGISTER', 'UNDO'}
    obj_name: StringProperty()

    def execute(self, context):
        seq = _sequence_registry.pop(self.obj_name, None)
        # Clear cached frames for this sequence
        if seq:
            ply_dir = seq["ply_dir"]
            keys_to_remove = [k for k in _frame_cache
                              if k[0] == ply_dir]
            for k in keys_to_remove:
                _frame_cache.pop(k, None)
        obj = bpy.data.objects.get(self.obj_name)
        if obj:
            bpy.data.objects.remove(obj, do_unlink=True)
        if not _sequence_registry:
            _unregister_frame_handler()
        return {'FINISHED'}


class LIDAR_OT_BackupSequence(Operator):
    bl_idname = "lidar.backup_sequence"
    bl_label  = "Backup Sequence"
    obj_name: StringProperty()

    def execute(self, context):
        import shutil
        seq = _sequence_registry.get(self.obj_name)
        if not seq:
            self.report({'ERROR'}, "Sequence not found.")
            return {'CANCELLED'}
        backup = seq["ply_dir"] + "_backup"
        if os.path.exists(backup):
            self.report({'WARNING'},
                        f"Backup already exists: "
                        f"{os.path.basename(backup)}")
            return {'CANCELLED'}
        shutil.copytree(seq["ply_dir"], backup)
        self.report({'INFO'}, f"Backup: {os.path.basename(backup)}")
        return {'FINISHED'}


# ===========================================================================
# OPERATORS — Interactive crop
# ===========================================================================

class LIDAR_OT_PlaceCropBox(Operator):
    """Place a wireframe cube. Move / Scale / Rotate it, then Execute."""
    bl_idname  = "lidar.place_crop_box"
    bl_label   = "Place Crop Box"
    bl_options = {'REGISTER', 'UNDO'}
    obj_name: StringProperty()

    def execute(self, context):
        if not _sequence_registry.get(self.obj_name):
            self.report({'ERROR'}, "Sequence not found.")
            return {'CANCELLED'}
        existing = bpy.data.objects.get(f"CropBox_{self.obj_name}")
        if existing:
            bpy.data.objects.remove(existing, do_unlink=True)
        bpy.ops.mesh.primitive_cube_add(size=10.0, location=(0, 0, 0))
        box = context.active_object
        box.name              = f"CropBox_{self.obj_name}"
        box.display_type      = 'WIRE'
        box["pm_crop_target"] = self.obj_name
        self.report({'INFO'},
                    "Crop box placed. Move/Scale/Rotate, then Execute.")
        return {'FINISHED'}


class LIDAR_OT_ExecuteInteractiveCrop(Operator):
    bl_idname  = "lidar.execute_interactive_crop"
    bl_label   = "Execute Interactive Crop"
    bl_options = {'REGISTER'}
    obj_name: StringProperty()
    mode: EnumProperty(
        name="Mode",
        items=[
            ('KEEP_INSIDE',  "Keep Inside Box",  ""),
            ('KEEP_OUTSIDE', "Keep Outside Box", ""),
        ],
        default='KEEP_INSIDE'
    )

    def invoke(self, context, event):
        if not bpy.data.objects.get(f"CropBox_{self.obj_name}"):
            self.report({'ERROR'}, "No crop box found. Place one first.")
            return {'CANCELLED'}
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "mode")
        layout.separator()
        layout.label(
            text="⚠️  Permanently rewrites PLY files on disk.",
            icon='ERROR')
        layout.label(text="Use Backup first if needed.")

    def execute(self, context):
        seq = _sequence_registry.get(self.obj_name)
        if not seq:
            self.report({'ERROR'}, "Sequence not found.")
            return {'CANCELLED'}
        box = bpy.data.objects.get(f"CropBox_{self.obj_name}")
        if not box:
            self.report({'ERROR'}, "Crop box missing — place it again.")
            return {'CANCELLED'}
        bmin, bmax = _get_crop_box_bounds(box)
        modified = _apply_box_crop(seq["ply_dir"], seq["total"],
                                   bmin, bmax, self.mode)
        bpy.data.objects.remove(box, do_unlink=True)
        self.report({'INFO'}, f"Crop applied to {modified} frames.")
        _on_frame_change(context.scene)
        return {'FINISHED'}


# ===========================================================================
# OPERATOR — Merge Sequences
# ===========================================================================

def _merge_seq_enum_items(self, context):
    items = []
    for name in _sequence_registry:
        items.append((name, name, ""))
    if not items:
        items.append(("NONE", "No sequences loaded", ""))
    return items


class LIDAR_OT_MergeSequences(Operator):
    """Merge two PLY sequences frame-by-frame into one combined sequence."""
    bl_idname  = "lidar.merge_sequences"
    bl_label   = "Merge Two Sequences"
    bl_options = {'REGISTER', 'UNDO'}

    seq_a: EnumProperty(name="Sequence A", items=_merge_seq_enum_items)
    seq_b: EnumProperty(name="Sequence B", items=_merge_seq_enum_items)
    auto_load: BoolProperty(
        name="Load merged sequence",
        description="Import the merged PLY sequence into Blender",
        default=True
    )
    remove_originals: BoolProperty(
        name="Remove original sequences",
        description="Remove source sequences from Blender after merge. "
                    "PLY files on disk are kept.",
        default=True
    )

    def invoke(self, context, event):
        if len(_sequence_registry) < 2:
            self.report({'ERROR'},
                        "Need at least two loaded sequences to merge.")
            return {'CANCELLED'}
        return context.window_manager.invoke_props_dialog(self, width=400)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "seq_a")
        layout.prop(self, "seq_b")
        layout.separator()
        layout.prop(self, "auto_load")
        layout.prop(self, "remove_originals")
        layout.separator()
        layout.label(
            text="Merges frame-by-frame. Both sequences should be "
                 "aligned (use extrinsics on import).",
            icon='INFO')

    def execute(self, context):
        if self.seq_a == self.seq_b:
            self.report({'ERROR'}, "Select two different sequences.")
            return {'CANCELLED'}
        a = _sequence_registry.get(self.seq_a)
        b = _sequence_registry.get(self.seq_b)
        if not a or not b:
            self.report({'ERROR'}, "Sequence(s) not found in registry.")
            return {'CANCELLED'}

        has_color = a.get("has_color", False) or \
            b.get("has_color", False)

        merged_name = f"{self.seq_a}__merged__{self.seq_b}"
        output_dir  = os.path.join(
            os.path.dirname(a["ply_dir"]),
            merged_name + "_ply_frames"
        )

        try:
            _, total = merge_ply_sequences(
                a["ply_dir"], b["ply_dir"], output_dir)
        except Exception as e:
            self.report({'ERROR'}, f"Merge failed: {e}")
            return {'CANCELLED'}

        # Remove originals from Blender (not from disk)
        if self.remove_originals:
            for name in (self.seq_a, self.seq_b):
                _sequence_registry.pop(name, None)
                obj = bpy.data.objects.get(name)
                if obj:
                    bpy.data.objects.remove(obj, do_unlink=True)

        if self.auto_load:
            _create_sequence_object(context, merged_name,
                                    output_dir, total,
                                    has_color=has_color)
        self.report({'INFO'},
                    f"Merged {total} frames → {merged_name}")
        return {'FINISHED'}


# ===========================================================================
# UI Panel
# ===========================================================================

class LIDAR_PT_MainPanel(Panel):
    bl_label       = "Point Motion LiDAR — Beta"
    bl_idname      = "LIDAR_PT_main_beta"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = "LiDAR Beta"

    def draw(self, context):
        layout = self.layout

        # ── Import section ────────────────────────────────────────────
        box = layout.box()
        box.label(text="Import", icon='IMPORT')
        box.operator("lidar.import_source",
                     text="Import PCAP / OSF", icon='FILE')
        box.operator("lidar.load_ply_folder",
                     text="Load PLY Sequence", icon='SEQUENCE')
        box.operator("lidar.load_slam_output",
                     text="Load SLAM Output", icon='WORLD')
        sub = box.column(align=True)
        sub.scale_y = 0.75
        sub.label(text="PCAP: single-sensor decode (multi-sensor → OSF)",
                  icon='INFO')
        sub.label(text="OSF: full multi-sensor + extrinsics support",
                  icon='INFO')

        layout.separator()

        # ── Active sequences ──────────────────────────────────────────
        box = layout.box()
        box.label(text="Active Sequences", icon='POINTCLOUD_DATA')

        if not _sequence_registry:
            box.label(text="No sequences loaded", icon='INFO')
        else:
            for obj_name, seq in _sequence_registry.items():
                col = box.column(align=True)
                row = col.row(align=True)
                row.label(text=obj_name, icon='MESH_DATA')
                row.label(text=f"{seq['total']} frames")
                row.operator("lidar.remove_sequence", text="",
                             icon='X').obj_name = obj_name

                sub = col.row(align=True)
                sub.operator("lidar.backup_sequence",
                             text="Backup",
                             icon='FILE_BACKUP').obj_name = obj_name
                sub.operator("lidar.place_crop_box",
                             text="Crop Box",
                             icon='MESH_CUBE').obj_name = obj_name
                sub.operator("lidar.execute_interactive_crop",
                             text="Apply Crop",
                             icon='CHECKMARK').obj_name = obj_name
                sub2 = col.row(align=True)
                sub2.operator("lidar.interpolate_sequence",
                              text="Interpolate",
                              icon='IPO_BEZIER').obj_name = obj_name
                col.separator()

        # ── Merge section ─────────────────────────────────────────────
        if len(_sequence_registry) >= 2:
            layout.separator()
            box = layout.box()
            box.label(text="Multi-Sensor Merge", icon='MOD_BOOLEAN')
            box.operator("lidar.merge_sequences",
                         text="Merge Two Sequences",
                         icon='AUTOMERGE_ON')
            sub = box.column(align=True)
            sub.scale_y = 0.75
            sub.label(text="Combines two sequences frame-by-frame",
                      icon='INFO')
            sub.label(text="Import with extrinsics for correct alignment",
                      icon='INFO')

        layout.separator()

        # ── SDK status ────────────────────────────────────────────────
        box = layout.box()
        open_source, _ = get_ouster_sdk()
        if open_source:
            box.label(text="ouster-sdk: ✓ Ready", icon='CHECKMARK')
        else:
            box.label(text="ouster-sdk: ✗ Not found", icon='ERROR')
            box.label(text="Reinstall with complete wheels/ folder")

        layout.label(text="v1.5.6-beta — JibuFilm", icon='EXPERIMENTAL')


class LIDAR_PT_ColorTexture(Panel):
    bl_label       = "Color / Texture"
    bl_idname      = "LIDAR_PT_color_texture"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = "LiDAR Beta"
    bl_parent_id   = "LIDAR_PT_main_beta"
    bl_options     = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return bool(_sequence_registry)

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        layout.prop(scene, "pm_active_channel", text="Channel")
        layout.prop(scene, "pm_colormap", text="Colormap")
        layout.prop(scene, "pm_auto_levels")
        layout.prop(scene, "pm_push_all_attributes")


# ===========================================================================
# Registration
# ===========================================================================



classes = (
    LIDAR_OT_ImportSource,
    LIDAR_OT_ImportPCAP,       # legacy alias
    LIDAR_OT_LoadPLYFolder,
    LIDAR_OT_RemoveSequence,
    LIDAR_OT_BackupSequence,
    LIDAR_OT_PlaceCropBox,
    LIDAR_OT_ExecuteInteractiveCrop,
    LIDAR_OT_MergeSequences,
    LIDAR_PT_MainPanel,
    LIDAR_PT_ColorTexture,
)


def register():
    from bpy.props import EnumProperty, BoolProperty

    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.pm_active_channel = EnumProperty(
        name="Channel",
        items=[
            ("reflectivity", "Reflectivity",
             "Calibrated reflectivity (0-255)"),
            ("signal", "Signal",
             "Raw signal photon count"),
            ("near_ir", "Near IR",
             "Ambient sunlight photon count"),
            ("range", "Range",
             "Distance from sensor"),
        ],
        default="reflectivity",
        update=_on_viz_setting_changed,
    )
    bpy.types.Scene.pm_colormap = EnumProperty(
        name="Colormap",
        items=[
            ("VIRIDIS",   "Viridis",
             "Purple → teal → green → yellow"),
            ("MAGMA",     "Magma",
             "Black → purple → orange → white"),
            ("GREYSCALE", "Greyscale",
             "Black → white"),
            ("OUSTER",    "Ouster",
             "Ouster branded palette"),
            ("CALREF",    "CalRef",
             "Calibrated reflectivity ramp"),
        ],
        default="VIRIDIS",
        update=_on_viz_setting_changed,
    )
    bpy.types.Scene.pm_auto_levels = BoolProperty(
        name="Auto-Levels",
        description="Use sequence-stable sampled contrast normalization",
        default=True,
        update=_on_viz_setting_changed,
    )
    bpy.types.Scene.pm_push_all_attributes = BoolProperty(
        name="Push All Attributes",
        description="Upload every attribute each frame for custom materials "
                    "and Geometry Nodes. Disable for faster default renders",
        default=False,
        update=_on_viz_setting_changed,
    )
    _register_frame_handler()
    if _on_load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load_post)
    if _on_save_pre not in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.append(_on_save_pre)


def unregister():
    _unregister_frame_handler()
    if _on_load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load_post)
    if _on_save_pre in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.remove(_on_save_pre)
    _sequence_registry.clear()
    _clear_frame_cache()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.pm_active_channel
    del bpy.types.Scene.pm_colormap
    del bpy.types.Scene.pm_auto_levels
    del bpy.types.Scene.pm_push_all_attributes


if __name__ == "__main__":
    register()
