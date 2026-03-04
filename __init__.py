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
# https://jibujin.com

bl_info = {
    "name": "Point Motion LiDAR Importer",
    "author": "JibuFilm",
    "version": (1, 1, 0),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > LiDAR",
    "description": "Import Ouster LiDAR PCAP recordings as animated point cloud sequences",
    "category": "Import-Export",
}

import bpy
import os
import numpy as np
from bpy.props import StringProperty, IntProperty, BoolProperty, FloatProperty
from bpy.types import Operator, Panel
from bpy_extras.io_utils import ImportHelper
from bpy.app.handlers import persistent


# ---------------------------------------------------------------------------
# Ouster SDK
# ---------------------------------------------------------------------------

def get_ouster_sdk():
    try:
        from ouster.sdk import open_source
        import ouster.sdk.core as core
        return open_source, core
    except ImportError:
        return None, None


# ---------------------------------------------------------------------------
# PLY utilities
# ---------------------------------------------------------------------------

def decode_pcap_to_ply(pcap_path, json_path, output_dir):
    open_source, core = get_ouster_sdk()
    if open_source is None:
        raise ImportError("ouster-sdk not found. Please install it: pip install ouster-sdk")

    os.makedirs(output_dir, exist_ok=True)
    source = open_source(pcap_path, meta=[json_path])
    info = source.sensor_info[0]
    xyz_lut = core.XYZLut(info)

    frame_count = 0
    for i, scanset in enumerate(source):
        scan = scanset[0]
        xyz = xyz_lut(scan)
        points = xyz.reshape(-1, 3)
        mask = np.linalg.norm(points, axis=1) > 0.1
        points = points[mask]
        _write_ply(os.path.join(output_dir, f"frame_{i:04d}.ply"), points)
        frame_count += 1
        print(f"  Decoded frame {i:04d} — {len(points)} points")

    return output_dir, frame_count


def _write_ply(filepath, points):
    with open(filepath, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        np.savetxt(f, points, fmt="%.4f")


def load_ply_as_mesh_verts(filepath):
    vertices = []
    in_header = True
    vertex_count = 0
    parsed = 0
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if in_header:
                if line.startswith("element vertex"):
                    vertex_count = int(line.split()[-1])
                elif line == "end_header":
                    in_header = False
            else:
                if parsed >= vertex_count:
                    break
                parts = line.split()
                if len(parts) >= 3:
                    vertices.append((float(parts[0]), float(parts[1]), float(parts[2])))
                    parsed += 1
    return vertices


# ---------------------------------------------------------------------------
# Geometry nodes
# ---------------------------------------------------------------------------

def apply_point_cloud_geometry_nodes(obj):
    modifier = obj.modifiers.new(name="PointCloud", type='NODES')
    node_group = bpy.data.node_groups.new(name="LiDAR_PointCloud", type='GeometryNodeTree')
    modifier.node_group = node_group
    nodes = node_group.nodes
    links = node_group.links
    group_input = nodes.new('NodeGroupInput')
    group_output = nodes.new('NodeGroupOutput')
    node_group.interface.new_socket('Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
    node_group.interface.new_socket('Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
    m2p = nodes.new('GeometryNodeMeshToPoints')
    m2p.inputs['Radius'].default_value = 0.02
    group_input.location = (-300, 0)
    m2p.location = (0, 0)
    group_output.location = (300, 0)
    links.new(group_input.outputs['Geometry'], m2p.inputs['Mesh'])
    links.new(m2p.outputs['Points'], group_output.inputs['Geometry'])


# ---------------------------------------------------------------------------
# Sequence registry
# ---------------------------------------------------------------------------

_sequence_registry = {}


def _register_sequence(obj, ply_dir, total_frames, start_frame):
    _sequence_registry[obj.name] = {
        "ply_dir": ply_dir,
        "total": total_frames,
        "start_frame": start_frame,
    }
    # Persist to object custom properties — survives file save/load
    obj["pm_ply_dir"] = ply_dir
    obj["pm_total_frames"] = total_frames
    obj["pm_start_frame"] = start_frame


def _restore_registry_from_scene():
    _sequence_registry.clear()
    for obj in bpy.data.objects:
        if "pm_ply_dir" in obj and "pm_total_frames" in obj:
            ply_dir = obj["pm_ply_dir"]
            if os.path.exists(ply_dir):
                _sequence_registry[obj.name] = {
                    "ply_dir": ply_dir,
                    "total": obj["pm_total_frames"],
                    "start_frame": obj.get("pm_start_frame", 1),
                }
                print(f"[PointMotion] Restored sequence: {obj.name}")
            else:
                print(f"[PointMotion] WARNING: PLY folder missing for {obj.name}: {ply_dir}")


# ---------------------------------------------------------------------------
# Frame change handler
# ---------------------------------------------------------------------------

def _on_frame_change(scene, depsgraph=None):
    current_frame = scene.frame_current
    for obj_name, seq in list(_sequence_registry.items()):
        obj = bpy.data.objects.get(obj_name)
        if obj is None:
            _sequence_registry.pop(obj_name, None)
            continue
        ply_dir = seq["ply_dir"]
        total = seq["total"]
        start = seq["start_frame"]
        frame_idx = current_frame - start
        if frame_idx < 0 or frame_idx >= total:
            obj.data.clear_geometry()
            continue
        filepath = os.path.join(ply_dir, f"frame_{frame_idx:04d}.ply")
        if not os.path.exists(filepath):
            continue
        vertices = load_ply_as_mesh_verts(filepath)
        mesh = obj.data
        mesh.clear_geometry()
        mesh.from_pydata(vertices, [], [])
        mesh.update()


@persistent
def _on_load_post(filepath):
    _restore_registry_from_scene()
    _register_frame_handler()


def _register_frame_handler():
    if _on_frame_change not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(_on_frame_change)


def _unregister_frame_handler():
    if _on_frame_change in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(_on_frame_change)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _create_sequence_object(context, base_name, ply_dir, total_frames, start_frame):
    # Remove stale object with same name
    existing = bpy.data.objects.get(base_name)
    if existing and "pm_ply_dir" in existing:
        bpy.data.objects.remove(existing, do_unlink=True)

    mesh = bpy.data.meshes.new(name=base_name)
    obj = bpy.data.objects.new(name=base_name, object_data=mesh)
    context.collection.objects.link(obj)
    context.view_layer.objects.active = obj
    apply_point_cloud_geometry_nodes(obj)
    _register_sequence(obj, ply_dir, total_frames, start_frame)
    _register_frame_handler()

    context.scene.frame_start = start_frame
    context.scene.frame_end = start_frame + total_frames - 1

    first_ply = os.path.join(ply_dir, "frame_0000.ply")
    if os.path.exists(first_ply):
        verts = load_ply_as_mesh_verts(first_ply)
        mesh.from_pydata(verts, [], [])
        mesh.update()

    return obj


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

class LIDAR_OT_ImportPCAP(Operator, ImportHelper):
    """Import an Ouster LiDAR PCAP as an animated point cloud"""
    bl_idname = "lidar.import_pcap"
    bl_label = "Import LiDAR PCAP"
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(default="*.pcap", options={'HIDDEN'})
    filename_ext = ".pcap"

    start_frame: IntProperty(name="Start Frame", default=1, min=0)

    auto_load_sequence: BoolProperty(
        name="Auto-load sequence after decode",
        description="Automatically load the PLY sequence into Blender after decoding. Disable to load manually later via Load PLY Sequence.",
        default=True
    )

    def execute(self, context):
        pcap_path = self.filepath
        json_path = os.path.splitext(pcap_path)[0] + ".json"

        if not os.path.exists(json_path):
            self.report({'ERROR'}, f"JSON metadata not found: {json_path}")
            return {'CANCELLED'}

        base_name = os.path.splitext(os.path.basename(pcap_path))[0]
        output_dir = os.path.join(os.path.dirname(pcap_path), base_name + "_ply_frames")

        self.report({'INFO'}, f"Decoding {pcap_path} ...")

        try:
            ply_dir, total_frames = decode_pcap_to_ply(pcap_path, json_path, output_dir)
        except ImportError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        except Exception as e:
            self.report({'ERROR'}, f"Decode failed: {e}")
            return {'CANCELLED'}

        if total_frames == 0:
            self.report({'ERROR'}, "No frames decoded from PCAP.")
            return {'CANCELLED'}

        if self.auto_load_sequence:
            _create_sequence_object(context, base_name, ply_dir, total_frames, self.start_frame)
            self.report({'INFO'}, f"Loaded {total_frames} frames from {base_name}")
        else:
            self.report({'INFO'}, f"Decoded {total_frames} frames to {ply_dir}. Use Load PLY Sequence to load.")

        return {'FINISHED'}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "start_frame")
        layout.separator()
        layout.prop(self, "auto_load_sequence")


class LIDAR_OT_LoadPLYFolder(Operator, ImportHelper):
    """Load an already-exported PLY sequence folder"""
    bl_idname = "lidar.load_ply_folder"
    bl_label = "Load PLY Sequence"
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(default="*.ply", options={'HIDDEN'})
    filename_ext = ".ply"
    start_frame: IntProperty(name="Start Frame", default=1, min=0)

    def execute(self, context):
        ply_dir = os.path.dirname(self.filepath)
        plys = sorted([f for f in os.listdir(ply_dir) if f.endswith(".ply")])
        total_frames = len(plys)

        if total_frames == 0:
            self.report({'ERROR'}, "No PLY files found in folder.")
            return {'CANCELLED'}

        base_name = os.path.basename(ply_dir)
        _create_sequence_object(context, base_name, ply_dir, total_frames, self.start_frame)
        self.report({'INFO'}, f"Loaded PLY sequence: {total_frames} frames")
        return {'FINISHED'}


class LIDAR_OT_RemoveSequence(Operator):
    """Remove the selected LiDAR sequence from the scene"""
    bl_idname = "lidar.remove_sequence"
    bl_label = "Remove Sequence"
    bl_options = {'REGISTER', 'UNDO'}

    obj_name: StringProperty()

    def execute(self, context):
        _sequence_registry.pop(self.obj_name, None)
        obj = bpy.data.objects.get(self.obj_name)
        if obj:
            bpy.data.objects.remove(obj, do_unlink=True)
        if not _sequence_registry:
            _unregister_frame_handler()
        return {'FINISHED'}


class LIDAR_OT_BackupSequence(Operator):
    """Create a backup copy of the PLY sequence folder"""
    bl_idname = "lidar.backup_sequence"
    bl_label = "Backup Sequence"

    obj_name: StringProperty()

    def execute(self, context):
        import shutil
        seq = _sequence_registry.get(self.obj_name)
        if not seq:
            self.report({'ERROR'}, "Sequence not found.")
            return {'CANCELLED'}

        ply_dir = seq["ply_dir"]
        backup_dir = ply_dir + "_backup"

        if os.path.exists(backup_dir):
            self.report({'WARNING'}, f"Backup already exists: {backup_dir}")
            return {'CANCELLED'}

        shutil.copytree(ply_dir, backup_dir)
        self.report({'INFO'}, f"Backup created: {backup_dir}")
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# UI Panel
# ---------------------------------------------------------------------------

class LIDAR_PT_MainPanel(Panel):
    bl_label = "Point Motion LiDAR"
    bl_idname = "LIDAR_PT_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "LiDAR"

    def draw(self, context):
        layout = self.layout

        # Import
        box = layout.box()
        box.label(text="Import", icon='IMPORT')
        box.operator("lidar.import_pcap", text="Import PCAP + JSON", icon='FILE')
        box.operator("lidar.load_ply_folder", text="Load PLY Sequence", icon='SEQUENCE')

        layout.separator()

        # Active sequences
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
                op = row.operator("lidar.remove_sequence", text="", icon='X')
                op.obj_name = obj_name
                op_b = col.operator("lidar.backup_sequence",
                                    text="Backup Sequence", icon='FILE_BACKUP')
                op_b.obj_name = obj_name
                col.separator()

        layout.separator()

        # SDK status
        box = layout.box()
        open_source, core = get_ouster_sdk()
        if open_source is not None:
            box.label(text="ouster-sdk: ✓ Ready", icon='CHECKMARK')
        else:
            box.label(text="ouster-sdk: ✗ Not found", icon='ERROR')
            box.label(text="pip install ouster-sdk")

        layout.label(text="v1.1.0 — JibuFilm", icon='SEQUENCE')


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

classes = (
    LIDAR_OT_ImportPCAP,
    LIDAR_OT_LoadPLYFolder,
    LIDAR_OT_RemoveSequence,
    LIDAR_OT_BackupSequence,
    LIDAR_PT_MainPanel,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    _register_frame_handler()
    if _on_load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load_post)


def unregister():
    _unregister_frame_handler()
    if _on_load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load_post)
    _sequence_registry.clear()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
