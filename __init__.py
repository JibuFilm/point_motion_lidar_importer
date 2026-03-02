"""
Point Motion LiDAR Importer
Blender Add-on by JibuFilm

Decodes LiDAR PCAP recordings into animated point cloud sequences in Blender.
Sequence loading logic inspired by blender-sequence-loader (MIT License)
https://github.com/InteractiveComputerGraphics/blender-sequence-loader
"""

bl_info = {
    "name": "Point Motion LiDAR Importer",
    "author": "JibuFilm",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > LiDAR",
    "description": "Import Ouster LiDAR PCAP recordings as animated point cloud sequences",
    "category": "Import-Export",
}

import bpy
import os
import sys
import numpy as np
import tempfile
import threading
from bpy.props import StringProperty, IntProperty, BoolProperty, FloatProperty
from bpy.types import Operator, Panel, PropertyGroup
from bpy_extras.io_utils import ImportHelper


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def get_ouster_sdk():
    """Try to import ouster.sdk — works whether bundled via wheels or system."""
    try:
        from ouster.sdk import open_source
        import ouster.sdk.core as core
        return open_source, core
    except ImportError:
        return None, None


def count_scans(pcap_path, json_path):
    """Count total scans in PCAP without exporting."""
    open_source, core = get_ouster_sdk()
    if open_source is None:
        return 0
    source = open_source(pcap_path, meta=[json_path])
    total = sum(1 for _ in source)
    return total


def decode_pcap_to_ply(pcap_path, json_path, output_dir, progress_callback=None):
    """
    Decode a PCAP file into per-frame PLY files.
    progress_callback(frame_index, total) called each frame if provided.
    Returns (output_dir, total_frames) or raises on error.
    """
    open_source, core = get_ouster_sdk()
    if open_source is None:
        raise ImportError(
            "ouster-sdk not found. Please install it:\n"
            "  pip install ouster-sdk"
        )

    os.makedirs(output_dir, exist_ok=True)

    source = open_source(pcap_path, meta=[json_path])
    info = source.sensor_info[0]
    xyz_lut = core.XYZLut(info)

    frame_count = 0
    for i, scanset in enumerate(source):
        scan = scanset[0]
        xyz = xyz_lut(scan)

        # Flatten (H, W, 3) → (N, 3) and strip zero/invalid points
        points = xyz.reshape(-1, 3)
        mask = np.linalg.norm(points, axis=1) > 0.1
        points = points[mask]

        filepath = os.path.join(output_dir, f"frame_{i:04d}.ply")
        _write_ply(filepath, points)

        frame_count += 1
        if progress_callback:
            progress_callback(i, frame_count)

    return output_dir, frame_count


def _write_ply(filepath, points):
    """Write an ASCII PLY file from an (N, 3) numpy array."""
    with open(filepath, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        np.savetxt(f, points, fmt="%.4f")


def load_ply_as_mesh(filepath):
    """
    Load a PLY file and return a list of (x, y, z) vertex tuples.
    Pure Python — no external deps needed beyond what Blender provides.
    """
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


def apply_point_cloud_geometry_nodes(obj):
    """
    Apply a geometry nodes setup that converts vertices to renderable point cloud spheres.
    Mirrors the geometry node setup from blender-sequence-loader.
    """
    modifier = obj.modifiers.new(name="PointCloud", type='NODES')
    node_group = bpy.data.node_groups.new(name="LiDAR_PointCloud", type='GeometryNodeTree')
    modifier.node_group = node_group

    nodes = node_group.nodes
    links = node_group.links

    # Input / Output
    group_input = nodes.new('NodeGroupInput')
    group_output = nodes.new('NodeGroupOutput')
    node_group.interface.new_socket('Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
    node_group.interface.new_socket('Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')

    # Mesh to Points
    m2p = nodes.new('GeometryNodeMeshToPoints')
    m2p.inputs['Radius'].default_value = 0.02

    group_input.location = (-300, 0)
    m2p.location = (0, 0)
    group_output.location = (300, 0)

    links.new(group_input.outputs['Geometry'], m2p.inputs['Mesh'])
    links.new(m2p.outputs['Points'], group_output.inputs['Geometry'])


# ---------------------------------------------------------------------------
# Frame Change Handler — just-in-time PLY loading
# ---------------------------------------------------------------------------

_sequence_registry = {}   # obj_name → {"ply_dir": str, "total": int, "start_frame": int}


def _on_frame_change(scene, depsgraph=None):
    """Called every frame change — loads the correct PLY for each registered sequence."""
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
            # Outside range — show empty mesh
            mesh = obj.data
            mesh.clear_geometry()
            continue

        filepath = os.path.join(ply_dir, f"frame_{frame_idx:04d}.ply")
        if not os.path.exists(filepath):
            continue

        vertices = load_ply_as_mesh(filepath)
        mesh = obj.data
        mesh.clear_geometry()
        mesh.from_pydata(vertices, [], [])
        mesh.update()


def _register_frame_handler():
    if _on_frame_change not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(_on_frame_change)


def _unregister_frame_handler():
    if _on_frame_change in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(_on_frame_change)


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

    point_radius: FloatProperty(
        name="Point Radius",
        description="Display radius of each point",
        default=0.02,
        min=0.001,
        max=1.0
    )

    start_frame: IntProperty(
        name="Start Frame",
        description="Blender frame to start the sequence on",
        default=1,
        min=0
    )

    def execute(self, context):
        pcap_path = self.filepath
        json_path = os.path.splitext(pcap_path)[0] + ".json"

        if not os.path.exists(json_path):
            self.report({'ERROR'}, f"JSON metadata not found: {json_path}")
            return {'CANCELLED'}

        # Output folder next to the PCAP, named after it
        base_name = os.path.splitext(os.path.basename(pcap_path))[0]
        output_dir = os.path.join(os.path.dirname(pcap_path), base_name + "_ply_frames")

        self.report({'INFO'}, f"Decoding {pcap_path} → {output_dir} ...")

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

        # Create mesh object
        mesh = bpy.data.meshes.new(name=base_name)
        obj = bpy.data.objects.new(name=base_name, object_data=mesh)
        context.collection.objects.link(obj)
        context.view_layer.objects.active = obj

        # Apply point cloud geometry nodes
        apply_point_cloud_geometry_nodes(obj)

        # Register for just-in-time loading
        _sequence_registry[obj.name] = {
            "ply_dir": ply_dir,
            "total": total_frames,
            "start_frame": self.start_frame,
        }
        _register_frame_handler()

        # Set timeline range
        context.scene.frame_start = self.start_frame
        context.scene.frame_end = self.start_frame + total_frames - 1

        # Load first frame immediately
        first_ply = os.path.join(ply_dir, "frame_0000.ply")
        if os.path.exists(first_ply):
            vertices = load_ply_as_mesh(first_ply)
            mesh.from_pydata(vertices, [], [])
            mesh.update()

        self.report({'INFO'}, f"Loaded {total_frames} frames from {base_name}")
        return {'FINISHED'}


class LIDAR_OT_LoadPLYFolder(Operator, ImportHelper):
    """Load an already-exported PLY sequence folder directly (skip PCAP decode)"""
    bl_idname = "lidar.load_ply_folder"
    bl_label = "Load PLY Sequence"
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(default="*.ply", options={'HIDDEN'})
    filename_ext = ".ply"

    start_frame: IntProperty(
        name="Start Frame",
        default=1,
        min=0
    )

    def execute(self, context):
        ply_dir = os.path.dirname(self.filepath)
        plys = sorted([f for f in os.listdir(ply_dir) if f.endswith(".ply")])
        total_frames = len(plys)

        if total_frames == 0:
            self.report({'ERROR'}, "No PLY files found in folder.")
            return {'CANCELLED'}

        base_name = os.path.basename(ply_dir)
        mesh = bpy.data.meshes.new(name=base_name)
        obj = bpy.data.objects.new(name=base_name, object_data=mesh)
        context.collection.objects.link(obj)
        context.view_layer.objects.active = obj

        apply_point_cloud_geometry_nodes(obj)

        _sequence_registry[obj.name] = {
            "ply_dir": ply_dir,
            "total": total_frames,
            "start_frame": self.start_frame,
        }
        _register_frame_handler()

        context.scene.frame_start = self.start_frame
        context.scene.frame_end = self.start_frame + total_frames - 1

        first_ply = os.path.join(ply_dir, plys[0])
        vertices = load_ply_as_mesh(first_ply)
        mesh.from_pydata(vertices, [], [])
        mesh.update()

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

        # Import section
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
                row = box.row(align=True)
                row.label(text=obj_name, icon='MESH_DATA')
                row.label(text=f"{seq['total']} frames")
                op = row.operator("lidar.remove_sequence", text="", icon='X')
                op.obj_name = obj_name

        layout.separator()

        # SDK status
        box = layout.box()
        open_source, core = get_ouster_sdk()
        if open_source is not None:
            box.label(text="ouster-sdk: ✓ Ready", icon='CHECKMARK')
        else:
            box.label(text="ouster-sdk: ✗ Not found", icon='ERROR')
            box.label(text="pip install ouster-sdk")


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

classes = (
    LIDAR_OT_ImportPCAP,
    LIDAR_OT_LoadPLYFolder,
    LIDAR_OT_RemoveSequence,
    LIDAR_PT_MainPanel,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    _register_frame_handler()


def unregister():
    _unregister_frame_handler()
    _sequence_registry.clear()
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
