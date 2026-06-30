#!/usr/bin/env python3
"""
Convert URDF robot model to a single STL file in home configuration.

This script:
1. Parses the URDF file
2. Loads all STL mesh files
3. Applies transformations based on joint positions (home configuration)
4. Combines all meshes into a single assembly
5. Exports to STL format
"""

import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import math
from pathlib import Path

try:
    import trimesh
except ImportError:
    print("Error: trimesh is required. Install with: pip install trimesh")
    sys.exit(1)

# Try to import STEP export libraries
HAS_OCC = False
HAS_MESHIO = False
HAS_CADQUERY = False

try:
    import meshio
    HAS_MESHIO = True
except ImportError:
    pass

try:
    from OCC.Core.STEPControl_Writer import STEPControl_Writer
    from OCC.Core.IFSelect_ReturnStatus import IFSelect_ReturnStatus
    from OCC.Core.Interface_Static import Interface_Static_SetCVal
    from OCC.Core.StlAPI import StlAPI_Reader
    from OCC.Core.TopoDS import TopoDS_Shape
    HAS_OCC = True
except ImportError:
    pass

try:
    import cadquery as cq
    HAS_CADQUERY = True
except ImportError:
    pass

if not (HAS_OCC or HAS_MESHIO or HAS_CADQUERY):
    print("Warning: No STEP export library found. Install one of:")
    print("  - pythonocc-core: pip install pythonocc-core")
    print("  - meshio: pip install meshio[all]")
    print("  - cadquery: pip install cadquery")


def parse_urdf(urdf_path):
    """Parse URDF file and return root element."""
    tree = ET.parse(urdf_path)
    return tree.getroot()


def resolve_package_path(package_uri, urdf_dir):
    """
    Resolve package:// URI to absolute path.
    Format: package://lerobot/meshes/Base.stl
    """
    if not package_uri.startswith('package://'):
        return os.path.join(urdf_dir, package_uri)
    
    # Remove package:// prefix
    path = package_uri.replace('package://', '')
    parts = path.split('/')
    package_name = parts[0]
    
    # Since URDF is inside the package (e.g., ros_ws/src/lerobot/urdf/lerobot.urdf),
    # the package root is the parent directory of urdf_dir
    # urdf_dir is typically: .../package_name/urdf, so package root is os.path.dirname(urdf_dir)
    package_root = os.path.dirname(os.path.abspath(urdf_dir))
    
    # Construct full path relative to package root
    relative_path = '/'.join(parts[1:])
    full_path = os.path.join(package_root, relative_path)
    
    # If the file doesn't exist, try finding it in the workspace structure
    if not os.path.exists(full_path):
        # Find workspace root (directory containing 'src')
        workspace_root = os.path.abspath(urdf_dir)
        while not os.path.exists(os.path.join(workspace_root, 'src')):
            parent = os.path.dirname(workspace_root)
            if parent == workspace_root:
                break
            workspace_root = parent
        
        # Try workspace/src/package_name structure
        alt_path = os.path.join(workspace_root, 'src', package_name, relative_path)
        if os.path.exists(alt_path):
            return alt_path
    
    return full_path


def rpy_to_matrix(rpy):
    """Convert roll, pitch, yaw to rotation matrix."""
    roll, pitch, yaw = rpy
    # Rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    return R_z @ R_y @ R_x


def xyz_to_array(xyz_str):
    """Parse xyz string to numpy array."""
    parts = xyz_str.strip().split()
    return np.array([float(x) for x in parts])


def parse_origin(origin_elem):
    """Parse origin element and return transform matrix."""
    xyz = xyz_to_array(origin_elem.get('xyz', '0 0 0'))
    rpy = xyz_to_array(origin_elem.get('rpy', '0 0 0'))
    
    R = rpy_to_matrix(rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    
    return T


def get_joint_transform(joint, joint_positions):
    """Get transformation matrix for a joint based on its position."""
    origin_elem = joint.find('origin')
    if origin_elem is None:
        T = np.eye(4)
    else:
        T = parse_origin(origin_elem)
    
    # Apply joint rotation if it's a revolute joint
    joint_type = joint.get('type')
    if joint_type == 'revolute':
        axis_elem = joint.find('axis')
        if axis_elem is not None:
            axis = xyz_to_array(axis_elem.get('xyz', '0 0 1'))
            axis = axis / np.linalg.norm(axis)  # Normalize
            
            joint_name = joint.get('name')
            angle = joint_positions.get(joint_name, 0.0)
            
            # Create rotation matrix around axis
            # Using Rodrigues' rotation formula
            K = np.array([[0, -axis[2], axis[1]],
                         [axis[2], 0, -axis[0]],
                         [-axis[1], axis[0], 0]])
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
            
            # Apply rotation to transform
            T_rot = np.eye(4)
            T_rot[:3, :3] = R
            T = T @ T_rot
    
    return T


def get_link_transform(link_name, root, joint_positions, transforms_cache=None):
    """Get cumulative transformation for a link by traversing the kinematic chain."""
    if transforms_cache is None:
        transforms_cache = {}
    
    if link_name in transforms_cache:
        return transforms_cache[link_name]
    
    # Find joint that has this link as child
    for joint in root.findall('joint'):
        child_link = joint.find('child').get('link')
        if child_link == link_name:
            parent_link = joint.find('parent').get('link')
            
            # Get parent transform
            if parent_link == 'world':
                parent_T = np.eye(4)
            else:
                parent_T = get_link_transform(parent_link, root, joint_positions, transforms_cache)
            
            # Get joint transform
            joint_T = get_joint_transform(joint, joint_positions)
            
            # Combine transforms
            T = parent_T @ joint_T
            transforms_cache[link_name] = T
            return T
    
    # If no parent found, return identity
    transforms_cache[link_name] = np.eye(4)
    return np.eye(4)


def load_mesh_with_transform(mesh_path, transform):
    """Load STL mesh and apply transformation."""
    if not os.path.exists(mesh_path):
        print(f"Warning: Mesh file not found: {mesh_path}")
        return None
    
    try:
        mesh = trimesh.load(mesh_path)
        if isinstance(mesh, trimesh.Scene):
            # If it's a scene, combine all meshes
            meshes = []
            for node_name in mesh.graph.nodes_geometry:
                geom = mesh.geometry[node_name]
                if isinstance(geom, trimesh.Trimesh):
                    meshes.append(geom)
            if meshes:
                mesh = trimesh.util.concatenate(meshes)
            else:
                return None
        elif not isinstance(mesh, trimesh.Trimesh):
            return None
        
        # Apply transformation
        mesh.apply_transform(transform)
        return mesh
    except Exception as e:
        print(f"Error loading mesh {mesh_path}: {e}")
        return None


def get_home_configuration():
    """Get home configuration joint positions in radians."""
    DEG2RAD = math.pi / 180.0
    return {
        'Shoulder_Rotation': 0.0,
        'Shoulder_Pitch': DEG2RAD * 105.0,
        'Elbow': -DEG2RAD * 70.0,
        'Wrist_Pitch': -DEG2RAD * 60.0,
        'Wrist_Roll': 0.0,
        'Gripper': 0.0  # Closed gripper
    }




def export_to_stl(meshes, output_path):
    """Export meshes to STL format."""
    # Combine all meshes
    valid_meshes = [m for m in meshes if m is not None]
    if not valid_meshes:
        print("Error: No valid meshes to export")
        return False
    
    combined = trimesh.util.concatenate(valid_meshes)
    
    # Export to STL using trimesh
    try:
        combined.export(output_path, file_type='stl')
        return True
    except Exception as e:
        print(f"Error exporting to STL: {e}")
        return False


def main():
    if len(sys.argv) < 2:
        print("Usage: urdf_to_step.py <urdf_file> [output_stl_file] [--home-config joint1=val1 joint2=val2 ...]")
        print("Example: urdf_to_step.py urdf/lerobot.urdf lerobot_home.stl")
        sys.exit(1)
    
    urdf_path = sys.argv[1]
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found: {urdf_path}")
        sys.exit(1)
    
    # Determine output path
    if len(sys.argv) > 2 and not sys.argv[2].startswith('--'):
        output_path = sys.argv[2]
        # Ensure .stl extension
        if not output_path.endswith('.stl'):
            output_path = output_path.rsplit('.', 1)[0] + '.stl'
    else:
        # Default output path
        urdf_dir = os.path.dirname(urdf_path)
        output_path = os.path.join(urdf_dir, 'lerobot_home.stl')
    
    # Parse custom joint positions if provided
    joint_positions = get_home_configuration()
    if '--home-config' in sys.argv:
        idx = sys.argv.index('--home-config')
        for arg in sys.argv[idx+1:]:
            if '=' in arg:
                joint_name, value = arg.split('=', 1)
                joint_positions[joint_name] = float(value)
    
    print(f"Loading URDF from: {urdf_path}")
    print(f"Output STL file: {output_path}")
    print(f"Home configuration: {joint_positions}")
    
    # Parse URDF
    root = parse_urdf(urdf_path)
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    
    # Collect all meshes with their transformations
    meshes = []
    transforms_cache = {}
    
    # Process all links
    for link in root.findall('link'):
        link_name = link.get('name')
        if link_name == 'world':
            continue
        
        # Get link transform
        link_transform = get_link_transform(link_name, root, joint_positions, transforms_cache)
        
        # Process all visual elements in the link
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            if geometry is None:
                continue
            
            mesh_elem = geometry.find('mesh')
            if mesh_elem is None:
                continue
            
            mesh_filename = mesh_elem.get('filename')
            if not mesh_filename:
                continue
            
            # Resolve mesh path
            mesh_path = resolve_package_path(mesh_filename, urdf_dir)
            
            # Get visual origin transform
            visual_origin = visual.find('origin')
            if visual_origin is not None:
                visual_transform = parse_origin(visual_origin)
            else:
                visual_transform = np.eye(4)
            
            # Combine transforms
            final_transform = link_transform @ visual_transform
            
            # Load and transform mesh
            mesh = load_mesh_with_transform(mesh_path, final_transform)
            if mesh is not None:
                meshes.append(mesh)
                print(f"Loaded mesh: {mesh_filename} for link: {link_name}")
    
    if not meshes:
        print("Error: No meshes were loaded")
        sys.exit(1)
    
    print(f"\nLoaded {len(meshes)} mesh(es)")
    
    # Export to STL
    print(f"\nExporting to STL: {output_path}")
    success = export_to_stl(meshes, output_path)
    
    if success:
        print(f"Successfully exported STL file: {output_path}")
    else:
        print("Failed to export STL file. Check errors above.")
        sys.exit(1)


if __name__ == '__main__':
    main()
