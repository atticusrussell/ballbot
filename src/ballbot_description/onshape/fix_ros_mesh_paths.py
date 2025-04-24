# fix_mesh_paths.py
# Purpose: Reads an input URDF, corrects mesh paths for ROS installation,
#          and writes the result to an output URDF file.
# Run via CMake during build process.
# Assumes link/joint names have already been cleaned.

import sys
import re
import os

def correct_mesh_paths(input_urdf_path, output_urdf_path, package_name, assets_install_subdir):
    """
    Reads input URDF, corrects mesh paths, writes to output URDF.
    Changes 'package://assets/...' to 'package://<pkg_name>/<assets_subdir>/...'
    """
    if not os.path.exists(input_urdf_path):
        print(f"Error: Input URDF file not found: {input_urdf_path}")
        return False
    try:
        print(f"Reading input URDF: {input_urdf_path}")
        with open(input_urdf_path, 'r') as f_in:
            content = f_in.read()
        original_content = content

        print(f"Correcting mesh paths to use package '{package_name}' and subdir '{assets_install_subdir}'...")
        # Find 'package://assets/...' and replace with 'package://<package_name>/<assets_subdir>/...'
        # NOTE: It looks for the *original* asset filenames (with suffixes) as they should be
        #       present in the input URDF passed to this script.
        mesh_path_pattern = r'package://assets/([^"]+)'
        correct_mesh_path_replacement = rf'package://{package_name}/{assets_install_subdir}/\1'
        content, num_mesh_subs = re.subn(mesh_path_pattern, correct_mesh_path_replacement, content)

        if num_mesh_subs > 0:
            print(f"  Corrected {num_mesh_subs} mesh paths.")
        else:
            print("  Warning: No mesh paths matching 'package://assets/...' found to correct.")
            # Check if paths are already correct?
            check_pattern = rf'package://{package_name}/{assets_install_subdir}/'
            if check_pattern in content:
                 print("  (Paths might already be in the correct format)")
            else:
                 print("  (Double-check input URDF mesh paths)")


        print(f"Writing processed URDF to: {output_urdf_path}")
        # Ensure output directory exists (useful for CMake build directory)
        os.makedirs(os.path.dirname(output_urdf_path), exist_ok=True)
        with open(output_urdf_path, 'w') as f_out:
            f_out.write(content)
        print("Mesh path correction successful.")
        return True

    except Exception as e:
        print(f"An error occurred correcting mesh paths from {input_urdf_path} to {output_urdf_path}: {e}")
        return False

if __name__ == "__main__":
    # Expecting script name, input urdf, output urdf, package name, assets install subdir
    if len(sys.argv) != 5:
        print("Usage: python fix_mesh_paths.py <input_urdf> <output_urdf> <package_name> <assets_install_subdir>")
        print("Example: python fix_mesh_paths.py build/robot_raw.urdf build/robot_ros.urdf my_pkg meshes")
        sys.exit(1)

    input_urdf = sys.argv[1]
    output_urdf = sys.argv[2]
    ros_package_name = sys.argv[3]
    assets_subdir = sys.argv[4]

    print("-" * 20)
    success = correct_mesh_paths(input_urdf, output_urdf, ros_package_name, assets_subdir)
    print("-" * 20)

    if success:
        print("Mesh path fixing process completed successfully.")
        sys.exit(0)
    else:
        print("Mesh path fixing process finished with errors.")
        sys.exit(1)

