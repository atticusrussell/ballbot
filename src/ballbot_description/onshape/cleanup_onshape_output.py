# cleanup_onshape_output.py
import sys
import re
import os
import glob

def cleanup_urdf_content(filename):
    """Modifies URDF content in place."""
    if not os.path.exists(filename):
        print(f"Error: URDF file not found: {filename}")
        return False
    try:
        print(f"Processing URDF file: {filename}")
        with open(filename, 'r') as f_in:
            content = f_in.read()

        # --- Apply substitutions ---
        # 1. Handle names with instance numbers first
        pattern1 = r'__configuration_([a-zA-Z0-9_]+)_(\d+)'
        replacement1 = r'_\2'
        content_step1 = re.sub(pattern1, replacement1, content)

        # 2. Handle names without instance numbers
        pattern2 = r'__configuration_([a-zA-Z0-9_]+)'
        replacement2 = r'' # Replace with empty string
        new_content = re.sub(pattern2, replacement2, content_step1)
        # --- Substitutions done ---

        if new_content != content:
            print("Updating URDF content...")
            with open(filename, 'w') as f_out:
                f_out.write(new_content)
            print("URDF content update successful.")
        else:
            print("No changes needed for URDF content.")
        return True

    except Exception as e:
        print(f"An error occurred processing URDF {filename}: {e}")
        return False

def rename_asset_files(asset_dir):
    """Renames asset files in the specified directory."""
    if not os.path.isdir(asset_dir):
        print(f"Error: Asset directory not found: {asset_dir}")
        return False
    try:
        print(f"Processing asset files in: {asset_dir}")
        # Pattern to find in filenames
        pattern = r'__configuration_[a-zA-Z0-9_]+'
        replacement = r'' # Replace with empty string

        # Using glob to find all potentially relevant files
        # Adjust pattern if other file types need renaming (.part, .scad etc.)
        files_to_check = glob.glob(os.path.join(asset_dir, '*__configuration_*.*'))

        renamed_count = 0
        for old_filepath in files_to_check:
            if not os.path.isfile(old_filepath):
                continue

            directory, old_filename = os.path.split(old_filepath)
            new_filename = re.sub(pattern, replacement, old_filename)

            if new_filename != old_filename:
                new_filepath = os.path.join(directory, new_filename)
                print(f"  Renaming '{old_filename}' -> '{new_filename}'")
                os.rename(old_filepath, new_filepath)
                renamed_count += 1

        if renamed_count > 0:
            print(f"Successfully renamed {renamed_count} asset files.")
        else:
            print("No asset files needed renaming.")
        return True

    except Exception as e:
        print(f"An error occurred renaming files in {asset_dir}: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 3:
        # Expecting script name, urdf path, asset dir path
        print("Usage: python cleanup_onshape_output.py <path/to/robot.urdf> <path/to/assets_dir>")
        sys.exit(1)

    urdf_file_path = sys.argv[1]
    assets_directory_path = sys.argv[2]

    print("-" * 20)
    urdf_success = cleanup_urdf_content(urdf_file_path)
    print("-" * 20)
    assets_success = rename_asset_files(assets_directory_path)
    print("-" * 20)

    if urdf_success and assets_success:
        print("Cleanup process completed successfully.")
        sys.exit(0)
    else:
        print("Cleanup process finished with errors.")
        sys.exit(1)
