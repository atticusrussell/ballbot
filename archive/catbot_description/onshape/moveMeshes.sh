#!/bin/bash

robot_name="catbot"

# Source and destination directories
src_dir="./$robot_name"
dest_dir="../meshes"

# Create destination directory if it doesn't exist
mkdir -p "$dest_dir"

# Create subdirectories for "collision" and "visual" if they don't exist
mkdir -p "$dest_dir/collision"
mkdir -p "$dest_dir/visual"

# Find all STL files with "assm" in the name
find "$src_dir" -name '*assm*.stl' -print0 | while IFS= read -r -d '' file; do
    # Check if filename contains "collision" or "visual" and copy to corresponding subdirectory
    if [[ $file == *"collision"* ]]; then
        cp "$file" "$dest_dir/collision"
    elif [[ $file == *"visual"* ]]; then
        cp "$file" "$dest_dir/visual"
    else
        cp "$file" "$dest_dir"
    fi
done

# new name for urdf file
urdf_path="../urdf/$robot_name/"
urdf_suffix="_macro.xacro"
urdf_file="$robot_name$urdf_suffix"
urdf_file_path="$urdf_path$urdf_file"

# Copy robot.urdf to the appropriate subdirectory
mkdir -p "$urdf_path"
cp "$src_dir/robot.urdf" "$urdf_file_path"

# Change paths in copied URDF file to match new directory structure
sed -i 's#<mesh filename="package://\(.*\)/\(.*\)_visual.stl"/>#<mesh filename="package://\1/visual/\2_visual.stl"/>#' "$urdf_file_path"
sed -i 's#<mesh filename="package://\(.*\)/\(.*\)_collision.stl"/>#<mesh filename="package://\1/collision/\2_collision.stl"/>#' "$urdf_file_path"

# Replace "revolute" type joints with "continuous" in joints where the name contains "wheel"
sed -i '/<joint name=".*wheel.*" type="revolute">/s/revolute/continuous/' "$urdf_file_path"

# Replace the first line of the file with the specified content
sed -i '1c <?xml version="1.0" encoding="UTF-8"?>\n<robot xmlns:xacro="http://wiki.ros.org/xacro">\n\n  <xacro:macro name="catbot" params="prefix parent *origin">\n\n    <!-- base_joint fixes base_link to the environment -->\n\n    <joint name="${prefix}base_joint" type="fixed">\n      <xacro:insert_block name="origin" />\n      <parent link="${parent}" />\n      <child link="${prefix}base_link" />\n    </joint>' "$urdf_file_path"

# Remove the last three lines
head -n -3 "$urdf_file_path" > temp_file && mv temp_file "$urdf_file_path"

# Append the specified content to the end of the file
echo "  </xacro:macro>" >> "$urdf_file_path"
echo "</robot>" >> "$urdf_file_path"
echo "" >> "$urdf_file_path"


# format the URDF file
# indent
tidy -xml -i -quiet -o $urdf_file_path $urdf_file_path
# re-add  newlines
python3 urdf_newlines.py $urdf_file_path
