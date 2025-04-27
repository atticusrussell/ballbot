# cleanup_onshape_output.py
import sys
import re
import os
# import glob # Not needed for this version

def cleanup_urdf_content_selective(filename):
    """
    Modifies URDF content in place.
    Cleans link/joint names/references by removing '__configuration_ANYWORD' part,
    but PRESERVES instance number suffixes like '_2', '_3'.
    Leaves mesh filenames untouched.
    """
    if not os.path.exists(filename):
        print(f"Error: URDF file not found: {filename}")
        return False
    try:
        print(f"Processing URDF file selectively: {filename}")
        with open(filename, 'r') as f_in:
            content = f_in.read()
        original_content = content

        # --- Define patterns for cleaning ---
        # Pattern 1: Matches the full suffix with instance number (e.g., __configuration_default_2)
        # Captures: 1=full_suffix, 2=config_name, 3=instance_number
        pattern_with_instance = r'(__configuration_([a-zA-Z0-9_]+)_(\d+))'
        # Replacement 1: Keep only the underscore and instance number (e.g., _2)
        replacement_with_instance = r'_\3'

        # Pattern 2: Matches the suffix without instance number (e.g., __configuration_default)
        # Captures: 1=full_suffix, 2=config_name
        pattern_without_instance = r'(__configuration_([a-zA-Z0-9_]+))'
        # Replacement 2: Remove the suffix entirely
        replacement_without_instance = r''

        # --- Function to apply cleaning to specific attributes ---
        def apply_cleaning_to_attribute(attr_regex, text, value_group_index=1):
            """
            Applies the two cleaning patterns only within the captured group of attr_regex.
            attr_regex: The regex to find the attribute (e.g., <link name="VALUE">).
            text: The string to process.
            value_group_index: The index of the capturing group in attr_regex that holds the value to be cleaned.
            """
            new_text = text
            matches = list(re.finditer(attr_regex, text)) # Find all attribute matches first
            offset = 0 # Keep track of index changes due to replacements

            for match in matches:
                try:
                    # Get the original attribute value (e.g., the link name) using the specified group index
                    original_value = match.group(value_group_index)
                    # Calculate the start/end indices in the *current* state of new_text
                    start_index = match.start(value_group_index) + offset
                    end_index = match.end(value_group_index) + offset

                    # Apply cleaning patterns *only* to this specific value
                    # Pass 1: Handle suffixes with instance numbers FIRST
                    cleaned_value_pass1 = re.sub(pattern_with_instance, replacement_with_instance, original_value)
                    # Pass 2: Handle remaining suffixes without instance numbers
                    cleaned_value_final = re.sub(pattern_without_instance, replacement_without_instance, cleaned_value_pass1)

                    # If the value changed, replace it in the main string
                    if cleaned_value_final != original_value:
                        print(f"    Cleaning attribute value: '{original_value}' -> '{cleaned_value_final}'")
                        # Replace the value within the string slice
                        new_text = new_text[:start_index] + cleaned_value_final + new_text[end_index:]
                        # Update the offset for subsequent matches
                        offset += len(cleaned_value_final) - len(original_value)
                except IndexError:
                     print(f"Warning: Regex '{attr_regex}' did not find expected group {value_group_index} in match: {match.group(0)}")
                except Exception as e:
                    print(f"Warning: Error processing match {match.group(0)} for regex '{attr_regex}': {e}")


            return new_text

        # --- Apply cleaning selectively ---
        # 1. Link names: <link name="VALUE">
        print("  Cleaning link names...")
        content = apply_cleaning_to_attribute(r'<link\s+name="([^"]+)"', content, value_group_index=1)
        # 2. Joint names: <joint name="VALUE" ...>
        print("  Cleaning joint names...")
        content = apply_cleaning_to_attribute(r'<joint\s+name="([^"]+)"', content, value_group_index=1)
        # 3. Parent link references: <parent link="VALUE"/>
        print("  Cleaning parent links...")
        content = apply_cleaning_to_attribute(r'<parent\s+link="([^"]+)"\s*/>', content, value_group_index=1)
        # 4. Child link references: <child link="VALUE"/>
        print("  Cleaning child links...")
        content = apply_cleaning_to_attribute(r'<child\s+link="([^"]+)"\s*/>', content, value_group_index=1)
        # 5. Gazebo plugin joint references: <left_joint>VALUE</left_joint> (and right)
        #    Note: Value is in group 2 here
        print("  Cleaning gazebo plugin joints...")
        content = apply_cleaning_to_attribute(r'<(left_joint|right_joint)>([^<]+)</\1>', content, value_group_index=2)
        # 6. Gazebo plugin base frame: <robot_base_frame>VALUE</robot_base_frame>
        #    Note: Value is in group 2 here
        print("  Cleaning gazebo plugin base frame...")
        content = apply_cleaning_to_attribute(r'<(robot_base_frame)>([^<]+)</\1>', content, value_group_index=2)
         # 7. Gazebo reference tags: <gazebo reference="VALUE">
        print("  Cleaning gazebo reference links...")
        content = apply_cleaning_to_attribute(r'<gazebo\s+reference="([^"]+)"', content, value_group_index=1)


        # --- IMPORTANT: Do NOT modify <mesh filename="..."/> ---
        print("  Skipping mesh filenames.")

        # --- Write back if changed ---
        if content != original_content:
            print("Updating URDF content (links/joints/refs only, preserving instance numbers)...")
            with open(filename, 'w') as f_out:
                f_out.write(content)
            print("URDF selective content update successful.")
        else:
            print("No changes needed for URDF content (links/joints/refs).")
        return True

    except Exception as e:
        print(f"An error occurred processing URDF selectively {filename}: {e}")
        return False

# --- Asset File Renaming Function IS NOT USED ---

if __name__ == "__main__":
    # Expecting script name and urdf path only
    if len(sys.argv) != 2:
        print("Usage: python cleanup_onshape_output.py <path/to/robot.urdf>")
        sys.exit(1)

    urdf_file_path = sys.argv[1]
    # assets_directory_path is no longer needed or used

    print("-" * 20)
    # Call the modified function that ONLY changes URDF link/joint names selectively
    urdf_success = cleanup_urdf_content_selective(urdf_file_path)
    print("-" * 20)

    if urdf_success:
        print("URDF cleanup process completed successfully.")
        sys.exit(0)
    else:
        print("URDF cleanup process finished with errors.")
        sys.exit(1)
