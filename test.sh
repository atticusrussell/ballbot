#!/bin/bash
set -e

# --- Configuration ---
SKIP_PACKAGES_FILE=".ci_skip_packages"
declare -a skip_packages=() # Declare an empty array

# --- Read and Parse the Skip File ---
if [ -f "$SKIP_PACKAGES_FILE" ]; then
    echo "Reading packages to skip from '$SKIP_PACKAGES_FILE'..."
    # Read file line by line, trim comments/whitespace, add non-empty to array
    while IFS= read -r line || [[ -n "$line" ]]; do
        # Remove comments starting with # and any leading/trailing whitespace
        pkg=$(echo "$line" | sed 's/#.*//' | xargs)
        # Add to array if not empty
        if [[ -n "$pkg" ]]; then
            skip_packages+=("$pkg")
        fi
    done < "$SKIP_PACKAGES_FILE"

    if [ ${#skip_packages[@]} -gt 0 ]; then
        echo "Will skip tests for mandatory check: ${skip_packages[*]}"
    else
        echo "No valid packages found in '$SKIP_PACKAGES_FILE' to skip."
    fi
else
    echo "Skip file '$SKIP_PACKAGES_FILE' not found. No packages will be skipped."
fi

# --- Build colcon arguments dynamically ---
declare -a skip_colcon_args=()
declare -a select_colcon_args=()
if [ ${#skip_packages[@]} -gt 0 ]; then
    # Arguments for the main run (skip these packages)
    skip_colcon_args=(--packages-skip "${skip_packages[@]}")
    # Arguments for the optional run (select only these packages)
    select_colcon_args=(--packages-select "${skip_packages[@]}")
fi

# --- Sourcing Environment ---
echo "Sourcing ROS environment..."
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "Error: install/setup.bash not found."
  exit 1
fi

# --- Main Test Run ---
echo "Running mandatory tests (skipping packages listed in $SKIP_PACKAGES_FILE)..."
# Use array expansion for arguments: "${skip_colcon_args[@]}"
# If the array is empty, this expands to nothing.
colcon test --merge-install "${skip_colcon_args[@]}"
MAIN_TEST_EXIT_CODE=$?
if [ $MAIN_TEST_EXIT_CODE -ne 0 ]; then
    echo "Mandatory package tests failed!"
    exit $MAIN_TEST_EXIT_CODE
fi
echo "Mandatory package tests passed."

# --- Optional Skipped Package Test Run ---
# Only run this part if there were actually packages to skip/select
if [ ${#select_colcon_args[@]} -gt 0 ]; then
    echo "Optionally running skipped package tests (failures will be reported but ignored)..."
    ( # Start subshell
      # Use array expansion for arguments: "${select_colcon_args[@]}"
      colcon test --merge-install "${select_colcon_args[@]}"
      SUBMODULE_EXIT_CODE=$? # Capture the exit code IMMEDIATELY
      echo "-----------------------------------------------------"
      echo "Optional package(s) test exit code: $SUBMODULE_EXIT_CODE (0=success, non-zero=failure)"
      echo "Packages tested: ${skip_packages[*]}"
      echo "-----------------------------------------------------"
      exit 0 # Ensure subshell always exits successfully
    ) # End subshell
else
    echo "No optional packages were specified to run separately."
fi

# --- Final Report ---
echo "Displaying all test results (ignoring exit code of test-result)..."
# Add '|| true' here to prevent its exit code from failing the script via set -e
colcon test-result --verbose || true

echo "Test script finished successfully (ignoring potential failures in skipped packages)."
exit 0