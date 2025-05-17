#!/bin/bash

# Exit immediately on error
set -e

# ---- CONFIG ----
ENV_DIR="src/ballbot_description/onshape"
VENV_NAME="venv"
REQUIREMENTS_FILE="requirements.txt"
PYTHON_COMMAND="onshape-to-robot ballbot_generated"
# ----------------

echo "Changing to environment directory: $ENV_DIR"
cd "$ENV_DIR"

if [ ! -d "$VENV_NAME" ]; then
    echo "Virtual environment '$VENV_NAME' not found. Creating it..."
    python3 -m venv "$VENV_NAME"
else
    echo "Virtual environment '$VENV_NAME' already exists. Reusing it."
fi

echo "Activating virtual environment"
source "$VENV_NAME/bin/activate"

echo "Installing requirements from $REQUIREMENTS_FILE"
pip install --upgrade pip
pip install -r "$REQUIREMENTS_FILE"

echo "Running Python command: $PYTHON_COMMAND"
$PYTHON_COMMAND

echo "Done."

