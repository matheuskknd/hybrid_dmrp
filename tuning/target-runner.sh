#!/usr/bin/env bash
###############################################################################
# Script to run the python module.
#
# PARAMETERS:
# $1 is the ID of the candidate to be evaluated
# $2 is the instance ID
# $3 is the seed
# $4 is the instance name
# The rest ($* after `shift 4') are parameters for running the module
#
# RETURN VALUE:
# This script should print a single numerical value (the value to be minimized).
###############################################################################

function error() {
    echo -e "$(TZ=UTC+3 date): $0: error: $@" >&2
    exit 1
}

CONFIG_ID="$1"
INSTANCE_ID="$2"
SEED="$3"
INSTANCE="$4"

# All other parameters are the candidate parameters to be passed to program
shift 4 || error "Not enough parameters"
CONFIG_PARAMS=$*

# Go to parent directory
cd ..
PDW="$(pwd)"

# Path to the module (module name):
MODULE="hybrid_dmrp"

if [ ! -d "$MODULE" ]; then
    error "$MODULE: not found (pwd: $PWD)"
fi

# Create an empty folder to store the logs
if [ ! -d "tuning/output" ]; then
    mkdir "tuning/output"
else
    rm -rf "tuning/output"
    mkdir "tuning/output"
fi

STDOUT="$PDW/tuning/output/c${CONFIG_ID}-${INSTANCE_ID}-${SEED}.stdout"
STDERR="$PDW/tuning/output/c${CONFIG_ID}-${INSTANCE_ID}-${SEED}.stderr"

# Check for python3.10 installation
if ! { command -V "python3.10" >"/dev/null" 2>&1; }; then
    error "No python3.10 installation found"
fi

# Check for the ".venv" folder
if [ ! -d ".venv" ]; then
    python3.10 -m venv .venv || error "Python .venv creation failed"
fi

# Enter the virtual environment
if [ -f ".venv/Scripts/activate" ]; then
    source .venv/Scripts/activate || error "Failed: source .venv/Scripts/activate"
elif [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate || error "Failed: source .venv/bin/activate"
else
    error "Could not found .venv activation script"
fi

# Check the dependencies
if ! { python -m pip install -r requirements.txt --no-index >"/dev/null" 2>&1; }; then
    error "Ensure all dependencies are installed then try again"
fi

# Effectivelly execute the program
python -m "$MODULE" "${INSTANCE//\?/ }" --seed "$SEED" $CONFIG_PARAMS 1>"$STDOUT" 2>"$STDERR"

# Check if the output file exists
if [ ! -s "$STDOUT" ]; then
    error "$STDOUT: No such file"
fi

OUTPUT_LAST_LINE=$(tail --lines=1 "$STDOUT")
COST=$(echo "$OUTPUT_LAST_LINE" | cut -d' ' -f1)
TIME=$(echo "$OUTPUT_LAST_LINE" | cut -d' ' -f2)

if ! [[ "$COST" =~ ^[-+0-9.e]+$ ]] ; then
    error "${STDOUT}: Output cost is not a number"
fi

if ! [[ "$TIME" =~ ^[-+0-9.e]+$ ]] ; then
    error "${TIME}: Output time is not a number"
fi

# Print the result!
echo "$COST $TIME"

# We are done with our duty. Clean files and exit with 0 (no error).
rm -f "${STDOUT}" "${STDERR}"
rm -f best.* stat.* cmp.*
exit 0
