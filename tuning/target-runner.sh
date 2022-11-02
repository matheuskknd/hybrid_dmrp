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

# Fixed parameters
FIXED_PARAMS="--quiet"

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
    mkdir "tuning/output" >"/dev/null" 2>&1 || [ -d "tuning/output" ] || error "$MODULE: error creating output directory."
fi

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

# Create an auxiliar stderr FIFO pipe
FILE_NAME="$PDW/tuning/output/c${CONFIG_ID}-${INSTANCE_ID}-${SEED}"

mkfifo "$FILE_NAME.pipe"
{ STDERR=$(cat <"$FILE_NAME.pipe"); printf "$STDERR" >"$FILE_NAME.pipe"; } &

# Effectivelly execute the program
STDOUT=$(python -m "$MODULE" "${INSTANCE//\?/ }" $FIXED_PARAMS --seed "$SEED" $CONFIG_PARAMS 2>"$FILE_NAME.pipe")

STDERR=$(cat <"$FILE_NAME.pipe")
unlink "$FILE_NAME.pipe"

# Check the error output is empty
if [ -n "$STDERR" ] ; then
    printf "$STDOUT" >"$FILE_NAME.stdout"
    printf "$STDERR" >"$FILE_NAME.stderr"
    error "Configuration '$FILE_NAME' error"
fi

OUTPUT_LAST_LINE=$(printf "$STDOUT" | tail --lines=1)
COST=$(printf "$OUTPUT_LAST_LINE" | cut -d' ' -f1)
TIME=$(printf "$OUTPUT_LAST_LINE" | cut -d' ' -f2)

if ! [[ "$COST" =~ ^[-+0-9.e]+$ ]] ; then
    printf "$STDOUT" >"$FILE_NAME.stdout"
    printf "$STDERR" >"$FILE_NAME.stderr"
    error "Configuration '$FILE_NAME': Output cost is not a number"
fi

if ! [[ "$TIME" =~ ^[-+0-9.e]+$ ]] ; then
    printf "$STDOUT" >"$FILE_NAME.stdout"
    printf "$STDERR" >"$FILE_NAME.stderr"
    error "Configuration '$FILE_NAME': Output time is not a number"
fi

# Print the result!
printf "$COST $TIME"

# We are done with our duty. Exit with 0 (no error).
exit 0
