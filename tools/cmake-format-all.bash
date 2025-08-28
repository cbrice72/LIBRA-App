#!/usr/bin/env bash

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

# Assume project root is one directory above this script
# Explanation:
#   "dirname "${BASH_SOURCE[0]}"" gives the path to the directory this script is in
#   "cd ... && pwd" changes to the specified directory and gets its absolute path
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${PROJECT_ROOT}/cmake-format.yaml"

# Helper function for colorized output
color_echo() {
  local level=$1
  shift
  local msg="$*"

  case "$level" in
    INFO)
      # bold-cyan label + cyan message
      echo -e "\033[1;36m[INFO]\033[0;36m $msg\033[0m"
      ;;
    WARN)
      # bold-yellow label + yellow message
      echo -e "\033[1;33m[WARN]\033[0;33m $msg\033[0m"
      ;;
    ERROR)
      # bold-red label + red message
      echo -e "\033[1;31m[ERROR]\033[0;31m $msg\033[0m"
      ;;
    *)
      echo "[UNKNOWN] $msg"
      ;;
  esac
}

# ==================== SCRIPT START ====================

echo "Formatting all project CMakeLists files..."
sleep 1;  # let user see this message

# ==================== (1) VALIDATE CONFIG AND COLLECT FILES ====================

#-------------------------
START_TIME=$(date +%s.%N)
SLEEP_TIME=0

if [[ ! -f "$CONFIG_FILE" ]]; then
  color_echo ERROR "Configuration file (cmake-format.yaml) not found in project root!"
  exit 1
fi
#-------------------------
color_echo INFO "Globbing CMakeLists (excluding thirdparty)..."
sleep 1
SLEEP_TIME=$(echo "$SLEEP_TIME + 1" | bc)

# Explanation:
#   "-path ..." matches a directory, and "-prune" tells it to not descend into the matched directory.
#   "-o" is logical OR, so if the directory isn't matched, proceed to the next search.
#   "-type f -name ..." matches any files with the specified name.
#   "-print" ensures that the path is saved only if the necessary match clause is met.
CMAKE_FILES=$(find "$PROJECT_ROOT" -path "$PROJECT_ROOT/thirdparty" -prune -o -type f -name "CMakeLists.txt" -print)

color_echo INFO "Globbing CMakeLists (excluding thirdparty)... SUCCESS"
#-------------------------

# ==================== (2) FORMAT ALL FILES ====================

#-------------------------
color_echo INFO "Formatting CMakeLists..."
sleep 1
SLEEP_TIME=$(echo "$SLEEP_TIME + 1" | bc)

for FILE in $CMAKE_FILES; do
  echo "Formatting: $FILE"
  cmake-format -c "$CONFIG_FILE" -i "$FILE"
done

color_echo INFO "Formatting CMakeLists... SUCCESS"
#-------------------------

# ==================== SCRIPT END ====================

END_TIME=$(date +%s.%N)
ELAPSED_TIME=$(echo "$END_TIME - $START_TIME - $SLEEP_TIME" | bc -l)
F_ELAPSED_TIME=$(printf "Script finished in %.2f s" "$ELAPSED_TIME")
color_echo INFO "$F_ELAPSED_TIME"
