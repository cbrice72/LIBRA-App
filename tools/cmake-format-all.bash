#!/usr/bin/env bash

# Assume project root is one directory above this script
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Explanation:
#   "dirname "${BASH_SOURCE[0]}"" gives the path to the directory this script is in
#   "cd ... && pwd" changes to the specified directory and gets its absolute path

# Path to cmake-format config file
CONFIG_FILE="${PROJECT_ROOT}/cmake-format.yaml"

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "[ERROR] Configuration file (cmake-format.yaml) not found in project root!"
  exit 1
fi

# Find all "CMakeLists.txt" (excluding third-party) and run cmake-format on each
CMAKE_FILES=$(find "$PROJECT_ROOT" -path "$PROJECT_ROOT/thirdparty" -prune -o -type f -name "CMakeLists.txt" -print)
# Explanation:
#   "-path ..." matches a directory, and "-prune" tells it to not descend into the matched directory.
#   "-o" is logical OR, so if the directory isn't matched, proceed to the next search.
#   "-type f -name ..." matches any files with the specified name.
#   "-print" ensures that the path is saved only if the necessary match clause is met.

for FILE in $CMAKE_FILES; do
  echo "Formatting: $FILE"
  cmake-format -c "$CONFIG_FILE" -i "$FILE"
done

echo "--- Finished ---"
