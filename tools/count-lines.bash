#!/usr/bin/env bash

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

EXCLUDE_DIRS='^(build/|html/|latex/|install/|log/|thirdparty/|LIBRA-ROS2-Tools/|lightwarelidar2/|realsense-ros/)'
EXCLUDE_FILES='(Doxyfile\.txt)'
INCLUDE_EXT='\.(bash|cpp|h|hrdf|ino|md|msg|py|txt|ui|urdf|xacro|xml|yaml)$'

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

echo "Counting all lines in user-generated project files..."
sleep 1;  # let user see this message

# ==================== (1) LOOP THROUGH FILES TRACKED BY GIT ====================

#-------------------------
START_TIME=$(date +%s.%N)
SLEEP_TIME=0
#-------------------------
color_echo INFO "Checking files tracked by git..."
sleep 1
SLEEP_TIME=$(echo "$SLEEP_TIME + 1" | bc)

total=0
while read -r f; do
  if [ -f "$f" ]; then
    count=$(wc -l < "$f")
    printf "  %5d %s\n" "$count" "$f"
    total=$((total + count))
  fi
done < <(
  git ls-files \
    | grep -Ev "$EXCLUDE_DIRS" \
    | grep -Ev "$EXCLUDE_FILES" \
    | grep -E  "$INCLUDE_EXT" \
    | sort
)
echo "  -----"
printf "  %5d TOTAL\n" "$total"

color_echo INFO "Checking files tracked by git... SUCCESS"
#-------------------------

# ==================== SCRIPT END ====================

END_TIME=$(date +%s.%N)
ELAPSED_TIME=$(echo "$END_TIME - $START_TIME - $SLEEP_TIME" | bc -l)
F_ELAPSED_TIME=$(printf "Script finished in %.2f s" "$ELAPSED_TIME")
color_echo INFO "$F_ELAPSED_TIME"
