#!/usr/bin/env bash

# Runs clang-tidy on project source (lib/, src/ only; no third-party) and
# reports diagnostics, minus accepted ones.
#
# Disclaimer: This script was primarily written by Claude Opus 5.5

# ==================== LOCAL VARIABLES AND FUNCTIONS ====================

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_ROOT="${PROJECT_ROOT}/build"

OWN_REGEX="^${PROJECT_ROOT}/(lib|src)/"
EXCLUDE_REGEX="/archive/|/thirdparty/|_autogen/"

PRESETS=()
N_JOBS="$(nproc)"

# Manually accepted warnings (format: "<file name>|<check>|<regex>")
ACCEPTED=(
  # Short names that read clearly in their context
  "libra2_calibration_json.cpp|readability-identifier-length|name '(p|b)'"
  "libra2_calibrator.cpp|readability-identifier-length|name '(p|r)'"
  "libra2_model.cpp|readability-identifier-length|name '(c|s)'"
  "libra2_motion_controller.cpp|readability-identifier-length|name '(a|b)'"
  "hebi_thread.cpp|readability-identifier-length|name '(v|d)'"
  "main_window.cpp|readability-identifier-length|name '(p|w|l)'"

  # Documented coefficients, LIBRA-I servo math, animation values
  "libra2_calibrator.cpp|readability-magic-numbers|^0\.75 is"
  "manip_controller.cpp|readability-magic-numbers|^180 is"
  "tank_widget.cpp|readability-magic-numbers|^(120|160|200|16) is"

  # uint8_t bit constants get promoted to int (water command is a char QByteArray)
  "water_controller.cpp|hicpp-signed-bitwise|."

  # Switches that shrink to one case under `#if LIBRA_VERSION`
  "water_controller.cpp|hicpp-multiway-paths-covered|switch \(side\)"
  "main_window.cpp|hicpp-multiway-paths-covered|switch \((mode|side|type)\)"

  # EPOS API takes `long*`
  "epos_thread.cpp|google-runtime-int|long t_pos"

  # Intentional: control loop only needs the MotionCommand part
  "hebi_thread.cpp|cppcoreguidelines-slicing|MotionCommand\(cmd\)"

  # Layouts nest, so removing one recurses
  "main_window.cpp|misc-no-recursion|RemoveUiElement"

  # ROS defines image encodings as C strings
  "camera_manager.cpp|cppcoreguidelines-pro-bounds-array-to-pointer-decay|image_encodings::"

  # Its commented-out alternative uses `cw_flow_`
  "libra2_motion_controller.cpp|readability-convert-member-functions-to-static|PredictiveCwCommand"

  # `exec()` is static, but `app.exec()` is clearer (Qt idiom)
  "main.cpp|readability-static-accessed-through-instance|app\.exec\(\)"

  # Tracked by `@todo` in code
  "libra2_motion_controller.cpp|readability-function-cognitive-complexity|'Update'"
  "main_window.cpp|readability-function-cognitive-complexity|'RemoveUiElement"
)

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

# Helper function for usage information
usage() {
  color_echo INFO "Usage: $0 [-p|--preset NAME]... [-j|--N_JOBS N]"
  color_echo INFO "  Presets default to every build/<preset> with a compile_commands.json"
}

# Helper function for running clang-tidy on one file
# (output goes to its own file so parallel runs don't write over each other)
tidy_one() {
  local file=$1
  local out
  out="${TIDY_OUT_DIR}/$(md5sum <<< "$file" | cut -c1-12).txt"
  # shellcheck disable=SC2086  # paths in TIDY_EXTRA_ARGS have no spaces
  clang-tidy -p "$TIDY_BUILD_DIR" --quiet --header-filter="$OWN_REGEX" \
    $TIDY_EXTRA_ARGS "$file" > "$out" 2> /dev/null
}

# =================== (1) PARSE COMMAND-LINE ARGS ====================

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--preset)
      if [[ $# -lt 2 ]]; then
        color_echo ERROR "Missing value for $1."
        usage
        exit 1
      fi
      PRESETS+=("$2")
      shift 2
      ;;
    -j|--N_JOBS)
      if [[ $# -lt 2 ]]; then
        color_echo ERROR "Missing value for $1."
        usage
        exit 1
      fi
      N_JOBS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      color_echo ERROR "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ ${#PRESETS[@]} -eq 0 ]]; then
  # No presets given: use every configured build
  for db in "$BUILD_ROOT"/*/compile_commands.json; do
    [[ -f "$db" ]] && PRESETS+=("$(basename "$(dirname "$db")")")
  done
fi
if [[ ${#PRESETS[@]} -eq 0 ]]; then
  color_echo ERROR "No compile_commands.json found in ${BUILD_ROOT}; configure a preset first."
  exit 1
fi

# =================== (2) FIND STANDARD LIBRARY HEADERS ====================

# Clang picks the newest GCC install it finds, not the compiler the project
# builds with; point it at the project's to avoid confusing diagnostics
CXX="$(jq -r '.[0].command // .[0].arguments[0]' \
  "${BUILD_ROOT}/${PRESETS[0]}/compile_commands.json" | awk '{print $1}')"
TIDY_EXTRA_ARGS=""

# Add C++ stdlib headers to clang-tidy's include path
while read -r dir; do
  TIDY_EXTRA_ARGS+=" --extra-arg=-isystem${dir}"
done < <(echo | "$CXX" -xc++ -E -v - 2>&1 \
  | sed -n '/#include <...> search starts here:/,/End of search list./p' \
  | grep '/c++/' | sed 's/^ *//')

# =================== (3) RUN CLANG-TIDY ====================

# Collect all output in a temporary file, then filter it in one pass
ALL_OUT="$(mktemp)"
trap 'rm -rf "$ALL_OUT" "$TIDY_OUT_DIR"' EXIT

for preset in "${PRESETS[@]}"; do
  TIDY_BUILD_DIR="${BUILD_ROOT}/${preset}"
  if [[ ! -f "${TIDY_BUILD_DIR}/compile_commands.json" ]]; then
    color_echo ERROR "No compile_commands.json in ${TIDY_BUILD_DIR}"
    exit 1
  fi

  # Only check this project's sources
  mapfile -t files < <(jq -r '.[].file' "${TIDY_BUILD_DIR}/compile_commands.json" \
    | grep -E "$OWN_REGEX" | grep -vE "$EXCLUDE_REGEX" | sort -u)
  color_echo INFO "${preset}: checking ${#files[@]} files..."

  # Run clang-tidy in parallel
  # (NOTE: xargs starts a new bash for each, so func + vars must be exported)
  TIDY_OUT_DIR="$(mktemp -d)"
  export -f tidy_one
  export TIDY_BUILD_DIR TIDY_OUT_DIR TIDY_EXTRA_ARGS OWN_REGEX
  printf '%s\n' "${files[@]}" | xargs -P "$N_JOBS" -I{} bash -c 'tidy_one "$1"' _ {}
  cat "$TIDY_OUT_DIR"/*.txt >> "$ALL_OUT" 2> /dev/null
  rm -rf "$TIDY_OUT_DIR"
done

# =================== (4) FILTER AND REPORT ====================

# Filtering runs as an inline Python script:
#  - Each diagnostic spans several lines (message, flagged code, notes) and must
#    be parsed into "path|check|message", which bash and awk handle poorly.
#  - Inlining the script keeps the tool to one file.

# Bash arrays can't be passed to another program, so send one line per entry
ACCEPTED_LIST="$(printf '%s\n' "${ACCEPTED[@]}")"
python3 - "$ALL_OUT" "$PROJECT_ROOT" "$OWN_REGEX" "$EXCLUDE_REGEX" \
  "$ACCEPTED_LIST" << 'EOF'
import collections
import re
import sys

out_path, root, own_regex, exclude_regex, accepted_text = sys.argv[1:6]
accepted = [line.split("|", 2) for line in accepted_text.splitlines() if line]
diag_re = re.compile(
    r"^(/[^:]+):(\d+):(\d+): (warning|error): (.*) \[([^\]]+)\]$")

# Split output into diagnostics, each with the source line it flags
diags = []
for line in open(out_path, encoding="utf-8", errors="replace"):
    line = line.rstrip("\n")
    match = diag_re.match(line)
    if match:
        diags.append({"header": line, "groups": match.groups(), "lines": []})
    elif diags and not re.match(r"^\d+ (warning|error)s? generated", line):
        diags[-1]["lines"].append(line)

# Same diagnostic can come from several presets, or from a header included by
# several files; keep the first, drop external and accepted ones
seen = set()
kept = []
n_accepted = 0
n_external = collections.Counter()
for diag in diags:
    path, _, _, level, message, checks = diag["groups"]
    if diag["header"] in seen:
        continue
    seen.add(diag["header"])

    if not re.search(own_regex, path) or re.search(exclude_regex, path):
        n_external[level] += 1
        continue

    source = diag["lines"][0] if diag["lines"] else ""
    if level == "warning" and any(
            path.endswith("/" + name) and check in checks.split(",")
            and (re.search(regex, message) or re.search(regex, source))
            for name, check, regex in accepted):
        n_accepted += 1
        continue
    kept.append(diag)

# Report what's left, then a count per check
counts = collections.Counter()
for diag in kept:
    print(diag["header"].replace(root + "/", ""))
    print("\n".join(diag["lines"]))
    counts[diag["groups"][5].split(",")[0]] += 1

print()
for check, n in counts.most_common():
    print(f"{n:5d}  {check}")
print(f"{sum(counts.values()):5d}  total ({n_accepted} accepted, "
      f"{sum(n_external.values())} in external code hidden)")
# Errors in external headers usually mean a broken include path, which hides
# real diagnostics
if n_external["error"]:
    print(f"\nNOTE: {n_external['error']} errors in external code; results may "
          "be incomplete (check include paths)")
sys.exit(1 if kept else 0)  # nonzero if anything is left to fix
EOF
