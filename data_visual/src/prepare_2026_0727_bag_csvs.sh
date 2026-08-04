#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_DATA_ROOT="/home/liu/liu_workplace/2026-0727-data"
DEFAULT_OUT_ROOT="${ROOT_DIR}/data_2026_0727"
EXCLUDED_TOPICS=(/scan)

usage() {
  cat <<'EOF'
Usage:
  ./prepare_2026_0727_bag_csvs.sh [data_root] [output_root]

Defaults:
  data_root  = /home/liu/liu_workplace/2026-0727-data
  output_root = /home/liu/robot_research_liu/data_visual/data_2026_0727

The output keeps the date, run number, and bag type, for example:
  data_2026_0727/2026-07-27/01/fformation/robot_traj.csv
  data_2026_0727/2026-07-27/01/manual/person_traj.csv

Every topic except /scan in each fformation.bag and manual.bag is exported to
one CSV.
The plotting-related CSV names remain robot_traj.csv, person_traj.csv, and
actual_trajectory.csv. No plots are generated.
EOF
}

require_ros_tools() {
  if command -v rostopic >/dev/null 2>&1 && command -v rosbag >/dev/null 2>&1; then
    return
  fi

  for setup in /opt/ros/*/setup.bash; do
    if [[ -f "${setup}" ]]; then
      # shellcheck disable=SC1090
      source "${setup}"
      if command -v rostopic >/dev/null 2>&1 && command -v rosbag >/dev/null 2>&1; then
        return
      fi
    fi
  done

  echo "ROS tools not found. Please source your ROS environment first." >&2
  exit 1
}

topic_to_filename() {
  local topic="$1"
  local topic_name

  case "${topic}" in
    /pose_particle_localizer)
      echo "robot_traj.csv"
      ;;
    /pose_person_following)
      echo "person_traj.csv"
      ;;
    /actual_trajectory)
      echo "actual_trajectory.csv"
      ;;
    *)
      topic_name="${topic#/}"
      topic_name="${topic_name//\//__}"
      topic_name="${topic_name//[^[:alnum:]_.-]/_}"
      echo "${topic_name}.csv"
      ;;
  esac
}

is_excluded_topic() {
  local topic="$1"
  local excluded_topic

  for excluded_topic in "${EXCLUDED_TOPICS[@]}"; do
    if [[ "${topic}" == "${excluded_topic}" ]]; then
      return 0
    fi
  done
  return 1
}

export_topic() {
  local bag_file="$1"
  local topic="$2"
  local out_file="$3"
  local tmp_file="${out_file}.tmp"

  echo "  ${topic} -> ${out_file}"
  if ! rostopic echo -b "${bag_file}" -p "${topic}" > "${tmp_file}"; then
    rm -f -- "${tmp_file}"
    echo "Export failed: ${topic} (${bag_file})" >&2
    return 1
  fi

  if [[ ! -s "${tmp_file}" ]]; then
    rm -f -- "${tmp_file}"
    echo "Export produced an empty CSV: ${topic} (${bag_file})" >&2
    return 1
  fi

  mv -- "${tmp_file}" "${out_file}"
}

process_bag() {
  local bag_file="$1"
  local data_root="$2"
  local out_root="$3"
  local relative_path run_dir bag_type out_dir bag_info topic out_name
  local -a topics=()
  local -A used_names=()
  local exported=0

  relative_path="${bag_file#"${data_root}/"}"
  run_dir="${relative_path%/*}"
  bag_type="$(basename "${bag_file}" .bag)"
  out_dir="${out_root}/${run_dir}/${bag_type}"

  echo "Processing: ${relative_path}"
  bag_info="$(rosbag info --yaml "${bag_file}")"
  mapfile -t topics < <(
    awk '/^[[:space:]]*- topic: / {print $3}' <<< "${bag_info}"
  )

  if (( ${#topics[@]} == 0 )); then
    echo "No topics found in bag: ${bag_file}" >&2
    return 1
  fi

  mkdir -p "${out_dir}"
  for topic in "${topics[@]}"; do
    if is_excluded_topic "${topic}"; then
      echo "  Skipping excluded topic: ${topic}"
      continue
    fi

    out_name="$(topic_to_filename "${topic}")"
    if [[ -n "${used_names[${out_name}]:-}" ]]; then
      echo "CSV filename collision: ${topic} and ${used_names[${out_name}]} -> ${out_name}" >&2
      return 1
    fi
    used_names["${out_name}"]="${topic}"
    export_topic "${bag_file}" "${topic}" "${out_dir}/${out_name}"
    ((exported += 1))
  done

  echo "  Exported ${exported} of ${#topics[@]} topics."
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  local data_root="${1:-${DEFAULT_DATA_ROOT}}"
  local out_root="${2:-${DEFAULT_OUT_ROOT}}"
  local bag_file
  local processed=0

  if [[ ! -d "${data_root}" ]]; then
    echo "Data directory not found: ${data_root}" >&2
    exit 1
  fi

  require_ros_tools
  mkdir -p "${out_root}"

  while IFS= read -r -d '' bag_file; do
    process_bag "${bag_file}" "${data_root}" "${out_root}"
    ((processed += 1))
  done < <(
    find "${data_root}" -mindepth 3 -maxdepth 3 -type f \
      \( -name 'fformation.bag' -o -name 'manual.bag' \) -print0 | sort -z
  )

  if (( processed == 0 )); then
    echo "No fformation.bag or manual.bag files found under: ${data_root}" >&2
    exit 1
  fi

  echo "Done. Processed ${processed} bag files."
  echo "CSV outputs are under: ${out_root}"
}

main "$@"
