#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_BAG="/home/liu/liu_workplace/consent_full_2.4.bag"
DEFAULT_OUT_DIR="${ROOT_DIR}/data_26_07_13"

usage() {
  cat <<'EOF'
Usage:
  ./prepare_bag_visuals.sh [bag_file] [output_dir]

Defaults:
  bag_file   = /home/liu/liu_workplace/consent_full_2.4.bag
  output_dir = /home/liu/robot_research_liu/data_visual/data_26_07_13
EOF
}

require_rostopic() {
  if command -v rostopic >/dev/null 2>&1; then
    return
  fi

  for setup in /opt/ros/*/setup.bash; do
    if [[ -f "${setup}" ]]; then
      # shellcheck disable=SC1090
      source "${setup}"
      if command -v rostopic >/dev/null 2>&1; then
        return
      fi
    fi
  done

  echo "rostopic not found. Please source your ROS environment first." >&2
  exit 1
}

export_topic() {
  local bag_file="$1"
  local topic="$2"
  local out_file="$3"

  rostopic echo -b "${bag_file}" -p "${topic}" > "${out_file}"
  if [[ ! -s "${out_file}" ]]; then
    echo "Export failed or empty output: ${topic}" >&2
    exit 1
  fi
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  local bag_file="${1:-${DEFAULT_BAG}}"
  local out_dir="${2:-${DEFAULT_OUT_DIR}}"

  if [[ ! -f "${bag_file}" ]]; then
    echo "Bag file not found: ${bag_file}" >&2
    exit 1
  fi

  require_rostopic

  mkdir -p "${out_dir}"

  echo "Exporting CSV from ${bag_file}"
  export_topic "${bag_file}" /pose_particle_localizer "${out_dir}/robot_traj.csv"
  export_topic "${bag_file}" /pose_person_following "${out_dir}/person_traj.csv"
  export_topic "${bag_file}" /actual_trajectory "${out_dir}/actual_trajectory.csv"

  echo "Generating plots into ${out_dir}"
  python3 "${SCRIPT_DIR}/traj_graph.py" --data-dir "${out_dir}" --no-show
  python3 "${SCRIPT_DIR}/person_traj.py" --data-dir "${out_dir}" --no-show
  python3 "${SCRIPT_DIR}/plot_robot_person_26_07_13_ver.py" --data-dir "${out_dir}" --no-show
  python3 "${SCRIPT_DIR}/robot_person.py" --data-dir "${out_dir}" --no-show

  echo "Done. Outputs are under: ${out_dir}"
}

main "$@"
