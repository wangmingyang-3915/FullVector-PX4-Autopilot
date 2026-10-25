#!/usr/bin/env bash
# Start the local-only dashboard once and open it in the Jetson desktop browser.
set -Eeo pipefail

readonly base=/home/jetson/yahboom_ws/vision_dashboard
readonly url=http://127.0.0.1:8765
readonly state_dir=/home/jetson/.local/state/vision-dashboard
readonly relay=/home/jetson/yahboom_ws/vision_dashboard/relay_install/lib/vision_dashboard_image_relay/image_relay

if [[ ${1:-} != '' && ${1:-} != --background ]]; then
    printf 'Usage: %s [--background]\n' "$0" >&2
    exit 2
fi

mkdir -p "$state_dir"
export ROS_DOMAIN_ID=99
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/jetson/yahboom_ws/install/setup.bash

# This C++ relay continuously subscribes to every production image and forwards
# one unchanged frame per second to the Python UI. It avoids deserializing the
# 1280x1024, 60 Hz stream in Python while preserving continuous chain coverage.
if [[ -x "$relay" ]] && ! pgrep -f "^${relay}$" >/dev/null; then
    nohup nice -n 10 "$relay" >>"$state_dir/image-relay.log" 2>&1 </dev/null &
fi

healthy() {
    curl --noproxy '*' --silent --show-error --fail --max-time 1 "$url/api/health" 2>/dev/null | grep -q '"app": "vision-dashboard"'
}

if ! healthy; then
    nohup /usr/bin/python3 "$base/server.py" >>"$state_dir/server.log" 2>&1 </dev/null &

    for _ in {1..40}; do
        if healthy; then
            break
        fi
        sleep 0.25
    done
fi

if ! healthy; then
    printf '视觉监控平台未启动。请查看 %s/server.log\n' "$state_dir" >&2
    exit 1
fi

if [[ ${1:-} != --background ]]; then
    xdg-open "$url/" >/dev/null 2>&1
fi
