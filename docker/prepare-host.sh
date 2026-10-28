#!/usr/bin/env bash
set -euo pipefail
[[ $# == 1 ]] || { echo 'Usage: bash prepare-host.sh /path/to/ConPRplusplus' >&2; exit 2; }
(( EUID != 0 )) || { echo 'Run prepare-host.sh without sudo, as your desktop user.' >&2; exit 2; }
if [[ -f /.dockerenv ]]; then
    echo 'Run this script on the local desktop HOST, not inside a container.' >&2; exit 2
fi
command -v xauth >/dev/null || { echo 'Install xauth: sudo apt-get install xauth' >&2; exit 2; }
[[ -n "${DISPLAY:-}" ]] || { echo 'DISPLAY is empty. Use a terminal on the local desktop.' >&2; exit 2; }
case "$DISPLAY" in :*|unix:*) ;; *) echo 'This setup requires a local X11/XWayland display, not SSH X forwarding.' >&2; exit 2;; esac
data="$(realpath -e -- "$1")"
[[ -d "$data/Hall" || -d "$data/Office" || -d "$data/Quad" ]] || { echo "Dataset root must contain Hall, Office or Quad: $data" >&2; exit 2; }
[[ "$data" != *"'"* && "$data" != *$'\n'* ]] || { echo 'Dataset path contains an unsupported quote/newline.' >&2; exit 2; }
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
umask 077
mkdir -p .runtime exports/noetic exports/humble exports/jazzy
cookie="$(xauth nlist "$DISPLAY")"
[[ -n "$cookie" ]] || { echo 'No Xauthority cookie found for DISPLAY. Check the host XAUTHORITY environment.' >&2; exit 2; }
tmp_auth="$(mktemp .runtime/xauthority.XXXXXX)"
trap 'rm -f "$tmp_auth"' EXIT
printf '%s\n' "$cookie" | sed 's/^..../ffff/' | xauth -f "$tmp_auth" nmerge -
# Keep the inode: existing containers bind-mount this file across stop/start.
cat "$tmp_auth" > .runtime/xauthority
chmod 600 .runtime/xauthority
# .env is data read by Compose, not a shell script. Quote paths literally.
printf "LOCAL_UID=%s\nLOCAL_GID=%s\nPLAYER_DISPLAY='%s'\nCONPR_DATASET='%s'\n" \
    "$(id -u)" "$(id -g)" "$DISPLAY" "$data" > .env
echo "Prepared host display $DISPLAY and read-only dataset $data"
echo 'Next: sudo docker compose build noetic humble jazzy'
