#!/usr/bin/env bash
# Image-build only. Never run this script directly on the host.
set -euo pipefail
target_uid="${1:?UID required}"
target_gid="${2:?GID required}"
[[ "$target_uid" =~ ^[0-9]+$ && "$target_gid" =~ ^[0-9]+$ ]] || { echo 'UID/GID must be integers.' >&2; exit 2; }
(( target_uid > 0 && target_gid > 0 )) || { echo 'The player must use a non-root UID/GID.' >&2; exit 2; }
uid_owner="$(getent passwd "$target_uid" | cut -d: -f1 || true)"
if [[ -n "$uid_owner" && "$uid_owner" != ubuntu ]]; then
    echo "UID $target_uid is already owned by $uid_owner in the base image; refusing to rename a system account." >&2
    exit 2
fi
target_group="$(getent group "$target_gid" | cut -d: -f1 || true)"
if [[ -z "$target_group" ]]; then
    if getent group ubuntu >/dev/null; then
        groupmod -g "$target_gid" ubuntu
    else
        groupadd -g "$target_gid" ubuntu
    fi
    target_group=ubuntu
fi
if id ubuntu >/dev/null 2>&1; then
    usermod -u "$target_uid" -g "$target_group" -d /home/ubuntu -s /bin/bash ubuntu
else
    useradd --no-log-init -m -u "$target_uid" -g "$target_group" -d /home/ubuntu -s /bin/bash ubuntu
fi
mkdir -p /home/ubuntu /exports
[[ -f /home/ubuntu/.bashrc ]] || cp /etc/skel/.bashrc /home/ubuntu/.bashrc
cat >> /home/ubuntu/.bashrc <<'BASHRC'

# Native ROS and the installed ConPR++ player for docker exec ... bash.
source /etc/profile.d/conpr-player.sh
BASHRC
chown -R "ubuntu:$target_group" /home/ubuntu
chown "ubuntu:$target_group" /exports
test "$(id -u ubuntu)" = "$target_uid"
test "$(id -g ubuntu)" = "$target_gid"
