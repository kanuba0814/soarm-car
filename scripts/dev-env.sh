#!/usr/bin/env bash
# Source this file before Docker Compose so GUI/GPU device groups match the host.
set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export HOSTNAME="${HOSTNAME:-$(hostname)}"

if [[ ! -S /tmp/.X11-unix/X${DISPLAY#*:} ]]; then
    echo "X11/XWayland socket for DISPLAY=$DISPLAY was not found." >&2
    return 1 2>/dev/null || exit 1
fi
if [[ ! -r "$XAUTHORITY" ]]; then
    echo "Xauthority file is not readable: $XAUTHORITY" >&2
    return 1 2>/dev/null || exit 1
fi
intel_render_node=/dev/dri/renderD128
if [[ ! -c "$intel_render_node" ]]; then
    echo "$intel_render_node is not an available Intel render node." >&2
    return 1 2>/dev/null || exit 1
fi

export RENDER_GID="$(stat -c %g "$intel_render_node")"
