#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash

# The image build creates this workspace; container startup only loads it.
if [ -f /workspaces/soarm-car/install/setup.bash ]; then
    source /workspaces/soarm-car/install/setup.bash
fi

exec "$@"
