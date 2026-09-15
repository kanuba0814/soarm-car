#!/bin/bash
# Build runner2 against the prebuilt ORB_SLAM3 tree inside the ros2-dev image.
set -euo pipefail
base=/workspaces/soarm-car/vendor/orb_slam3/ORB_SLAM3
cd "$base/build"
flags=$(sed -n 's/^CXX_INCLUDES = //p' CMakeFiles/rgbd_tum.dir/flags.make)
c++ -std=c++11 -O3 -DCOMPILEDWITHC11 $flags -c /da/runner2.cc -o /tmp/dc1-runner2.o
sed 's@CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o@/tmp/dc1-runner2.o@; s@-o ../Examples/RGB-D/rgbd_tum@-o /results/runner2@' CMakeFiles/rgbd_tum.dir/link.txt > /tmp/dc1-link2.sh
bash /tmp/dc1-link2.sh
