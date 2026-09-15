#!/bin/bash
set -euo pipefail
base=/workspaces/soarm-car/vendor/orb_slam3/ORB_SLAM3
cd "$base/build"
flags=$(sed -n 's/^CXX_INCLUDES = //p' CMakeFiles/rgbd_tum.dir/flags.make)
c++ -std=c++11 -O3 -DCOMPILEDWITHC11 $flags -c /validation/runner.cc -o /tmp/dc1-runner.o
sed 's@CMakeFiles/rgbd_tum.dir/Examples/RGB-D/rgbd_tum.cc.o@/tmp/dc1-runner.o@; s@-o ../Examples/RGB-D/rgbd_tum@-o /results/runner@' CMakeFiles/rgbd_tum.dir/link.txt > /tmp/dc1-link.sh
bash /tmp/dc1-link.sh
