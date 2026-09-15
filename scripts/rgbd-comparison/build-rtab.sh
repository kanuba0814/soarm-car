#!/usr/bin/env bash
set -euo pipefail
c++ -std=c++17 -O3 /comparison/rtab.cc -o /results/rtab-runner \
 -I/opt/ros/humble/include/rtabmap-0.23 -I/usr/include/opencv4 -I/usr/include/eigen3 -I/usr/include/pcl-1.12 \
 -L/opt/ros/humble/lib/x86_64-linux-gnu -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu \
 -lrtabmap_core -lrtabmap_utilite $(pkg-config --libs opencv4)
