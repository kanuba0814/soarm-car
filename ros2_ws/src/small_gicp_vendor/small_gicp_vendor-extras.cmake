# The committed upstream subproject installs its CMake package in this same prefix.
# ament includes CONFIG_EXTRAS before importing export_small_gicp_vendor.
include("${CMAKE_CURRENT_LIST_DIR}/../../../lib/cmake/small_gicp/small_gicp-config.cmake")
