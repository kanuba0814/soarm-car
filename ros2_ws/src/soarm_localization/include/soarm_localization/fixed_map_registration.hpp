#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <small_gicp/registration/registration_helper.hpp>
#include <limits>
#include <memory>
#include <vector>

namespace soarm_localization {

struct Se2 { double x = 0.0, y = 0.0, yaw = 0.0; };

struct SearchProfile {
  double xy_extent = 0.25;
  double yaw_extent = 0.15;
  double xy_step = 0.02;
  double yaw_step = 0.02;
  double nms_radius = 0.08;
  std::size_t peak_count = 4;
};

// max_cells is the bounded per-source-cloud limit retained for API compatibility.
// Map-model storage is independently limited by the fixed byte budgets below.
struct Options {
  double voxel_resolution = 0.10;
  double max_correspondence_distance = 0.75;
  int min_cells = 20;
  int max_cells = 1200;
  int max_iterations = 30;
  int threads = 1;
  double distance_field_resolution = 0.01;
  double capped_squared_loss = 0.04;
  // Model-only resource limit; default is the core's conservative 128 MiB cell allowance.
  std::size_t max_model_cells = 524288;
  std::size_t max_model_bytes = 128U * 1024U * 1024U;
  std::size_t max_field_payload_bytes = 64U * 1024U * 1024U;
};

struct RefinementProfile {
  double max_relative_x = 0.10;
  double max_relative_y = 0.10;
  double max_relative_yaw = 0.10;
};

struct Peak { Se2 pose; double score = std::numeric_limits<double>::infinity(); };
struct SearchMetrics { std::size_t source_cells = 0, lattice_evaluations = 0; double wall_seconds = 0.0; };
struct PreRefinementResult { bool accepted_input = false; std::vector<Peak> peaks; double score_gap = std::numeric_limits<double>::infinity(); SearchMetrics metrics; };
struct ModelStats {
  std::size_t map_cells = 0;
  std::size_t tiles = 0;
  std::size_t field_payload_bytes = 0;
  std::size_t build_count = 0;
  double build_seconds = 0.0;
};

struct Observability {
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  double min_trace_ratio = 0.0;
  double condition_number = std::numeric_limits<double>::infinity();
  int valid_support = 0;
  int rank_for_relative_tolerance(double relative_tolerance) const;
};
struct Result {
  bool accepted_input = false, converged = false;
  bool bounded_refinement_used = false, trust_region_limited = false;
  bool all_evaluations_within_bounds = true;
  std::size_t bounded_evaluations = 0;
  Se2 transform;
  double mean_squared_error = 0.0, inlier_fraction = 0.0, wall_seconds = 0.0;
  Observability observability;
};

class FixedMapRegistration {
 public:
  explicit FixedMapRegistration(Options options = {});
  bool set_map(const std::vector<Eigen::Vector3d>& points);
  std::size_t cell_count() const;
  ModelStats model_stats() const;
  // Testable model seams: exact cap semantics and canonical-index tie breaking.
  double field_squared_distance(double x, double y) const;
  std::size_t nearest_cell_index(double x, double y) const;
  PreRefinementResult search(const std::vector<Eigen::Vector3d>& source, const SearchProfile& profile) const;
  // Candidate lattice deltas are composed onto map_anchor; returned peak poses are map-frame poses.
  PreRefinementResult search_around(const std::vector<Eigen::Vector3d>& source_local, const Se2& map_anchor,
                                    const SearchProfile& profile) const;
  std::vector<Result> refine_selected(const std::vector<Eigen::Vector3d>& source, const std::vector<Peak>& selected) const;
  Result refine(const std::vector<Eigen::Vector3d>& scan, const Se2& initial) const;
  Result refine_bounded(const std::vector<Eigen::Vector3d>& scan, const Se2& initial,
                        const RefinementProfile& profile) const;
  // initial_delta and every bounded correction are in map_anchor's local SE(2) frame.
  // The returned transform is map-frame: map_anchor composed with the final local delta.
  Result refine_bounded_around(const std::vector<Eigen::Vector3d>& source_local, const Se2& map_anchor,
                               const Se2& initial_delta, const RefinementProfile& profile) const;

 private:
  struct Model;
  double field_at(double x, double y) const;
  Options options_;
  std::shared_ptr<const Model> model_;
  std::size_t build_count_ = 0;
  double last_build_seconds_ = 0.0;
};
}  // namespace soarm_localization
