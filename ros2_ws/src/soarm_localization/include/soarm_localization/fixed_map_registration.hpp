#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <small_gicp/registration/registration_helper.hpp>
#include <limits>
#include <memory>
#include <vector>

namespace soarm_localization {

struct Se2 { double x = 0.0, y = 0.0, yaw = 0.0; };

// Search settings are explicit caller/profile policy.  No acceptance cutoff is embedded here.
struct SearchProfile {
  double xy_extent = 0.25;
  double yaw_extent = 0.15;
  double xy_step = 0.02;
  double yaw_step = 0.02;
  double nms_radius = 0.08;
  std::size_t peak_count = 4;
};

struct Options {
  double voxel_resolution = 0.10;
  double max_correspondence_distance = 0.75;
  int min_cells = 20;
  int max_cells = 1200;
  int max_iterations = 30;
  int threads = 1;
  double distance_field_resolution = 0.01;
  double capped_squared_loss = 0.04;
};

// Bounds are inclusive.  A zero bound locks that relative SE(2) component.
struct RefinementProfile {
  double max_relative_x = 0.10;
  double max_relative_y = 0.10;
  double max_relative_yaw = 0.10;
};

struct Peak { Se2 pose; double score = std::numeric_limits<double>::infinity(); };
struct SearchMetrics { std::size_t source_cells = 0, lattice_evaluations = 0; double wall_seconds = 0.0; };
struct PreRefinementResult {
  bool accepted_input = false;
  std::vector<Peak> peaks;
  double score_gap = std::numeric_limits<double>::infinity();
  SearchMetrics metrics;
};

// Raw spectrum only; the caller supplies its own rank tolerance and policy.
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
  // True only for refine_bounded(); retained separately from convergence so caller policy stays external.
  bool bounded_refinement_used = false, trust_region_limited = false;
  // Diagnostic invariant for audited cache callers: all correspondence/error evaluations were bounded.
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
  // Scores the fixed map before any GICP invocation.  Output ordering is (score,x,y,yaw).
  PreRefinementResult search(const std::vector<Eigen::Vector3d>& source, const SearchProfile& profile) const;
  // Optional GICP refinement is deliberately separate from search: it cannot alter peak order/gaps.
  std::vector<Result> refine_selected(const std::vector<Eigen::Vector3d>& source, const std::vector<Peak>& selected) const;
  // Legacy unbounded small_gicp refinement. It is unsuitable for finite-cache guarantees.
  Result refine(const std::vector<Eigen::Vector3d>& scan, const Se2& initial) const;
  // Bounded planar refinement about initial. Every correspondence and LM trial is checked first.
  Result refine_bounded(const std::vector<Eigen::Vector3d>& scan, const Se2& initial,
                        const RefinementProfile& profile) const;

 private:
  double field_at(double x, double y) const;
  Options options_;
  std::vector<Eigen::Vector3d> cells_;
  std::vector<Eigen::Vector2d> normals_;
  std::vector<bool> normal_valid_;
  double field_min_x_ = 0.0, field_min_y_ = 0.0;
  int field_width_ = 0, field_height_ = 0;
  std::vector<double> field_;
  std::shared_ptr<small_gicp::PointCloud> map_cloud_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> map_tree_;
};

}  // namespace soarm_localization
