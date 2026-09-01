#include "soarm_localization/fixed_map_registration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <tuple>

namespace soarm_localization {
namespace {
bool finite_points(const std::vector<Eigen::Vector3d>& points) {
  return !points.empty() && std::all_of(points.begin(), points.end(), [](const auto& point) { return point.allFinite(); });
}
bool valid_profile(const SearchProfile& p) {
  return std::isfinite(p.xy_extent) && std::isfinite(p.yaw_extent) && std::isfinite(p.xy_step) &&
         std::isfinite(p.yaw_step) && std::isfinite(p.nms_radius) && p.xy_extent >= 0.0 &&
         p.yaw_extent >= 0.0 && p.xy_step > 0.0 && p.yaw_step > 0.0 && p.nms_radius >= 0.0 && p.peak_count > 0;
}
Eigen::Isometry3d as_isometry(const Se2& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.x, pose.y, 0.0);
  result.linear() = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}
Se2 as_se2(const Eigen::Isometry3d& pose) {
  return {pose.translation().x(), pose.translation().y(), std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0))};
}
}  // namespace

int Observability::rank_for_relative_tolerance(double tolerance) const {
  if (!std::isfinite(tolerance) || tolerance < 0.0 || eigenvalues.maxCoeff() <= std::numeric_limits<double>::epsilon()) return 0;
  int rank = 0;
  for (int index = 0; index < 3; ++index) if (eigenvalues[index] > eigenvalues.maxCoeff() * tolerance) ++rank;
  return rank;
}

FixedMapRegistration::FixedMapRegistration(Options options) : options_(options) { options_.threads = std::max(1, options.threads); }

bool FixedMapRegistration::set_map(const std::vector<Eigen::Vector3d>& points) {
  cells_.clear(); field_.clear(); map_cloud_.reset(); map_tree_.reset();
  if (!finite_points(points) || !std::isfinite(options_.voxel_resolution) || options_.voxel_resolution <= 0.0 ||
      !std::isfinite(options_.distance_field_resolution) || options_.distance_field_resolution <= 0.0 ||
      !std::isfinite(options_.capped_squared_loss) || options_.capped_squared_loss <= 0.0) return false;
  std::map<std::tuple<int, int, int>, std::pair<Eigen::Vector3d, int>> bins;
  for (const auto& point : points) {
    auto key = std::make_tuple(static_cast<int>(std::floor(point.x() / options_.voxel_resolution)),
                               static_cast<int>(std::floor(point.y() / options_.voxel_resolution)),
                               static_cast<int>(std::floor(point.z() / options_.voxel_resolution)));
    auto& bin = bins[key];
    if (bin.second == 0) bin.first.setZero();
    bin.first += point; ++bin.second;
  }
  if (static_cast<int>(bins.size()) < options_.min_cells || static_cast<int>(bins.size()) > options_.max_cells) return false;
  for (const auto& bin : bins) cells_.push_back(bin.second.first / bin.second.second);
  normals_.assign(cells_.size(), Eigen::Vector2d::Zero()); normal_valid_.assign(cells_.size(), false);
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    std::vector<std::pair<double, std::size_t>> neighbors;
    for (std::size_t j = 0; j < cells_.size(); ++j) if (i != j) {
      const double distance2 = (cells_[i].head<2>() - cells_[j].head<2>()).squaredNorm();
      if (distance2 <= 16.0 * options_.voxel_resolution * options_.voxel_resolution) neighbors.emplace_back(distance2, j);
    }
    if (neighbors.size() < 4) continue;
    std::partial_sort(neighbors.begin(), neighbors.begin() + 4, neighbors.end());
    Eigen::Vector2d mean = Eigen::Vector2d::Zero(); for (int k = 0; k < 4; ++k) mean += cells_[neighbors[k].second].head<2>(); mean /= 4.0;
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero(); for (int k = 0; k < 4; ++k) { const auto delta = cells_[neighbors[k].second].head<2>() - mean; covariance += delta * delta.transpose(); }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance); const auto values = solver.eigenvalues();
    if (values[1] > std::numeric_limits<double>::epsilon() && values[0] <= .10 * values[1]) { normals_[i] = solver.eigenvectors().col(0); normal_valid_[i] = true; }
  }
  double min_x = cells_.front().x(), max_x = min_x, min_y = cells_.front().y(), max_y = min_y;
  for (const auto& cell : cells_) { min_x = std::min(min_x, cell.x()); max_x = std::max(max_x, cell.x()); min_y = std::min(min_y, cell.y()); max_y = std::max(max_y, cell.y()); }
  // The field deliberately extends beyond the map; outside samples receive exactly the fixed cap.
  const double margin = 0.75;
  field_min_x_ = min_x - margin; field_min_y_ = min_y - margin;
  field_width_ = static_cast<int>(std::ceil((max_x - min_x + 2.0 * margin) / options_.distance_field_resolution)) + 1;
  field_height_ = static_cast<int>(std::ceil((max_y - min_y + 2.0 * margin) / options_.distance_field_resolution)) + 1;
  if (field_width_ <= 0 || field_height_ <= 0 || static_cast<std::size_t>(field_width_) * field_height_ > 50000000U) return false;
  field_.assign(static_cast<std::size_t>(field_width_) * field_height_, options_.capped_squared_loss);
  for (int iy = 0; iy < field_height_; ++iy) for (int ix = 0; ix < field_width_; ++ix) {
    const Eigen::Vector2d grid(field_min_x_ + ix * options_.distance_field_resolution, field_min_y_ + iy * options_.distance_field_resolution);
    double best = options_.capped_squared_loss;
    for (const auto& cell : cells_) best = std::min(best, (grid - cell.head<2>()).squaredNorm());
    field_[static_cast<std::size_t>(iy) * field_width_ + ix] = best;
  }
  auto processed = small_gicp::preprocess_points(cells_, options_.voxel_resolution, 10, options_.threads);
  map_cloud_ = processed.first; map_tree_ = processed.second;
  return true;
}

double FixedMapRegistration::field_at(double x, double y) const {
  const int ix = static_cast<int>(std::llround((x - field_min_x_) / options_.distance_field_resolution));
  const int iy = static_cast<int>(std::llround((y - field_min_y_) / options_.distance_field_resolution));
  if (ix < 0 || iy < 0 || ix >= field_width_ || iy >= field_height_) return options_.capped_squared_loss;
  return field_[static_cast<std::size_t>(iy) * field_width_ + ix];
}

PreRefinementResult FixedMapRegistration::search(const std::vector<Eigen::Vector3d>& source, const SearchProfile& profile) const {
  PreRefinementResult result; const auto start = std::chrono::steady_clock::now();
  if (!map_tree_ || !finite_points(source) || static_cast<int>(source.size()) < options_.min_cells || !valid_profile(profile)) return result;
  result.accepted_input = true; result.metrics.source_cells = source.size();
  const int xy_steps = static_cast<int>(std::floor(profile.xy_extent / profile.xy_step + 1e-9));
  const int yaw_steps = static_cast<int>(std::floor(profile.yaw_extent / profile.yaw_step + 1e-9));
  std::vector<Peak> raw;
  raw.reserve(static_cast<std::size_t>(2 * xy_steps + 1) * (2 * xy_steps + 1) * (2 * yaw_steps + 1));
  for (int ia = -yaw_steps; ia <= yaw_steps; ++ia) for (int iy = -xy_steps; iy <= xy_steps; ++iy) for (int ix = -xy_steps; ix <= xy_steps; ++ix) {
    Peak peak{{ix * profile.xy_step, iy * profile.xy_step, ia * profile.yaw_step}, 0.0};
    const double c = std::cos(peak.pose.yaw), s = std::sin(peak.pose.yaw);
    // Serial accumulation and fixed source denominator are intentional determinism invariants.
    for (const auto& point : source) peak.score += std::min(options_.capped_squared_loss, field_at(peak.pose.x + c * point.x() - s * point.y(), peak.pose.y + s * point.x() + c * point.y()));
    peak.score /= static_cast<double>(source.size()); raw.push_back(peak);
  }
  result.metrics.lattice_evaluations = raw.size();
  std::sort(raw.begin(), raw.end(), [](const Peak& left, const Peak& right) {
    if (left.score != right.score) return left.score < right.score;
    if (left.pose.x != right.pose.x) return left.pose.x < right.pose.x;
    if (left.pose.y != right.pose.y) return left.pose.y < right.pose.y;
    return left.pose.yaw < right.pose.yaw;
  });
  const double radius2 = profile.nms_radius * profile.nms_radius;
  for (const auto& peak : raw) {
    bool distinct = true;
    for (const auto& kept : result.peaks) if (std::pow(peak.pose.x - kept.pose.x, 2) + std::pow(peak.pose.y - kept.pose.y, 2) < radius2) { distinct = false; break; }
    if (distinct) result.peaks.push_back(peak);
    if (result.peaks.size() == profile.peak_count) break;
  }
  if (result.peaks.size() > 1) result.score_gap = result.peaks[1].score - result.peaks[0].score;
  result.metrics.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return result;
}

std::vector<Result> FixedMapRegistration::refine_selected(const std::vector<Eigen::Vector3d>& source, const std::vector<Peak>& selected) const {
  std::vector<Result> results; results.reserve(selected.size());
  for (const auto& peak : selected) results.push_back(refine(source, peak.pose));
  return results;
}

Result FixedMapRegistration::refine(const std::vector<Eigen::Vector3d>& scan, const Se2& initial) const {
  Result result; const auto start = std::chrono::steady_clock::now();
  if (!map_tree_ || !finite_points(scan) || static_cast<int>(scan.size()) < options_.min_cells || !std::isfinite(initial.x) || !std::isfinite(initial.y) || !std::isfinite(initial.yaw)) return result;
  result.accepted_input = true;
  small_gicp::RegistrationSetting settings; settings.type = small_gicp::RegistrationSetting::GICP; settings.num_threads = options_.threads; settings.max_iterations = options_.max_iterations; settings.max_correspondence_distance = options_.max_correspondence_distance; settings.downsampling_resolution = options_.voxel_resolution;
  auto source = small_gicp::preprocess_points(scan, options_.voxel_resolution, 10, options_.threads);
  auto alignment = small_gicp::align(*map_cloud_, *source.first, *map_tree_, as_isometry(initial), settings);
  result.converged = alignment.converged; result.transform = as_se2(alignment.T_target_source); result.mean_squared_error = alignment.error;
  result.inlier_fraction = alignment.num_inliers / static_cast<double>(std::max<std::size_t>(1, source.first->size()));
  // Independent map-normal point-to-plane SE(2) information, normalized in yaw by source radius.
  double radius2 = 0.0; for (const auto& point : scan) radius2 += point.head<2>().squaredNorm();
  const double radius = std::max(options_.voxel_resolution, std::sqrt(radius2 / scan.size()));
  const double c = std::cos(result.transform.yaw), s = std::sin(result.transform.yaw), max2 = options_.max_correspondence_distance * options_.max_correspondence_distance;
  for (const auto& point : scan) {
    Eigen::Vector2d transformed(c * point.x() - s * point.y() + result.transform.x, s * point.x() + c * point.y() + result.transform.y);
    double best = max2; std::size_t target = cells_.size();
    for (std::size_t j = 0; j < cells_.size(); ++j) { const double distance2 = (transformed - cells_[j].head<2>()).squaredNorm(); if (distance2 < best) { best = distance2; target = j; } }
    if (target < cells_.size() && normal_valid_[target]) { const auto& normal = normals_[target]; Eigen::Vector2d dtheta(-s * point.x() - c * point.y(), c * point.x() - s * point.y()); Eigen::Vector3d jacobian(normal.x(), normal.y(), normal.dot(dtheta) / radius); result.observability.information += jacobian * jacobian.transpose(); ++result.observability.valid_support; }
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result.observability.information); result.observability.eigenvalues = solver.eigenvalues();
  const double trace = result.observability.eigenvalues.sum(), top = result.observability.eigenvalues.maxCoeff();
  if (top > std::numeric_limits<double>::epsilon()) { result.observability.min_trace_ratio = result.observability.eigenvalues.minCoeff() / std::max(trace, std::numeric_limits<double>::epsilon()); result.observability.condition_number = top / std::max(result.observability.eigenvalues.minCoeff(), std::numeric_limits<double>::epsilon()); }
  result.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(); return result;
}

std::size_t FixedMapRegistration::cell_count() const { return cells_.size(); }
}  // namespace soarm_localization
