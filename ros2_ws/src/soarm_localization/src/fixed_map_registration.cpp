#include "soarm_localization/fixed_map_registration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>

namespace soarm_localization {
namespace {
constexpr std::size_t kMaxCellBudget = 1200;
constexpr std::size_t kMaxFieldCells = 50000000U;
constexpr std::size_t kMaxLatticeEvaluations = 1000000U;
constexpr int kMaxIterations = 10000;

bool checked_floor_to_int(double value, int* output) {
  if (!std::isfinite(value)) return false;
  const double floored = std::floor(value);
  if (floored < static_cast<double>(std::numeric_limits<int>::min()) ||
      floored > static_cast<double>(std::numeric_limits<int>::max())) return false;
  *output = static_cast<int>(floored);
  return true;
}
bool checked_round_to_int(double value, int* output) {
  if (!std::isfinite(value)) return false;
  const double rounded = std::round(value);
  if (rounded < static_cast<double>(std::numeric_limits<int>::min()) ||
      rounded > static_cast<double>(std::numeric_limits<int>::max())) return false;
  *output = static_cast<int>(rounded);
  return true;
}
bool checked_ceil_to_int(double value, int* output) {
  if (!std::isfinite(value)) return false;
  const double ceiled = std::ceil(value);
  if (ceiled < static_cast<double>(std::numeric_limits<int>::min()) ||
      ceiled > static_cast<double>(std::numeric_limits<int>::max())) return false;
  *output = static_cast<int>(ceiled);
  return true;
}
bool checked_field_dimension(double span, double resolution, int* output) {
  const double quotient = span / resolution;
  if (!std::isfinite(quotient) || quotient < 0.0) return false;
  const double cells = std::ceil(quotient);
  if (cells > static_cast<double>(std::numeric_limits<int>::max() - 1)) return false;
  *output = static_cast<int>(cells) + 1;
  return *output > 0;
}
bool checked_multiply(std::size_t left, std::size_t right, std::size_t* output) {
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) return false;
  *output = left * right;
  return true;
}
bool valid_options(const Options& options) {
  return std::isfinite(options.voxel_resolution) && options.voxel_resolution > 0.0 &&
         std::isfinite(options.distance_field_resolution) && options.distance_field_resolution > 0.0 &&
         std::isfinite(options.capped_squared_loss) && options.capped_squared_loss > 0.0 &&
         std::isfinite(options.max_correspondence_distance) && options.max_correspondence_distance > 0.0 &&
         options.min_cells > 0 && options.max_cells >= options.min_cells &&
         static_cast<std::size_t>(options.max_cells) <= kMaxCellBudget &&
         options.max_iterations > 0 && options.max_iterations <= kMaxIterations && options.threads > 0;
}
bool finite_points(const std::vector<Eigen::Vector3d>& points) {
  return !points.empty() && std::all_of(points.begin(), points.end(), [](const auto& point) { return point.allFinite(); });
}
bool valid_profile(const SearchProfile& p) {
  return std::isfinite(p.xy_extent) && std::isfinite(p.yaw_extent) && std::isfinite(p.xy_step) &&
         std::isfinite(p.yaw_step) && std::isfinite(p.nms_radius) && p.xy_extent >= 0.0 &&
         p.yaw_extent >= 0.0 && p.xy_step > 0.0 && p.yaw_step > 0.0 && p.nms_radius >= 0.0 && p.peak_count > 0;
}
bool valid_refinement_profile(const RefinementProfile& p) {
  return std::isfinite(p.max_relative_x) && std::isfinite(p.max_relative_y) &&
         std::isfinite(p.max_relative_yaw) && p.max_relative_x >= 0.0 &&
         p.max_relative_y >= 0.0 && p.max_relative_yaw >= 0.0;
}
bool within_refinement_profile(const Se2& relative, const RefinementProfile& p) {
  return std::isfinite(relative.x) && std::isfinite(relative.y) && std::isfinite(relative.yaw) &&
         std::abs(relative.x) <= p.max_relative_x && std::abs(relative.y) <= p.max_relative_y &&
         std::abs(relative.yaw) <= p.max_relative_yaw;
}
Se2 compose_relative(const Se2& initial, const Se2& relative) {
  const double c = std::cos(relative.yaw), s = std::sin(relative.yaw);
  return {relative.x + c * initial.x - s * initial.y,
          relative.y + s * initial.x + c * initial.y, relative.yaw + initial.yaw};
}
bool checked_lattice(const SearchProfile& profile, int* xy_steps, int* yaw_steps, std::size_t* evaluations) {
  const double xy_quotient = profile.xy_extent / profile.xy_step;
  const double yaw_quotient = profile.yaw_extent / profile.yaw_step;
  if (!std::isfinite(xy_quotient) || !std::isfinite(yaw_quotient)) return false;
  if (!checked_floor_to_int(std::floor(xy_quotient + 1e-9), xy_steps) ||
      !checked_floor_to_int(std::floor(yaw_quotient + 1e-9), yaw_steps) || *xy_steps < 0 || *yaw_steps < 0 ||
      *xy_steps > (std::numeric_limits<int>::max() - 1) / 2 || *yaw_steps > (std::numeric_limits<int>::max() - 1) / 2) return false;
  const std::size_t xy_count = 2U * static_cast<std::size_t>(*xy_steps) + 1U;
  const std::size_t yaw_count = 2U * static_cast<std::size_t>(*yaw_steps) + 1U;
  std::size_t plane_count = 0;
  return checked_multiply(xy_count, xy_count, &plane_count) && checked_multiply(plane_count, yaw_count, evaluations) &&
         *evaluations <= kMaxLatticeEvaluations;
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

FixedMapRegistration::FixedMapRegistration(Options options) : options_(options) {}

bool FixedMapRegistration::set_map(const std::vector<Eigen::Vector3d>& points) {
  cells_.clear(); field_.clear(); normals_.clear(); normal_valid_.clear(); map_cloud_.reset(); map_tree_.reset();
  if (!finite_points(points) || !valid_options(options_)) return false;
  std::map<std::tuple<int, int, int>, std::pair<Eigen::Vector3d, std::size_t>> bins;
  for (const auto& point : points) {
    int x = 0, y = 0, z = 0;
    if (!checked_floor_to_int(point.x() / options_.voxel_resolution, &x) ||
        !checked_floor_to_int(point.y() / options_.voxel_resolution, &y) ||
        !checked_floor_to_int(point.z() / options_.voxel_resolution, &z)) return false;
    const auto key = std::make_tuple(x, y, z);
    auto found = bins.find(key);
    if (found == bins.end()) {
      if (bins.size() >= static_cast<std::size_t>(options_.max_cells)) return false;
      found = bins.emplace(key, std::make_pair(Eigen::Vector3d::Zero(), std::size_t{0})).first;
    }
    if (found->second.second == std::numeric_limits<std::size_t>::max()) return false;
    const Eigen::Vector3d accumulated = found->second.first + point;
    if (!accumulated.allFinite()) return false;
    found->second.first = accumulated;
    ++found->second.second;
  }
  if (bins.size() < static_cast<std::size_t>(options_.min_cells)) return false;
  cells_.reserve(bins.size());
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
  const double margin = 0.75;
  const double span_x = max_x - min_x + 2.0 * margin, span_y = max_y - min_y + 2.0 * margin;
  field_min_x_ = min_x - margin; field_min_y_ = min_y - margin;
  std::size_t field_cells = 0;
  if (!std::isfinite(span_x) || !std::isfinite(span_y) || !std::isfinite(field_min_x_) || !std::isfinite(field_min_y_) ||
      !checked_field_dimension(span_x, options_.distance_field_resolution, &field_width_) ||
      !checked_field_dimension(span_y, options_.distance_field_resolution, &field_height_) ||
      !checked_multiply(static_cast<std::size_t>(field_width_), static_cast<std::size_t>(field_height_), &field_cells) ||
      field_cells == 0 || field_cells > kMaxFieldCells) return false;
  field_.assign(field_cells, options_.capped_squared_loss);
  // A point farther than sqrt(cap) cannot improve a field cell. Scatter each
  // exact point distance only over that bounded window instead of scanning all
  // map points for every grid cell.
  const double influence_radius = std::sqrt(options_.capped_squared_loss);
  for (const auto& cell : cells_) {
    int min_ix = 0, max_ix = 0, min_iy = 0, max_iy = 0;
    if (!checked_ceil_to_int((cell.x() - influence_radius - field_min_x_) / options_.distance_field_resolution, &min_ix) ||
        !checked_floor_to_int((cell.x() + influence_radius - field_min_x_) / options_.distance_field_resolution, &max_ix) ||
        !checked_ceil_to_int((cell.y() - influence_radius - field_min_y_) / options_.distance_field_resolution, &min_iy) ||
        !checked_floor_to_int((cell.y() + influence_radius - field_min_y_) / options_.distance_field_resolution, &max_iy)) return false;
    min_ix = std::clamp(min_ix, 0, field_width_ - 1);
    max_ix = std::clamp(max_ix, 0, field_width_ - 1);
    min_iy = std::clamp(min_iy, 0, field_height_ - 1);
    max_iy = std::clamp(max_iy, 0, field_height_ - 1);
    for (int iy = min_iy; iy <= max_iy; ++iy) for (int ix = min_ix; ix <= max_ix; ++ix) {
      const Eigen::Vector2d grid(field_min_x_ + ix * options_.distance_field_resolution,
                                 field_min_y_ + iy * options_.distance_field_resolution);
      const std::size_t index = static_cast<std::size_t>(iy) * static_cast<std::size_t>(field_width_) +
                                static_cast<std::size_t>(ix);
      field_[index] = std::min(field_[index], (grid - cell.head<2>()).squaredNorm());
    }
  }
  auto processed = small_gicp::preprocess_points(cells_, options_.voxel_resolution, 10, options_.threads);
  map_cloud_ = processed.first; map_tree_ = processed.second;
  return true;
}

double FixedMapRegistration::field_at(double x, double y) const {
  int ix = 0, iy = 0;
  if (!checked_round_to_int((x - field_min_x_) / options_.distance_field_resolution, &ix) ||
      !checked_round_to_int((y - field_min_y_) / options_.distance_field_resolution, &iy) ||
      ix < 0 || iy < 0 || ix >= field_width_ || iy >= field_height_) return options_.capped_squared_loss;
  return field_[static_cast<std::size_t>(iy) * static_cast<std::size_t>(field_width_) + static_cast<std::size_t>(ix)];
}

PreRefinementResult FixedMapRegistration::search(const std::vector<Eigen::Vector3d>& source, const SearchProfile& profile) const {
  PreRefinementResult result; const auto start = std::chrono::steady_clock::now();
  int xy_steps = 0, yaw_steps = 0; std::size_t evaluations = 0;
  if (!map_tree_ || !valid_options(options_) || !finite_points(source) || source.size() < static_cast<std::size_t>(options_.min_cells) ||
      !valid_profile(profile) || !checked_lattice(profile, &xy_steps, &yaw_steps, &evaluations)) return result;
  result.accepted_input = true; result.metrics.source_cells = source.size();
  std::vector<Peak> raw;
  raw.reserve(evaluations);
  for (int ia = -yaw_steps; ia <= yaw_steps; ++ia) for (int iy = -xy_steps; iy <= xy_steps; ++iy) for (int ix = -xy_steps; ix <= xy_steps; ++ix) {
    Peak peak{{ix * profile.xy_step, iy * profile.xy_step, ia * profile.yaw_step}, 0.0};
    const double c = std::cos(peak.pose.yaw), s = std::sin(peak.pose.yaw);
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

Result FixedMapRegistration::refine_bounded(const std::vector<Eigen::Vector3d>& scan, const Se2& initial,
                                             const RefinementProfile& profile) const {
  Result result; const auto start = std::chrono::steady_clock::now();
  if (!map_tree_ || !valid_options(options_) || !valid_refinement_profile(profile) || !finite_points(scan) ||
      scan.size() < static_cast<std::size_t>(options_.min_cells) || !std::isfinite(initial.x) ||
      !std::isfinite(initial.y) || !std::isfinite(initial.yaw)) return result;
  result.accepted_input = true; result.bounded_refinement_used = true; result.transform = initial;
  struct Evaluation { double error = 0.0; int inliers = 0; Eigen::Matrix3d h = Eigen::Matrix3d::Zero(); Eigen::Vector3d b = Eigen::Vector3d::Zero(); };
  const double max2 = options_.max_correspondence_distance * options_.max_correspondence_distance;
  auto evaluate = [&](const Se2& relative, bool linearize, Evaluation* evaluation) -> bool {
    // This guard is deliberately before the first transformed point/correspondence lookup.
    if (!within_refinement_profile(relative, profile)) { result.trust_region_limited = true; return false; }
    ++result.bounded_evaluations;
    const double c0 = std::cos(initial.yaw), s0 = std::sin(initial.yaw);
    const double cd = std::cos(relative.yaw), sd = std::sin(relative.yaw);
    for (const auto& point : scan) {
      const Eigen::Vector2d base(c0 * point.x() - s0 * point.y() + initial.x,
                                 s0 * point.x() + c0 * point.y() + initial.y);
      const Eigen::Vector2d transformed(cd * base.x() - sd * base.y() + relative.x,
                                        sd * base.x() + cd * base.y() + relative.y);
      double best = max2; std::size_t target = cells_.size();
      for (std::size_t j = 0; j < cells_.size(); ++j) {
        const double distance2 = (transformed - cells_[j].head<2>()).squaredNorm();
        if (distance2 < best) { best = distance2; target = j; }
      }
      if (target == cells_.size()) continue;
      const Eigen::Vector2d residual = transformed - cells_[target].head<2>();
      evaluation->error += residual.squaredNorm(); ++evaluation->inliers;
      if (linearize) {
        Eigen::Matrix<double, 2, 3> jacobian;
        jacobian << 1.0, 0.0, -sd * base.x() - cd * base.y(),
                    0.0, 1.0,  cd * base.x() - sd * base.y();
        evaluation->h.noalias() += jacobian.transpose() * jacobian;
        evaluation->b.noalias() += jacobian.transpose() * residual;
      }
    }
    evaluation->error = evaluation->inliers ? evaluation->error / evaluation->inliers : std::numeric_limits<double>::infinity();
    return std::isfinite(evaluation->error);
  };
  Se2 relative; Evaluation current;
  if (!evaluate(relative, true, &current)) { result.all_evaluations_within_bounds = false; return result; }
  double damping = 1e-3;
  for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
    if (current.inliers < 3) break;
    Eigen::Matrix3d system = current.h;
    for (int axis = 0; axis < 3; ++axis) system(axis, axis) += damping * std::max(1.0, current.h(axis, axis));
    const Eigen::Vector3d increment = -system.ldlt().solve(current.b);
    if (!increment.allFinite()) break;
    Se2 candidate{relative.x + increment.x(), relative.y + increment.y(), relative.yaw + increment.z()};
    Evaluation trial;
    if (!evaluate(candidate, false, &trial)) { damping *= 10.0; continue; }
    if (trial.error < current.error) {
      relative = candidate;
      if (increment.norm() < 1e-6) { result.converged = true; current = trial; break; }
      if (!evaluate(relative, true, &current)) { result.all_evaluations_within_bounds = false; break; }
      damping = std::max(1e-9, damping * 0.5);
    } else {
      damping *= 10.0;
    }
  }
  result.transform = compose_relative(initial, relative);
  result.mean_squared_error = current.error;
  result.inlier_fraction = current.inliers / static_cast<double>(scan.size());
  // The same bounded final correspondence set supplies raw planar observability.
  result.observability.information = current.h;
  result.observability.valid_support = current.inliers;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(current.h); result.observability.eigenvalues = solver.eigenvalues();
  const double trace = result.observability.eigenvalues.sum(), top = result.observability.eigenvalues.maxCoeff();
  if (top > std::numeric_limits<double>::epsilon()) { result.observability.min_trace_ratio = result.observability.eigenvalues.minCoeff() / std::max(trace, std::numeric_limits<double>::epsilon()); result.observability.condition_number = top / std::max(result.observability.eigenvalues.minCoeff(), std::numeric_limits<double>::epsilon()); }
  result.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(); return result;
}

Result FixedMapRegistration::refine(const std::vector<Eigen::Vector3d>& scan, const Se2& initial) const {
  Result result; const auto start = std::chrono::steady_clock::now();
  if (!map_tree_ || !valid_options(options_) || !finite_points(scan) || scan.size() < static_cast<std::size_t>(options_.min_cells) || !std::isfinite(initial.x) || !std::isfinite(initial.y) || !std::isfinite(initial.yaw)) return result;
  result.accepted_input = true;
  small_gicp::RegistrationSetting settings; settings.type = small_gicp::RegistrationSetting::GICP; settings.num_threads = options_.threads; settings.max_iterations = options_.max_iterations; settings.max_correspondence_distance = options_.max_correspondence_distance; settings.downsampling_resolution = options_.voxel_resolution;
  auto source = small_gicp::preprocess_points(scan, options_.voxel_resolution, 10, options_.threads);
  auto alignment = small_gicp::align(*map_cloud_, *source.first, *map_tree_, as_isometry(initial), settings);
  result.converged = alignment.converged; result.transform = as_se2(alignment.T_target_source); result.mean_squared_error = alignment.error;
  result.inlier_fraction = alignment.num_inliers / static_cast<double>(std::max<std::size_t>(1, source.first->size()));
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
