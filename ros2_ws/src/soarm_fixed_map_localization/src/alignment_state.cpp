#include "soarm_fixed_map_localization/alignment_state.hpp"

#include <cmath>
#include <limits>

namespace soarm_fixed_map_localization {
namespace {

bool valid_transform(const Eigen::Isometry3d& transform) {
    if (!transform.matrix().allFinite()) {
        return false;
    }
    const Eigen::Matrix3d orthogonality =
        transform.linear().transpose() * transform.linear();
    return (orthogonality - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
           std::abs(transform.linear().determinant() - 1.0) <= 1e-6;
}

double yaw_distance(double first, double second) {
    return std::abs(std::remainder(first - second, 2.0 * std::acos(-1.0)));
}

}  // namespace

Eigen::Isometry3d compose_candidate(const Eigen::Isometry3d& alignment,
                                    const soarm_localization::Se2& delta,
                                    const Eigen::Vector2d& anchor) {
    Eigen::Isometry3d to_anchor = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d from_anchor = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    to_anchor.translation().head<2>() = anchor;
    from_anchor.translation().head<2>() = -anchor;
    correction.translation() = Eigen::Vector3d(delta.x, delta.y, 0.0);
    correction.linear() =
        Eigen::AngleAxisd(delta.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return to_anchor * correction * from_anchor * alignment;
}

CandidateConsensus::CandidateConsensus(CandidateConsensusOptions options)
    : options_(options) {
    valid_ = options_.confirmation_scans > 0 &&
             std::isfinite(options_.max_translation) &&
             options_.max_translation > 0.0 && std::isfinite(options_.max_yaw) &&
             options_.max_yaw > 0.0;
}

std::optional<soarm_localization::Se2> CandidateConsensus::observe(
    const soarm_localization::Se2& candidate) {
    if (!valid_ || !std::isfinite(candidate.x) || !std::isfinite(candidate.y) ||
        !std::isfinite(candidate.yaw)) {
        reset();
        return std::nullopt;
    }
    if (!pending_.empty()) {
        const auto& reference = pending_.front();
        if (std::hypot(candidate.x - reference.x,
                       candidate.y - reference.y) >= options_.max_translation ||
            yaw_distance(candidate.yaw, reference.yaw) >= options_.max_yaw) {
            reset();
            return std::nullopt;
        }
    }
    pending_.push_back(candidate);
    if (pending_.size() < options_.confirmation_scans) {
        return std::nullopt;
    }
    soarm_localization::Se2 result;
    double sine_sum = 0.0;
    double cosine_sum = 0.0;
    for (const auto& value : pending_) {
        result.x += value.x;
        result.y += value.y;
        sine_sum += std::sin(value.yaw);
        cosine_sum += std::cos(value.yaw);
    }
    const double count = static_cast<double>(pending_.size());
    result.x /= count;
    result.y /= count;
    result.yaw = std::atan2(sine_sum, cosine_sum);
    reset();
    return result;
}

void CandidateConsensus::reset() { pending_.clear(); }

std::size_t CandidateConsensus::pending_count() const { return pending_.size(); }

RecoveryState::RecoveryState(RecoveryStateOptions options) : options_(options) {
    valid_ = options_.max_tracking_rejections > 0 &&
             options_.max_recovery_attempts > 0 &&
             options_.stationary_settle_ns > 0 &&
             std::isfinite(options_.stationary_linear_speed) &&
             options_.stationary_linear_speed >= 0.0 &&
             std::isfinite(options_.stationary_angular_speed) &&
             options_.stationary_angular_speed >= 0.0;
}

bool RecoveryState::observe_motion(std::int64_t stamp_ns, double linear_speed,
                                   double angular_speed) {
    if (!valid_ || stamp_ns <= latest_stamp_ns_ || !std::isfinite(linear_speed) ||
        linear_speed < 0.0 || !std::isfinite(angular_speed) ||
        angular_speed < 0.0) {
        return false;
    }
    latest_stamp_ns_ = stamp_ns;
    if (last_motion_stamp_ns_ == 0 ||
        linear_speed >= options_.stationary_linear_speed ||
        angular_speed >= options_.stationary_angular_speed) {
        last_motion_stamp_ns_ = stamp_ns;
    }
    return true;
}

void RecoveryState::tracking_rejected() {
    if (!valid_ || lost_ ||
        consecutive_rejections_ == std::numeric_limits<std::size_t>::max()) {
        return;
    }
    ++consecutive_rejections_;
}

void RecoveryState::recovery_attempt_failed() {
    if (!valid_ || lost_) {
        return;
    }
    if (recovery_attempts_ == std::numeric_limits<std::size_t>::max() ||
        ++recovery_attempts_ >= options_.max_recovery_attempts) {
        lost_ = true;
    }
}

void RecoveryState::correction_committed() {
    consecutive_rejections_ = 0;
    recovery_attempts_ = 0;
    lost_ = false;
}

MatchingMode RecoveryState::mode() const {
    if (!valid_ || lost_) {
        return MatchingMode::kLost;
    }
    if (consecutive_rejections_ < options_.max_tracking_rejections) {
        return MatchingMode::kTracking;
    }
    if (latest_stamp_ns_ <= 0 || last_motion_stamp_ns_ <= 0 ||
        latest_stamp_ns_ - last_motion_stamp_ns_ < options_.stationary_settle_ns) {
        return MatchingMode::kWaitingForStationary;
    }
    return MatchingMode::kRecovery;
}

std::size_t RecoveryState::consecutive_rejections() const {
    return consecutive_rejections_;
}

AlignmentState::AlignmentState(AlignmentStateOptions options) : options_(options) {
    if (options_.odom_capacity < 2) {
        options_.odom_capacity = 2;
    }
    if (options_.max_interpolation_span_ns <= 0) {
        options_.max_interpolation_span_ns = 1;
    }
}

bool AlignmentState::add_odometry(
    std::int64_t stamp_ns, const Eigen::Isometry3d& camera_init_to_body) {
    if (stamp_ns <= 0 || !valid_transform(camera_init_to_body) ||
        (!odometry_.empty() && stamp_ns <= odometry_.back().stamp_ns)) {
        return false;
    }
    odometry_.push_back({stamp_ns, camera_init_to_body});
    while (odometry_.size() > options_.odom_capacity) {
        odometry_.pop_front();
    }
    return true;
}

std::optional<Eigen::Isometry3d> AlignmentState::odometry_at(
    std::int64_t stamp_ns) const {
    if (stamp_ns <= 0 || odometry_.empty() || stamp_ns < odometry_.front().stamp_ns ||
        stamp_ns > odometry_.back().stamp_ns) {
        return std::nullopt;
    }
    for (std::size_t index = 0; index < odometry_.size(); ++index) {
        const OdomSample& upper = odometry_[index];
        if (upper.stamp_ns == stamp_ns) {
            return upper.pose;
        }
        if (upper.stamp_ns > stamp_ns) {
            if (index == 0) {
                return std::nullopt;
            }
            const OdomSample& lower = odometry_[index - 1];
            const std::int64_t span = upper.stamp_ns - lower.stamp_ns;
            if (span <= 0 || span > options_.max_interpolation_span_ns) {
                return std::nullopt;
            }
            const double fraction = static_cast<double>(stamp_ns - lower.stamp_ns) /
                                    static_cast<double>(span);
            Eigen::Isometry3d interpolated = Eigen::Isometry3d::Identity();
            interpolated.translation() =
                lower.pose.translation() +
                fraction * (upper.pose.translation() - lower.pose.translation());
            Eigen::Quaterniond lower_rotation(lower.pose.linear());
            Eigen::Quaterniond upper_rotation(upper.pose.linear());
            interpolated.linear() =
                lower_rotation.slerp(fraction, upper_rotation).normalized().toRotationMatrix();
            return interpolated;
        }
    }
    return std::nullopt;
}

bool AlignmentState::seed_from_initial_pose(
    std::int64_t stamp_ns, const Eigen::Isometry3d& map_to_base,
    const Eigen::Isometry3d& base_to_body) {
    if (!valid_transform(map_to_base) || !valid_transform(base_to_body) ||
        alignment_generation_ == std::numeric_limits<std::uint64_t>::max()) {
        return false;
    }
    const std::optional<Eigen::Isometry3d> camera_init_to_body =
        odometry_at(stamp_ns);
    if (!camera_init_to_body.has_value()) {
        return false;
    }
    const Eigen::Isometry3d candidate =
        map_to_base * base_to_body * camera_init_to_body->inverse();
    if (!valid_transform(candidate)) {
        return false;
    }
    alignment_ = candidate;
    ++alignment_generation_;
    initialized_ = true;
    return true;
}

std::optional<CloudContext> AlignmentState::begin_cloud(std::int64_t stamp_ns) {
    if (!initialized_ || stamp_ns <= 0 || stamp_ns <= last_cloud_stamp_ns_) {
        return std::nullopt;
    }
    last_cloud_stamp_ns_ = stamp_ns;
    return CloudContext{stamp_ns, alignment_generation_, alignment_};
}

bool AlignmentState::apply_correction(const CloudContext& context,
                                      const soarm_localization::Se2& delta,
                                      const Eigen::Vector2d& anchor) {
    if (!initialized_ || context.stamp_ns != last_cloud_stamp_ns_ ||
        context.alignment_generation != alignment_generation_ ||
        alignment_generation_ == std::numeric_limits<std::uint64_t>::max() ||
        !context.alignment.matrix().allFinite() || !anchor.allFinite() ||
        !std::isfinite(delta.x) || !std::isfinite(delta.y) ||
        !std::isfinite(delta.yaw)) {
        return false;
    }
    const Eigen::Isometry3d candidate =
        compose_candidate(context.alignment, delta, anchor);
    if (!valid_transform(candidate)) {
        return false;
    }
    alignment_ = candidate;
    ++alignment_generation_;
    return true;
}

bool AlignmentState::apply_alignment(const CloudContext& context,
                                     const Eigen::Isometry3d& candidate) {
    if (!initialized_ || context.stamp_ns != last_cloud_stamp_ns_ ||
        context.alignment_generation != alignment_generation_ ||
        alignment_generation_ == std::numeric_limits<std::uint64_t>::max() ||
        !valid_transform(candidate)) {
        return false;
    }
    alignment_ = candidate;
    ++alignment_generation_;
    return true;
}

bool AlignmentState::initialized() const { return initialized_; }

std::uint64_t AlignmentState::alignment_generation() const {
    return alignment_generation_;
}

const Eigen::Isometry3d& AlignmentState::alignment() const { return alignment_; }

}  // namespace soarm_fixed_map_localization
