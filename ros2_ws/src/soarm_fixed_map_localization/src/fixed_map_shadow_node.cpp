#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "soarm_fixed_map_localization/alignment_state.hpp"
#include <soarm_localization/fixed_map_registration.hpp>

namespace {

using Points = std::vector<Eigen::Vector3d>;
using Iso = Eigen::Isometry3d;

std::optional<Iso> pose_to_iso(const geometry_msgs::msg::Pose& pose) {
    const auto& orientation = pose.orientation;
    Eigen::Quaterniond rotation(orientation.w, orientation.x, orientation.y,
                                orientation.z);
    const Eigen::Vector3d translation(pose.position.x, pose.position.y,
                                      pose.position.z);
    if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
        rotation.norm() < 1e-9) {
        return std::nullopt;
    }
    rotation.normalize();
    Iso result = Iso::Identity();
    result.linear() = rotation.toRotationMatrix();
    result.translation() = translation;
    return result;
}

geometry_msgs::msg::Quaternion quaternion(const Eigen::Matrix3d& matrix) {
    Eigen::Quaterniond value(matrix);
    value.normalize();
    geometry_msgs::msg::Quaternion result;
    result.x = value.x();
    result.y = value.y();
    result.z = value.z();
    result.w = value.w();
    return result;
}

std::string number(double value) {
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

soarm_localization::Se2 planar_pose(const Iso& transform) {
    return {transform.translation().x(), transform.translation().y(),
            std::atan2(transform.linear()(1, 0), transform.linear()(0, 0))};
}

soarm_localization::Se2 local_delta(
    const soarm_localization::Se2& anchor,
    const soarm_localization::Se2& map_pose) {
    const double dx = map_pose.x - anchor.x;
    const double dy = map_pose.y - anchor.y;
    const double cosine = std::cos(anchor.yaw);
    const double sine = std::sin(anchor.yaw);
    return {cosine * dx + sine * dy, -sine * dx + cosine * dy,
            std::remainder(map_pose.yaw - anchor.yaw, 2.0 * std::acos(-1.0))};
}

Iso replace_planar_pose(const Iso& trusted,
                        const soarm_localization::Se2& planar) {
    const double trusted_yaw =
        std::atan2(trusted.linear()(1, 0), trusted.linear()(0, 0));
    const Eigen::Matrix3d tilt =
        Eigen::AngleAxisd(-trusted_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        trusted.linear();
    Iso result = trusted;
    result.translation().x() = planar.x;
    result.translation().y() = planar.y;
    result.linear() =
        Eigen::AngleAxisd(planar.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        tilt;
    return result;
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
    return rclcpp::Time(stamp).nanoseconds();
}

class FixedMapShadow final : public rclcpp::Node {
public:
    FixedMapShadow()
        : Node("fixed_map_shadow"), options_(registration_options()),
          alignment_state_(alignment_options()) {
        map_path_ = declare_parameter<std::string>("map_file_path", "");
        base_to_body_z_ = declare_parameter<double>("base_to_body_z", 0.232);
        structural_min_ =
            declare_parameter<double>("structural_min_height", 0.15);
        structural_max_ =
            declare_parameter<double>("structural_max_height", 1.80);
        floor_bin_ = declare_parameter<double>("floor_bin", 0.05);
        min_floor_points_ = declare_parameter<int>("min_floor_points", 20);
        min_cells_ = declare_parameter<int>("min_cells", 20);
        max_source_cells_ = declare_parameter<int>("max_source_cells", 1200);
        rank_tolerance_ =
            declare_parameter<double>("observability_rank_tolerance", 0.01);

        // Shadow remains fail-closed until field evidence justifies every policy.
        accept_enabled_ = declare_parameter<bool>("accept_enabled", false);
        min_score_gap_ = declare_parameter<double>("min_score_gap", 0.0);
        min_inlier_fraction_ =
            declare_parameter<double>("min_inlier_fraction", 0.0);
        max_mse_ = declare_parameter<double>("max_mse", 0.0);
        min_rank_ = declare_parameter<int>("min_observability_rank", 3);
        const std::int64_t confirmation_scans =
            declare_parameter<int>("confirmation_scans", 3);
        soarm_fixed_map_localization::CandidateConsensusOptions consensus_options;
        consensus_options.confirmation_scans = confirmation_scans > 0
                                                  ? static_cast<std::size_t>(confirmation_scans)
                                                  : 0;
        consensus_options.max_translation =
            declare_parameter<double>("consensus_max_translation", 0.125);
        consensus_options.max_yaw =
            declare_parameter<double>("consensus_max_yaw", 0.075);
        consensus_ =
            std::make_unique<soarm_fixed_map_localization::CandidateConsensus>(
                consensus_options);
        recovery_enabled_ = declare_parameter<bool>("recovery_enabled", false);
        soarm_fixed_map_localization::RecoveryStateOptions recovery_options;
        const std::int64_t max_tracking_rejections =
            declare_parameter<int>("max_tracking_rejections", 5);
        const std::int64_t max_recovery_attempts =
            declare_parameter<int>("max_recovery_attempts", 30);
        const double stationary_settle =
            declare_parameter<double>("stationary_settle_seconds", 1.0);
        recovery_options.max_tracking_rejections =
            max_tracking_rejections > 0
                ? static_cast<std::size_t>(max_tracking_rejections)
                : 0;
        recovery_options.max_recovery_attempts =
            max_recovery_attempts > 0
                ? static_cast<std::size_t>(max_recovery_attempts)
                : 0;
        recovery_options.stationary_settle_ns =
            std::isfinite(stationary_settle) && stationary_settle > 0.0
                ? static_cast<std::int64_t>(stationary_settle * 1e9)
                : 0;
        recovery_options.stationary_linear_speed =
            declare_parameter<double>("stationary_linear_speed", 0.02);
        recovery_options.stationary_angular_speed =
            declare_parameter<double>("stationary_angular_speed", 0.05);
        recovery_state_ =
            std::make_unique<soarm_fixed_map_localization::RecoveryState>(
                recovery_options);

        search_.xy_extent = declare_parameter<double>("search_xy_extent", 0.25);
        search_.yaw_extent = declare_parameter<double>("search_yaw_extent", 0.15);
        search_.xy_step = declare_parameter<double>("search_xy_step", 0.02);
        search_.yaw_step = declare_parameter<double>("search_yaw_step", 0.02);
        search_.nms_radius = declare_parameter<double>("peak_nms_radius", 0.08);
        const std::int64_t peak_count =
            declare_parameter<int>("peak_count", 4);
        search_.peak_count = peak_count > 0
                                 ? static_cast<std::size_t>(peak_count)
                                 : 0;
        refinement_.max_relative_x =
            declare_parameter<double>("refine_max_relative_x", 0.10);
        refinement_.max_relative_y =
            declare_parameter<double>("refine_max_relative_y", 0.10);
        refinement_.max_relative_yaw =
            declare_parameter<double>("refine_max_relative_yaw", 0.10);
        recovery_search_.xy_extent =
            declare_parameter<double>("recovery_xy_extent", 0.75);
        recovery_search_.yaw_extent =
            declare_parameter<double>("recovery_yaw_extent", 0.35);
        recovery_search_.xy_step =
            declare_parameter<double>("recovery_xy_step", 0.05);
        recovery_search_.yaw_step =
            declare_parameter<double>("recovery_yaw_step", 0.05);
        recovery_search_.nms_radius = search_.nms_radius;
        recovery_search_.peak_count = search_.peak_count;

        candidate_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/fixed_map_shadow/candidate", 1);
        diagnostic_pub_ =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/fixed_map_shadow/diagnostics", 1);
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
                on_cloud(*message);
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr message) {
                on_odometry(*message);
            });
        initial_sub_ =
            create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose", 1,
                [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
                    on_initial_pose(*message);
                });
        const std::string motion_odom_topic =
            declare_parameter<std::string>("motion_odom_topic", "/wheel/odom_raw");
        motion_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            motion_odom_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr message) {
                on_motion_odometry(*message);
            });
        load_map();
    }

private:
    soarm_localization::Options registration_options() {
        soarm_localization::Options options;
        options.voxel_resolution =
            declare_parameter<double>("voxel_resolution", 0.10);
        options.max_correspondence_distance =
            declare_parameter<double>("max_correspondence_distance", 0.75);
        options.min_cells = declare_parameter<int>("core_min_cells", 20);
        options.max_cells = 1200;
        options.max_iterations = declare_parameter<int>("max_iterations", 30);
        options.threads = 1;
        options.distance_field_resolution =
            declare_parameter<double>("distance_field_resolution", 0.01);
        options.capped_squared_loss =
            declare_parameter<double>("capped_squared_loss", 0.04);
        return options;
    }

    soarm_fixed_map_localization::AlignmentStateOptions alignment_options() {
        soarm_fixed_map_localization::AlignmentStateOptions options;
        const std::int64_t odom_capacity =
            declare_parameter<int>("odom_buffer_capacity", 200);
        options.odom_capacity = odom_capacity >= 2
                                    ? static_cast<std::size_t>(odom_capacity)
                                    : 2;
        const double interpolation_seconds =
            declare_parameter<double>("max_odom_interpolation_span", 0.20);
        if (std::isfinite(interpolation_seconds) && interpolation_seconds > 0.0 &&
            interpolation_seconds <=
                static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9) {
            options.max_interpolation_span_ns = static_cast<std::int64_t>(
                interpolation_seconds * 1e9);
        } else {
            options.max_interpolation_span_ns = 1;
        }
        return options;
    }

    Iso base_to_body() const {
        Iso transform = Iso::Identity();
        transform.translation().z() = base_to_body_z_;
        return transform;
    }

    bool floor_height(const Points& points, double* value) const {
        if (!std::isfinite(floor_bin_) || floor_bin_ <= 0.0 || points.empty()) {
            return false;
        }
        std::map<std::int64_t, std::vector<double>> bins;
        for (const Eigen::Vector3d& point : points) {
            if (!point.allFinite()) {
                return false;
            }
            const double key_value = std::floor(point.z() / floor_bin_);
            if (!std::isfinite(key_value) ||
                key_value < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
                key_value > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
                return false;
            }
            bins[static_cast<std::int64_t>(key_value)].push_back(point.z());
        }
        const auto found = std::find_if(
            bins.begin(), bins.end(), [this](const auto& bin) {
                return bin.second.size() >=
                       static_cast<std::size_t>(std::max(0, min_floor_points_));
            });
        if (found == bins.end()) {
            return false;
        }
        double sum = 0.0;
        for (double height : found->second) {
            sum += height;
        }
        *value = sum / static_cast<double>(found->second.size());
        const auto bounds = std::minmax_element(found->second.begin(), found->second.end());
        return std::isfinite(*value) && *bounds.second - *bounds.first <= floor_bin_ + 1e-9;
    }

    std::optional<Points> structural_cells(const Points& input,
                                            double floor) const {
        if (!std::isfinite(options_.voxel_resolution) ||
            options_.voxel_resolution <= 0.0 || !std::isfinite(floor)) {
            return std::nullopt;
        }
        using Key = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
        std::map<Key, Eigen::Vector3d> voxels;
        for (const Eigen::Vector3d& point : input) {
            if (!point.allFinite()) {
                return std::nullopt;
            }
            const double height = point.z() - floor;
            if (height < structural_min_ || height > structural_max_) {
                continue;
            }
            std::array<std::int64_t, 3> key{};
            for (int axis = 0; axis < 3; ++axis) {
                const double value =
                    std::floor(point[axis] / options_.voxel_resolution);
                if (!std::isfinite(value) ||
                    value < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
                    value > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
                    return std::nullopt;
                }
                key[axis] = static_cast<std::int64_t>(value);
            }
            voxels.emplace(Key{key[0], key[1], key[2]}, point);
        }
        Points result;
        result.reserve(voxels.size());
        for (const auto& entry : voxels) {
            result.push_back(entry.second);
        }
        return result;
    }

    void load_map() {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (map_path_.empty() || pcl::io::loadPCDFile(map_path_, cloud) < 0) {
            publish_diagnostic("map_load_failed",
                               diagnostic_msgs::msg::DiagnosticStatus::ERROR);
            return;
        }
        Points raw;
        raw.reserve(cloud.size());
        for (const pcl::PointXYZ& point : cloud) {
            raw.emplace_back(point.x, point.y, point.z);
        }
        double floor = 0.0;
        if (!floor_height(raw, &floor)) {
            publish_diagnostic("invalid_map_floor_support",
                               diagnostic_msgs::msg::DiagnosticStatus::ERROR);
            return;
        }
        const std::optional<Points> cells = structural_cells(raw, floor);
        if (!cells.has_value() ||
            cells->size() < static_cast<std::size_t>(std::max(0, min_cells_))) {
            publish_diagnostic("invalid_map_structural_cells",
                               diagnostic_msgs::msg::DiagnosticStatus::ERROR);
            return;
        }
        map_cells_ = *cells;
        registration_ =
            std::make_unique<soarm_localization::FixedMapRegistration>(options_);
        if (!registration_->set_map(map_cells_)) {
            registration_.reset();
            publish_diagnostic("full_map_model_rejected",
                               diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                               0, map_cells_.size());
            return;
        }
        map_ready_ = true;
        const auto stats = registration_->model_stats();
        publish_diagnostic("full_map_model_loaded",
                           diagnostic_msgs::msg::DiagnosticStatus::OK, 0,
                           stats.map_cells, nullptr, nullptr, 0,
                           stats.build_seconds, false);
    }

    void on_odometry(const nav_msgs::msg::Odometry& message) {
        if (message.header.frame_id != "camera_init" ||
            message.child_frame_id != "body") {
            publish_diagnostic("invalid_odom_frames",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN);
            return;
        }
        const std::optional<Iso> pose = pose_to_iso(message.pose.pose);
        if (!pose.has_value() ||
            !alignment_state_.add_odometry(stamp_ns(message.header.stamp), *pose)) {
            publish_diagnostic("invalid_or_nonmonotonic_odom",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN);
            return;
        }
        if (pending_initial_.has_value()) {
            try_seed();
        }
    }

    void on_motion_odometry(const nav_msgs::msg::Odometry& message) {
        motion_linear_speed_ = std::hypot(message.twist.twist.linear.x,
                                         message.twist.twist.linear.y);
        motion_angular_speed_ = std::abs(message.twist.twist.angular.z);
        motion_odom_received_ = recovery_state_->observe_motion(
            stamp_ns(message.header.stamp), motion_linear_speed_,
            motion_angular_speed_) || motion_odom_received_;
    }

    void on_initial_pose(
        const geometry_msgs::msg::PoseWithCovarianceStamped& message) {
        if (message.header.frame_id != "map" || stamp_ns(message.header.stamp) <= 0 ||
            !pose_to_iso(message.pose.pose).has_value()) {
            publish_diagnostic("invalid_initialpose",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN);
            return;
        }
        pending_initial_ = message;
        try_seed();
    }

    void try_seed() {
        if (!pending_initial_.has_value()) {
            return;
        }
        const std::optional<Iso> map_to_base =
            pose_to_iso(pending_initial_->pose.pose);
        if (!map_to_base.has_value() ||
            !alignment_state_.seed_from_initial_pose(
                stamp_ns(pending_initial_->header.stamp), *map_to_base,
                base_to_body())) {
            publish_diagnostic("initialpose_awaiting_time_aligned_odom",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN);
            return;
        }
        pending_initial_.reset();
        consensus_->reset();
        recovery_state_->correction_committed();
        publish_diagnostic("alignment_seeded",
                           diagnostic_msgs::msg::DiagnosticStatus::OK);
    }

    void on_cloud(const sensor_msgs::msg::PointCloud2& message) {
        const auto started = std::chrono::steady_clock::now();
        if (!map_ready_ || message.header.frame_id != "camera_init") {
            publish_diagnostic(map_ready_ ? "invalid_cloud_frame" : "map_not_ready",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN);
            return;
        }
        const std::int64_t cloud_stamp = stamp_ns(message.header.stamp);
        const auto context = alignment_state_.begin_cloud(cloud_stamp);
        const auto odometry = alignment_state_.odometry_at(cloud_stamp);
        if (!context.has_value() || !odometry.has_value()) {
            publish_diagnostic("cloud_awaiting_alignment_or_time_aligned_odom",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN);
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(message, cloud);
        Points raw;
        raw.reserve(cloud.size());
        for (const pcl::PointXYZ& point : cloud) {
            raw.emplace_back(point.x, point.y, point.z);
        }
        const double source_floor =
            (*odometry * base_to_body().inverse()).translation().z();
        const std::optional<Points> source = structural_cells(raw, source_floor);
        if (!source.has_value() ||
            source->size() < static_cast<std::size_t>(std::max(0, min_cells_)) ||
            source->size() > static_cast<std::size_t>(std::max(0, max_source_cells_))) {
            publish_diagnostic("insufficient_or_excess_source_cells",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN,
                               source.has_value() ? source->size() : 0);
            return;
        }

        if (!registration_) {
            publish_diagnostic("full_map_model_not_ready",
                               diagnostic_msgs::msg::DiagnosticStatus::ERROR);
            return;
        }
        const auto matching_mode = recovery_state_->mode();
        if (recovery_enabled_ &&
            matching_mode ==
                soarm_fixed_map_localization::MatchingMode::kWaitingForStationary) {
            publish_diagnostic("recovery_waiting_for_stationary",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN,
                               source->size(), registration_->cell_count());
            return;
        }
        if (recovery_enabled_ &&
            matching_mode == soarm_fixed_map_localization::MatchingMode::kLost) {
            publish_diagnostic("recovery_exhausted",
                               diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                               source->size(), registration_->cell_count());
            return;
        }
        const bool recovering =
            recovery_enabled_ &&
            matching_mode == soarm_fixed_map_localization::MatchingMode::kRecovery;
        const soarm_localization::Se2 anchor = planar_pose(context->alignment);
        const auto pre = registration_->search_around(
            *source, anchor, recovering ? recovery_search_ : search_);
        if (!pre.accepted_input || pre.peaks.empty()) {
            if (accept_enabled_ && recovery_enabled_) {
                recovering ? recovery_state_->recovery_attempt_failed()
                           : recovery_state_->tracking_rejected();
            }
            publish_diagnostic("presearch_rejected",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN,
                               source->size(), registration_->cell_count());
            return;
        }
        const soarm_localization::Se2 initial_delta =
            local_delta(anchor, pre.peaks.front().pose);
        const soarm_localization::Result best =
            registration_->refine_bounded_around(
                *source, anchor, initial_delta, refinement_);
        if (!best.accepted_input || !std::isfinite(best.transform.x) ||
            !std::isfinite(best.transform.y) ||
            !std::isfinite(best.transform.yaw)) {
            if (recovering) {
                recovery_state_->recovery_attempt_failed();
            }
            publish_diagnostic("bounded_refinement_rejected",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN,
                               source->size(), registration_->cell_count(), &pre,
                               &best);
            return;
        }
        const int rank =
            best.observability.rank_for_relative_tolerance(rank_tolerance_);
        const bool qualifying =
            accept_enabled_ && pre.peaks.size() > 1 &&
            std::isfinite(pre.score_gap) && pre.score_gap >= min_score_gap_ &&
            best.converged && best.bounded_refinement_used &&
            best.all_evaluations_within_bounds &&
            best.inlier_fraction >= min_inlier_fraction_ &&
            best.mean_squared_error <= max_mse_ && rank >= min_rank_;

        const Iso raw_hypothesis =
            replace_planar_pose(context->alignment, best.transform);
        publish_candidate(message.header.stamp, raw_hypothesis);
        if (!qualifying) {
            consensus_->reset();
            if (accept_enabled_ && recovery_enabled_) {
                recovering ? recovery_state_->recovery_attempt_failed()
                           : recovery_state_->tracking_rejected();
            }
        }
        const std::optional<soarm_localization::Se2> consensus =
            qualifying ? consensus_->observe(planar_pose(raw_hypothesis))
                       : std::nullopt;
        if (qualifying && !consensus.has_value()) {
            if (recovering) {
                recovery_state_->recovery_attempt_failed();
            }
            const double elapsed = std::chrono::duration<double>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
            publish_diagnostic("candidate_pending_consensus",
                               diagnostic_msgs::msg::DiagnosticStatus::WARN,
                               source->size(), registration_->cell_count(), &pre,
                               &best, rank, elapsed, true);
            return;
        }
        bool accepted = false;
        if (consensus.has_value()) {
            const Iso hypothesis =
                replace_planar_pose(context->alignment, *consensus);
            if (!alignment_state_.apply_alignment(*context, hypothesis)) {
                publish_diagnostic("stale_candidate_not_committed",
                                   diagnostic_msgs::msg::DiagnosticStatus::WARN,
                                   source->size(), registration_->cell_count());
                return;
            }
            recovery_state_->correction_committed();
            accepted = true;
        }
        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - started)
                                   .count();
        publish_diagnostic(
            accepted ? "accepted_shadow_consensus" : "candidate_not_accepted",
            accepted ? diagnostic_msgs::msg::DiagnosticStatus::OK
                     : diagnostic_msgs::msg::DiagnosticStatus::WARN,
            source->size(), registration_->cell_count(), &pre, &best, rank,
            elapsed, true);
    }

    void publish_candidate(const builtin_interfaces::msg::Time& stamp,
                           const Iso& transform) {
        if (!transform.matrix().allFinite()) {
            return;
        }
        nav_msgs::msg::Odometry output;
        output.header.stamp = stamp;
        output.header.frame_id = "map";
        output.child_frame_id = "camera_init";
        output.pose.pose.position.x = transform.translation().x();
        output.pose.pose.position.y = transform.translation().y();
        output.pose.pose.position.z = transform.translation().z();
        output.pose.pose.orientation = quaternion(transform.linear());
        candidate_pub_->publish(output);
    }

    void publish_diagnostic(
        const std::string& reason, int level, std::size_t source_cells = 0,
        std::size_t map_cells = 0,
        const soarm_localization::PreRefinementResult* pre = nullptr,
        const soarm_localization::Result* result = nullptr, int rank = 0,
        double total_seconds = std::numeric_limits<double>::quiet_NaN(),
        bool cache_reused = false) {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "fixed_map_shadow";
        status.hardware_id = "fixed-map-shadow";
        status.level = level;
        status.message = reason;
        auto add = [&status](const std::string& key, const std::string& value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key;
            entry.value = value;
            status.values.push_back(entry);
        };
        add("candidate_semantics", "T_map_cameraInit");
        add("map_loaded", map_ready_ ? "true" : "false");
        add("alignment_initialized",
            alignment_state_.initialized() ? "true" : "false");
        add("alignment_generation",
            std::to_string(alignment_state_.alignment_generation()));
        add("consensus_pending",
            consensus_ ? std::to_string(consensus_->pending_count()) : "0");
        const auto mode = recovery_state_
                              ? recovery_state_->mode()
                              : soarm_fixed_map_localization::MatchingMode::kLost;
        add("matching_mode",
            mode == soarm_fixed_map_localization::MatchingMode::kTracking
                ? "tracking"
                : mode == soarm_fixed_map_localization::MatchingMode::kRecovery
                      ? "recovery"
                      : mode == soarm_fixed_map_localization::MatchingMode::kWaitingForStationary
                            ? "waiting_for_stationary"
                            : "lost");
        add("consecutive_rejections",
            recovery_state_
                ? std::to_string(recovery_state_->consecutive_rejections())
                : "0");
        add("motion_odom_received", motion_odom_received_ ? "true" : "false");
        add("motion_linear_speed", number(motion_linear_speed_));
        add("motion_angular_speed", number(motion_angular_speed_));
        add("source_cells", std::to_string(source_cells));
        add("map_cells", std::to_string(map_cells));
        add("model_reused", cache_reused ? "true" : "false");
        add("pre_best_score",
            pre && !pre->peaks.empty() ? number(pre->peaks.front().score) : "nan");
        add("pre_second_score",
            pre && pre->peaks.size() > 1 ? number(pre->peaks[1].score) : "nan");
        add("pre_score_gap", pre ? number(pre->score_gap) : "nan");
        add("search_seconds", pre ? number(pre->metrics.wall_seconds) : "nan");
        add("observability_rank", std::to_string(rank));
        add("observability_eigen_ratio",
            result ? number(result->observability.min_trace_ratio) : "nan");
        add("refinement_mse",
            result ? number(result->mean_squared_error) : "nan");
        add("inlier_fraction",
            result ? number(result->inlier_fraction) : "nan");
        add("refinement_seconds", result ? number(result->wall_seconds) : "nan");
        add("refinement_converged",
            result && result->converged ? "true" : "false");
        add("refinement_evaluations",
            result ? std::to_string(result->bounded_evaluations) : "0");
        add("trust_region_limited",
            result && result->trust_region_limited ? "true" : "false");
        add("all_evaluations_within_bounds",
            result && result->all_evaluations_within_bounds ? "true" : "false");
        add("total_seconds", number(total_seconds));
        array.status.push_back(status);
        diagnostic_pub_->publish(array);
    }

    std::string map_path_;
    double base_to_body_z_ = 0.0;
    double structural_min_ = 0.0;
    double structural_max_ = 0.0;
    double floor_bin_ = 0.0;
    double rank_tolerance_ = 0.0;
    double min_score_gap_ = 0.0;
    double min_inlier_fraction_ = 0.0;
    double max_mse_ = 0.0;
    int min_floor_points_ = 0;
    int min_cells_ = 0;
    int max_source_cells_ = 0;
    int min_rank_ = 0;
    bool map_ready_ = false;
    bool accept_enabled_ = false;
    bool recovery_enabled_ = false;
    bool motion_odom_received_ = false;
    double motion_linear_speed_ = std::numeric_limits<double>::quiet_NaN();
    double motion_angular_speed_ = std::numeric_limits<double>::quiet_NaN();
    soarm_localization::Options options_;
    soarm_localization::SearchProfile search_;
    soarm_localization::SearchProfile recovery_search_;
    soarm_localization::RefinementProfile refinement_;
    soarm_fixed_map_localization::AlignmentState alignment_state_;
    std::unique_ptr<soarm_fixed_map_localization::CandidateConsensus> consensus_;
    std::unique_ptr<soarm_fixed_map_localization::RecoveryState> recovery_state_;
    Points map_cells_;
    std::unique_ptr<soarm_localization::FixedMapRegistration> registration_;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pending_initial_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motion_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initial_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr candidate_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostic_pub_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixedMapShadow>());
    rclcpp::shutdown();
    return 0;
}
