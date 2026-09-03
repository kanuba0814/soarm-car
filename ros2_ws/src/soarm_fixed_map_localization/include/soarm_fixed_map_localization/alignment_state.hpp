#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include <soarm_localization/fixed_map_registration.hpp>

namespace soarm_fixed_map_localization {

struct AlignmentStateOptions {
    std::size_t odom_capacity = 200;
    std::int64_t max_interpolation_span_ns = 200000000;
};

struct CloudContext {
    std::int64_t stamp_ns = 0;
    std::uint64_t alignment_generation = 0;
    Eigen::Isometry3d alignment = Eigen::Isometry3d::Identity();
};

Eigen::Isometry3d compose_candidate(const Eigen::Isometry3d& alignment,
                                    const soarm_localization::Se2& delta,
                                    const Eigen::Vector2d& anchor);

struct CandidateConsensusOptions {
    std::size_t confirmation_scans = 3;
    double max_translation = 0.125;
    double max_yaw = 0.075;
};

class CandidateConsensus {
public:
    explicit CandidateConsensus(CandidateConsensusOptions options = {});

    std::optional<soarm_localization::Se2> observe(
        const soarm_localization::Se2& candidate);
    void reset();
    std::size_t pending_count() const;

private:
    CandidateConsensusOptions options_;
    std::vector<soarm_localization::Se2> pending_;
    bool valid_ = false;
};

enum class MatchingMode { kTracking, kWaitingForStationary, kRecovery, kLost };

struct RecoveryStateOptions {
    std::size_t max_tracking_rejections = 5;
    std::size_t max_recovery_attempts = 30;
    std::int64_t stationary_settle_ns = 1000000000;
    double stationary_linear_speed = 0.02;
    double stationary_angular_speed = 0.05;
};

class RecoveryState {
public:
    explicit RecoveryState(RecoveryStateOptions options = {});

    bool observe_motion(std::int64_t stamp_ns, double linear_speed,
                        double angular_speed);
    void tracking_rejected();
    void recovery_attempt_failed();
    void correction_committed();
    MatchingMode mode() const;
    std::size_t consecutive_rejections() const;

private:
    RecoveryStateOptions options_;
    std::size_t consecutive_rejections_ = 0;
    std::size_t recovery_attempts_ = 0;
    std::int64_t latest_stamp_ns_ = 0;
    std::int64_t last_motion_stamp_ns_ = 0;
    bool valid_ = false;
    bool lost_ = false;
};

class AlignmentState {
public:
    explicit AlignmentState(AlignmentStateOptions options = {});

    bool add_odometry(std::int64_t stamp_ns,
                      const Eigen::Isometry3d& camera_init_to_body);
    bool seed_from_initial_pose(std::int64_t stamp_ns,
                                const Eigen::Isometry3d& map_to_base,
                                const Eigen::Isometry3d& base_to_body);
    std::optional<Eigen::Isometry3d> odometry_at(std::int64_t stamp_ns) const;
    std::optional<CloudContext> begin_cloud(std::int64_t stamp_ns);
    bool apply_correction(const CloudContext& context,
                          const soarm_localization::Se2& delta,
                          const Eigen::Vector2d& anchor);
    bool apply_alignment(const CloudContext& context,
                         const Eigen::Isometry3d& candidate);

    bool initialized() const;
    std::uint64_t alignment_generation() const;
    const Eigen::Isometry3d& alignment() const;

private:
    struct OdomSample {
        std::int64_t stamp_ns = 0;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    };

    AlignmentStateOptions options_;
    std::deque<OdomSample> odometry_;
    Eigen::Isometry3d alignment_ = Eigen::Isometry3d::Identity();
    std::uint64_t alignment_generation_ = 0;
    std::int64_t last_cloud_stamp_ns_ = 0;
    bool initialized_ = false;
};

}  // namespace soarm_fixed_map_localization
