#include "soarm_fixed_map_localization/alignment_state.hpp"

#include <cassert>
#include <cmath>
#include <limits>

using soarm_fixed_map_localization::AlignmentState;
using soarm_fixed_map_localization::AlignmentStateOptions;
using soarm_fixed_map_localization::CandidateConsensus;
using soarm_fixed_map_localization::CandidateConsensusOptions;
using soarm_fixed_map_localization::MatchingMode;
using soarm_fixed_map_localization::RecoveryState;
using soarm_fixed_map_localization::RecoveryStateOptions;

namespace {

Eigen::Isometry3d pose(double x, double y, double z, double yaw) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = Eigen::Vector3d(x, y, z);
    result.linear() =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return result;
}

void assert_transform_near(const Eigen::Isometry3d& actual,
                           const Eigen::Isometry3d& expected,
                           double tolerance = 1e-10) {
    assert((actual.matrix() - expected.matrix()).norm() < tolerance);
}

}  // namespace

int main() {
    AlignmentStateOptions options;
    options.odom_capacity = 4;
    options.max_interpolation_span_ns = 200;
    AlignmentState state(options);

    // No alignment means no processable cloud. Invalid and non-monotonic
    // odometry never enter the interpolation buffer.
    assert(!state.begin_cloud(100).has_value());
    assert(!state.add_odometry(0, Eigen::Isometry3d::Identity()));
    assert(state.add_odometry(100, pose(1.0, 0.0, 0.0, 0.0)));
    assert(!state.add_odometry(100, pose(1.0, 0.0, 0.0, 0.0)));
    assert(state.add_odometry(200, pose(3.0, 0.0, 0.0, 0.2)));

    const Eigen::Isometry3d map_to_base = pose(10.0, 5.0, 0.0, -0.1);
    const Eigen::Isometry3d base_to_body = pose(0.0, 0.0, 0.2463, 0.0);
    const Eigen::Isometry3d interpolated_odom = pose(2.0, 0.0, 0.0, 0.1);
    const Eigen::Isometry3d expected_seed =
        map_to_base * base_to_body * interpolated_odom.inverse();

    // Initial pose is evaluated only against time-aligned odometry. A failed
    // seed leaves the existing alignment and generation untouched.
    assert(!state.seed_from_initial_pose(0, map_to_base, base_to_body));
    assert(!state.seed_from_initial_pose(50, map_to_base, base_to_body));
    assert(state.seed_from_initial_pose(150, map_to_base, base_to_body));
    assert(state.initialized());
    assert(state.alignment_generation() == 1);
    assert_transform_near(state.alignment(), expected_seed);
    const Eigen::Isometry3d seed_snapshot = state.alignment();
    assert(!state.seed_from_initial_pose(250, map_to_base, base_to_body));
    assert(state.alignment_generation() == 1);
    assert_transform_near(state.alignment(), seed_snapshot);

    // New odometry does not continuously recompute T_map_cameraInit.
    assert(state.add_odometry(300, pose(8.0, -2.0, 0.0, 0.4)));
    assert_transform_near(state.alignment(), seed_snapshot);

    // A cloud stamp is consumed once. Starting a newer cloud invalidates an
    // older in-flight result even when the alignment generation is unchanged.
    const auto first = state.begin_cloud(160);
    assert(first.has_value());
    assert(!state.begin_cloud(160).has_value());
    const auto second = state.begin_cloud(170);
    assert(second.has_value());
    soarm_localization::Se2 correction{0.1, -0.05, 0.03};
    const Eigen::Vector2d anchor = seed_snapshot.translation().head<2>();
    assert(!state.apply_correction(*first, correction, anchor));
    const Eigen::Isometry3d expected_corrected =
        soarm_fixed_map_localization::compose_candidate(
            seed_snapshot, correction, anchor);
    assert(state.apply_correction(*second, correction, anchor));
    assert(state.alignment_generation() == 2);
    assert_transform_near(state.alignment(), expected_corrected);
    assert(!state.apply_correction(*second, correction, anchor));

    // A stale result cannot commit after an explicit reseed increments the
    // generation. Exact-timestamp odometry is valid and no latest-pose fallback
    // is used for a missing timestamp.
    const auto stale = state.begin_cloud(180);
    assert(stale.has_value());
    assert(state.seed_from_initial_pose(300, map_to_base, base_to_body));
    assert(state.alignment_generation() == 3);
    assert(!state.apply_correction(*stale, correction, anchor));

    const auto current = state.begin_cloud(310);
    assert(current.has_value());
    soarm_localization::Se2 invalid = correction;
    invalid.yaw = std::numeric_limits<double>::quiet_NaN();
    assert(!state.apply_correction(*current, invalid,
                                   state.alignment().translation().head<2>()));
    const Eigen::Isometry3d direct_candidate = pose(4.0, -1.0, 0.3, 0.2);
    assert(state.apply_alignment(*current, direct_candidate));
    assert(state.alignment_generation() == 4);
    assert_transform_near(state.alignment(), direct_candidate);
    assert(!state.apply_alignment(*current, direct_candidate));

    // A correction is committed only after three independent, mutually
    // consistent candidates. Equality at either boundary is fail-closed.
    CandidateConsensus consensus(CandidateConsensusOptions{3, 0.125, 0.075});
    assert(!consensus.observe({1.0, 2.0, 0.10}).has_value());
    assert(!consensus.observe({1.02, 1.99, 0.11}).has_value());
    const auto agreed = consensus.observe({0.99, 2.01, 0.09});
    assert(agreed.has_value());
    assert(std::hypot(agreed->x - 1.0033333333333334,
                      agreed->y - 2.0) < 1e-12);
    assert(std::abs(agreed->yaw - 0.10) < 1e-4);
    assert(consensus.pending_count() == 0);

    assert(!consensus.observe({0.0, 0.0, 0.0}).has_value());
    assert(!consensus.observe({0.125, 0.0, 0.0}).has_value());
    assert(consensus.pending_count() == 0);
    assert(!consensus.observe({0.0, 0.0, 0.0}).has_value());
    assert(!consensus.observe({0.0, 0.0, 0.075}).has_value());
    assert(consensus.pending_count() == 0);

    soarm_localization::Se2 invalid_candidate;
    invalid_candidate.x = std::numeric_limits<double>::quiet_NaN();
    assert(!consensus.observe(invalid_candidate).has_value());
    assert(consensus.pending_count() == 0);

    // Recovery begins only after bounded tracking failures and a measured
    // stationary interval. It never expands indefinitely and a committed
    // correction atomically restores tracking.
    RecoveryState recovery(RecoveryStateOptions{5, 3, 100, 0.02, 0.05});
    assert(recovery.observe_motion(100, 0.0, 0.0));
    for (int rejection = 0; rejection < 4; ++rejection) {
        recovery.tracking_rejected();
    }
    assert(recovery.mode() == MatchingMode::kTracking);
    recovery.tracking_rejected();
    assert(recovery.mode() == MatchingMode::kWaitingForStationary);
    assert(recovery.observe_motion(150, 0.03, 0.0));
    assert(recovery.observe_motion(249, 0.0, 0.0));
    assert(recovery.mode() == MatchingMode::kWaitingForStationary);
    assert(recovery.observe_motion(250, 0.0, 0.0));
    assert(recovery.mode() == MatchingMode::kRecovery);
    recovery.recovery_attempt_failed();
    recovery.recovery_attempt_failed();
    assert(recovery.mode() == MatchingMode::kRecovery);
    recovery.correction_committed();
    assert(recovery.mode() == MatchingMode::kTracking);
    assert(recovery.consecutive_rejections() == 0);

    for (int rejection = 0; rejection < 5; ++rejection) {
        recovery.tracking_rejected();
    }
    recovery.recovery_attempt_failed();
    recovery.recovery_attempt_failed();
    recovery.recovery_attempt_failed();
    assert(recovery.mode() == MatchingMode::kLost);
    recovery.correction_committed();
    assert(recovery.mode() == MatchingMode::kTracking);

    return 0;
}
