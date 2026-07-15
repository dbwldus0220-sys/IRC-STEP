#pragma once
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using Eigen::Matrix4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace BRP_Kinematics{

#ifdef STEP_DEBUG_IK_SMALL_ROLL_BRANCH_TRACKING
struct SmallRollBranchCandidate {
    Eigen::VectorXd theta;
    double total_cost = std::numeric_limits<double>::infinity();
};

inline double wrappedAngleDifference(double angle, double reference)
{
    return std::atan2(
        std::sin(angle - reference),
        std::cos(angle - reference)
    );
}

inline SmallRollBranchCandidate selectSmallRollBranch(
    const Eigen::VectorXd& solved_theta,
    const Eigen::VectorXd& previous_theta,
    double pose_error
)
{
    constexpr double two_pi = 6.28318530717958647692;
    constexpr int roll_joint_indices[] = {1, 5};
    constexpr double pose_error_weight =
        static_cast<double>(STEP_DEBUG_IK_BRANCH_POSE_ERROR_WEIGHT);
    constexpr double branch_weight =
        static_cast<double>(STEP_DEBUG_IK_BRANCH_WEIGHT);
    constexpr double roll_weight =
        static_cast<double>(STEP_DEBUG_IK_SMALL_ROLL_WEIGHT);

    Eigen::VectorXd nearest_branch = solved_theta;
    for (Eigen::Index joint_index = 0;
         joint_index < nearest_branch.size(); ++joint_index) {
        nearest_branch(joint_index) = previous_theta(joint_index)
            + wrappedAngleDifference(
                solved_theta(joint_index),
                previous_theta(joint_index)
            );
    }

    SmallRollBranchCandidate best{nearest_branch};
    for (int hip_turn = -1; hip_turn <= 1; ++hip_turn) {
        for (int ankle_turn = -1; ankle_turn <= 1; ++ankle_turn) {
            Eigen::VectorXd candidate = nearest_branch;
            candidate(roll_joint_indices[0]) += hip_turn * two_pi;
            candidate(roll_joint_indices[1]) += ankle_turn * two_pi;

            double branch_distance_squared = 0.0;
            for (Eigen::Index joint_index = 0;
                 joint_index < candidate.size(); ++joint_index) {
                const double wrapped_delta = wrappedAngleDifference(
                    candidate(joint_index),
                    previous_theta(joint_index)
                );
                branch_distance_squared += wrapped_delta * wrapped_delta;
            }
            const double roll_amplitude_squared =
                candidate(roll_joint_indices[0])
                    * candidate(roll_joint_indices[0])
                + candidate(roll_joint_indices[1])
                    * candidate(roll_joint_indices[1]);
            const double total_cost = pose_error_weight * pose_error
                + branch_weight * branch_distance_squared
                + roll_weight * roll_amplitude_squared;

            if (std::isfinite(total_cost) && total_cost < best.total_cost) {
                best.theta = candidate;
                best.total_cost = total_cost;
            }
        }
    }
    return best;
}

template <typename ForwardKinematics>
inline SmallRollBranchCandidate selectBranchTrackingStep(
    const Eigen::VectorXd& current_theta,
    const Eigen::VectorXd& delta_theta,
    const Eigen::VectorXd& previous_theta,
    const Eigen::VectorXd& target_pose,
    const Eigen::VectorXd& link,
    ForwardKinematics&& forward_kinematics
)
{
    constexpr double step_scales[] = {0.0, 0.0625, 0.125, 0.25, 0.5, 1.0};
    constexpr double pose_error_weight =
        static_cast<double>(STEP_DEBUG_IK_BRANCH_POSE_ERROR_WEIGHT);
    constexpr double branch_weight =
        static_cast<double>(STEP_DEBUG_IK_BRANCH_WEIGHT);
    constexpr double roll_weight =
        static_cast<double>(STEP_DEBUG_IK_SMALL_ROLL_WEIGHT);

    SmallRollBranchCandidate best{current_theta};
    Eigen::VectorXd candidate_pose(target_pose.size());
    for (double step_scale : step_scales) {
        Eigen::VectorXd candidate = current_theta + step_scale * delta_theta;
        for (Eigen::Index joint_index = 0;
             joint_index < candidate.size(); ++joint_index) {
            candidate(joint_index) = previous_theta(joint_index)
                + wrappedAngleDifference(
                    candidate(joint_index),
                    previous_theta(joint_index)
                );
        }

        forward_kinematics(candidate, link, candidate_pose);
        Eigen::VectorXd pose_difference = target_pose - candidate_pose;
        for (Eigen::Index pose_index = 3;
             pose_index < pose_difference.size(); ++pose_index) {
            pose_difference(pose_index) = std::atan2(
                std::sin(pose_difference(pose_index)),
                std::cos(pose_difference(pose_index))
            );
        }
        const double pose_error = pose_difference.norm();

        double branch_distance_squared = 0.0;
        for (Eigen::Index joint_index = 0;
             joint_index < candidate.size(); ++joint_index) {
            const double wrapped_delta = wrappedAngleDifference(
                candidate(joint_index),
                previous_theta(joint_index)
            );
            branch_distance_squared += wrapped_delta * wrapped_delta;
        }
        const double roll_amplitude_squared =
            candidate(1) * candidate(1) + candidate(5) * candidate(5);
        const double total_cost = pose_error_weight * pose_error
            + branch_weight * branch_distance_squared
            + roll_weight * roll_amplitude_squared;

        if (std::isfinite(total_cost) && total_cost < best.total_cost) {
            best.theta = candidate;
            best.total_cost = total_cost;
        }
    }
    return best;
}
#endif

#ifdef STEP_DEBUG_SIMULATION
inline double last_RL_condition_number = std::numeric_limits<double>::infinity();
inline double last_LL_condition_number = std::numeric_limits<double>::infinity();
inline double last_RL_min_singular_value = 0.0;
inline double last_LL_min_singular_value = 0.0;
inline int last_RL_iteration_count = 0;
inline int last_LL_iteration_count = 0;
inline double last_RL_final_ERR = std::numeric_limits<double>::infinity();
inline double last_LL_final_ERR = std::numeric_limits<double>::infinity();
inline bool last_RL_converged = false;
inline bool last_LL_converged = false;
inline double last_RL_max_abs_delta_theta = 0.0;
inline double last_LL_max_abs_delta_theta = 0.0;
inline double last_RL_delta_theta_1 = 0.0;
inline double last_RL_delta_theta_5 = 0.0;
inline double last_LL_delta_theta_1 = 0.0;
inline double last_LL_delta_theta_5 = 0.0;
inline double last_RL_adaptive_lambda = 0.0;
inline double last_LL_adaptive_lambda = 0.0;
inline double last_RL_max_iteration_delta_norm = 0.0;
inline double last_LL_max_iteration_delta_norm = 0.0;
inline double last_RL_continuity_cost = 0.0;
inline double last_LL_continuity_cost = 0.0;
inline double last_RL_pose_error = std::numeric_limits<double>::infinity();
inline double last_LL_pose_error = std::numeric_limits<double>::infinity();
inline double last_RL_joint_distance_from_init = 0.0;
inline double last_LL_joint_distance_from_init = 0.0;
inline double last_RL_roll_pair_sum = 0.0;
inline double last_LL_roll_pair_sum = 0.0;
inline double last_RL_roll_pair_cost = 0.0;
inline double last_LL_roll_pair_cost = 0.0;
inline bool last_RL_ldlt_fallback_used = false;
inline bool last_LL_ldlt_fallback_used = false;
inline bool last_RL_convergence_fallback_used = false;
inline bool last_LL_convergence_fallback_used = false;
#endif

// th: [6x1], link: [7x1], PR: [6x1] (x, y, z, roll, pitch, yaw)
void BRP_RL_FK(const VectorXd& th, const VectorXd& link, VectorXd& PR){

    double t1 = th(0), t2 = th(1), t3 = th(2), t4 = th(3), t5 = th(4), t6 = th(5);
    double L0 = link(0), L1 = link(1), L2 = link(2), L3 = link(3), L4 = link(4), L5 = link(5), L6 = link(6);

    double c1 = cos(t1), c2 = cos(t2), c3 = cos(t3), c4 = cos(t4), c5 = cos(t5), c6 = cos(t6);
    double s1 = sin(t1), s2 = sin(t2), s3 = sin(t3), s4 = sin(t4), s5 = sin(t5), s6 = sin(t6);

    // x, y, z position setup
    PR(0) = L5 * s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))- L4 * s4 * (c1 * c3 - s1 * s2 * s3)
          - L5 * c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2))- L3 * c1 * s3 - L4 * c4 * (c1 * s3 + c3 * s1 * s2)- L2 * s1 * s2
          - L6 * c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)))
          - L3 * c3 * s1 * s2 - L6 * c2 * s1 * s6;

    PR(1) = L2 * c1 * s2 - L4 * c4 * (s1 * s3 - c1 * c3 * s2) - L4 * s4 * (c3 * s1 + c1 * s2 * s3)
          - L5 * c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2))- L0
          + L5 * s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - L3 * s1 * s3
          - L6 * c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)))
          + L3 * c1 * c3 * s2 + L6 * c1 * c2 * s6;

    PR(2) = L6 * s2 * s6 - L2 * c2 - L6 * c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3))
          - L3 * c2 * c3 - L1 - L5 * c5 * (c2 * c3 * c4 - c2 * s3 * s4) + L5 * s5 * (c2 * c3 * s4 + c2 * c4 * s3) - L4 * c2 * c3 * c4 + L4 * c2 * s3 * s4;

    // Orientation Calculation (n, o, a vector)
    double nx = -c5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - s5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2));
    double ny = -c5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - s5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2));
    double nz = -c5 * (c2 * c3 * s4 + c2 * c4 * s3) - s5 * (c2 * c3 * c4 - c2 * s3 * s4);

    double ox = s6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - c2 * c6 * s1;
    double oy = s6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + c1 * c2 * c6;
    double oz = c6 * s2 + s6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3));

    double ax = c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) + c2 * s1 * s6;
    double ay = c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) - c1 * c2 * s6;
    double az = c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - s2 * s6;

    // RPY 오일러 각
    PR(5) = atan2(ny, nx);                                                                     // Roll (z축)
    PR(4) = atan2(-nz, cos(PR(5)) * nx + sin(PR(5)) * ny);                                     // Pitch (y축)
    PR(3) = atan2(sin(PR(5)) * ax - cos(PR(5)) * ay, -sin(PR(5)) * ox + cos(PR(5)) * oy);      // Yaw (x축)
}


void BRP_RL_IK(const VectorXd& target_PR, const VectorXd& init_theta, const VectorXd& link, VectorXd& IK_theta)
{    
    const int dof = 6;
    Eigen::VectorXd th = init_theta;
    Eigen::VectorXd PR(dof), old_PR(dof), F(dof), old_Q(dof);
    Eigen::MatrixXd J(dof, dof), Inv_J(dof, dof), New_PR(dof, dof);
    Eigen::VectorXd New_Q4J(dof);
    double del_Q = 0.0001, ERR = 0.0;
    int iter, i, j, k;
    double sum = 0.0;

#ifdef STEP_DEBUG_SIMULATION
    last_RL_condition_number = std::numeric_limits<double>::infinity();
    last_RL_min_singular_value = 0.0;
    last_RL_iteration_count = 0;
    last_RL_final_ERR = std::numeric_limits<double>::infinity();
    last_RL_converged = false;
    last_RL_max_abs_delta_theta = 0.0;
    last_RL_delta_theta_1 = 0.0;
    last_RL_delta_theta_5 = 0.0;
    last_RL_adaptive_lambda = 0.0;
    last_RL_max_iteration_delta_norm = 0.0;
    last_RL_continuity_cost = 0.0;
    last_RL_pose_error = std::numeric_limits<double>::infinity();
    last_RL_joint_distance_from_init = 0.0;
    last_RL_roll_pair_sum = 0.0;
    last_RL_roll_pair_cost = 0.0;
    last_RL_ldlt_fallback_used = false;
    last_RL_convergence_fallback_used = false;
#endif

#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
#ifdef STEP_DEBUG_IK_CONTINUITY_REGULARIZATION
#ifdef STEP_DEBUG_IK_CONTINUITY_WEIGHT
    constexpr double continuity_weight =
        static_cast<double>(STEP_DEBUG_IK_CONTINUITY_WEIGHT);
#else
    constexpr double continuity_weight = 0.1;
#endif
#endif
#ifdef STEP_DEBUG_IK_ROLL_PAIR_COUPLING
#ifdef STEP_DEBUG_IK_ROLL_PAIR_WEIGHT
    constexpr double roll_pair_weight =
        static_cast<double>(STEP_DEBUG_IK_ROLL_PAIR_WEIGHT);
#elif defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION)
    constexpr double roll_pair_weight = continuity_weight;
#else
    constexpr double roll_pair_weight = 0.1;
#endif
    constexpr double roll_pair_coefficient = 0.70710678118654752440;
#endif
#ifdef STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION
#ifdef STEP_DEBUG_IK_ROLL_AMPLITUDE_WEIGHT
    constexpr double roll_amplitude_weight =
        static_cast<double>(STEP_DEBUG_IK_ROLL_AMPLITUDE_WEIGHT);
#else
    constexpr double roll_amplitude_weight = 0.5;
#endif
#endif
#ifdef STEP_DEBUG_IK_DAMPING_WEIGHT
    constexpr double damping_weight =
        static_cast<double>(STEP_DEBUG_IK_DAMPING_WEIGHT);
#else
    constexpr double damping_weight = 0.01;
#endif
#ifdef STEP_DEBUG_IK_POSITION_WEIGHT
    constexpr double position_weight =
        static_cast<double>(STEP_DEBUG_IK_POSITION_WEIGHT);
#else
    constexpr double position_weight = 1.0;
#endif
#ifdef STEP_DEBUG_IK_ORIENTATION_WEIGHT
    constexpr double orientation_weight =
        static_cast<double>(STEP_DEBUG_IK_ORIENTATION_WEIGHT);
#else
    constexpr double orientation_weight = 1.0;
#endif
    Eigen::VectorXd best_theta = init_theta;
    double best_pose_error = std::numeric_limits<double>::infinity();
#endif

    for (iter = 0; iter < 100; ++iter){
        old_Q = th;
        BRP_RL_FK(th, link, PR);
        F = target_PR - PR; // Error_vector
        ERR = F.norm();

#ifdef STEP_DEBUG_SIMULATION
        last_RL_iteration_count = iter + 1;
        last_RL_final_ERR = ERR;
#endif

#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
        if (std::isfinite(ERR) && ERR < best_pose_error) {
            best_pose_error = ERR;
            best_theta = th;
#ifdef STEP_DEBUG_SIMULATION
            const double roll_1_distance = th(1) - init_theta(1);
            const double roll_5_distance = th(5) - init_theta(5);
            const double roll_joint_distance = std::sqrt(
                roll_1_distance * roll_1_distance
                + roll_5_distance * roll_5_distance
            );
            last_RL_pose_error = ERR;
            last_RL_joint_distance_from_init = roll_joint_distance;
#ifdef STEP_DEBUG_IK_CONTINUITY_REGULARIZATION
            last_RL_continuity_cost = continuity_weight * continuity_weight
                * roll_joint_distance * roll_joint_distance;
#endif
#endif
        }
#endif

        if (ERR < 0.0001){
#ifdef STEP_DEBUG_IK_SMALL_ROLL_BRANCH_TRACKING
            IK_theta = selectSmallRollBranch(th, init_theta, ERR).theta;
#else
            IK_theta = th;
#endif
#ifdef STEP_DEBUG_IK_CONVERGENCE_GUARD
            constexpr double two_pi = 6.28318530717958647692;
            for (int joint_index = 0; joint_index < dof; ++joint_index) {
                IK_theta(joint_index) = init_theta(joint_index)
                    + std::remainder(
                        IK_theta(joint_index) - init_theta(joint_index),
                        two_pi
                    );
            }
#endif
#ifdef STEP_DEBUG_SIMULATION
            last_RL_converged = true;
#endif
#if defined(STEP_DEBUG_SIMULATION) \
    && defined(STEP_DEBUG_IK_ROLL_PAIR_COUPLING) \
    && (defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
        || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION))
            last_RL_roll_pair_sum = IK_theta(1) + IK_theta(5);
            const double normalized_roll_pair_sum =
                roll_pair_coefficient * last_RL_roll_pair_sum;
            last_RL_roll_pair_cost = roll_pair_weight * roll_pair_weight
                * normalized_roll_pair_sum * normalized_roll_pair_sum;
#endif
            break;
        }
        else if (iter == 99){
#ifdef STEP_DEBUG_IK_CONVERGENCE_GUARD
            IK_theta = init_theta;
#ifdef STEP_DEBUG_SIMULATION
            last_RL_convergence_fallback_used = true;
#endif
#elif defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
            IK_theta = best_theta;
#else
            IK_theta = init_theta;
#endif
#if defined(STEP_DEBUG_SIMULATION) \
    && defined(STEP_DEBUG_IK_ROLL_PAIR_COUPLING) \
    && (defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
        || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION))
            last_RL_roll_pair_sum = IK_theta(1) + IK_theta(5);
            const double normalized_roll_pair_sum =
                roll_pair_coefficient * last_RL_roll_pair_sum;
            last_RL_roll_pair_cost = roll_pair_weight * roll_pair_weight
                * normalized_roll_pair_sum * normalized_roll_pair_sum;
#endif
            break;
        }

        old_PR = PR;

        // Numerical Jacobian 
        for (i = 0; i < dof; ++i){
            New_Q4J = old_Q;
            New_Q4J(i) += del_Q;
            BRP_RL_FK(New_Q4J, link, PR);
            New_PR.col(i) = PR;
        }

        for (i = 0; i < dof; ++i)
            for (j = 0; j < dof; ++j)
                J(i, j) = (New_PR(i, j) - old_PR(i)) / del_Q;

#if defined(STEP_DEBUG_SIMULATION) || defined(STEP_DEBUG_ADAPTIVE_DAMPED_IK)
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
        const auto singular_values = svd.singularValues();
        const double max_singular_value = singular_values.maxCoeff();
        const double min_singular_value = singular_values.minCoeff();
#endif
#ifdef STEP_DEBUG_SIMULATION
        constexpr double singular_value_epsilon = 1.0e-12;
        last_RL_min_singular_value = min_singular_value;
        if (!std::isfinite(min_singular_value)
            || min_singular_value <= singular_value_epsilon) {
            last_RL_condition_number = std::numeric_limits<double>::infinity();
        } else {
            last_RL_condition_number = max_singular_value / min_singular_value;
        }
        last_RL_max_abs_delta_theta = 0.0;
#endif

        // Keep the original inverse by default; enable debug candidates explicitly.
#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
        Eigen::MatrixXd pose_weight_matrix = Eigen::MatrixXd::Zero(dof, dof);
        Eigen::MatrixXd roll_selection = Eigen::MatrixXd::Zero(dof, dof);
        for (int pose_index = 0; pose_index < 3; ++pose_index) {
            pose_weight_matrix(pose_index, pose_index) = position_weight;
        }
        for (int pose_index = 3; pose_index < dof; ++pose_index) {
            pose_weight_matrix(pose_index, pose_index) = orientation_weight;
        }
        roll_selection(1, 1) = 1.0;
        roll_selection(5, 5) = 1.0;

        const Eigen::MatrixXd weighted_jacobian = pose_weight_matrix * J;
        const Eigen::VectorXd weighted_error = pose_weight_matrix * F;
        Eigen::MatrixXd regularized_system =
            weighted_jacobian.transpose() * weighted_jacobian
            + damping_weight * damping_weight
                * Eigen::MatrixXd::Identity(dof, dof);
        Eigen::VectorXd regularized_rhs =
            weighted_jacobian.transpose() * weighted_error;

#ifdef STEP_DEBUG_IK_CONTINUITY_REGULARIZATION
        const double continuity_weight_squared =
            continuity_weight * continuity_weight;
        regularized_system += continuity_weight_squared * roll_selection;
        regularized_rhs -= continuity_weight_squared * roll_selection
            * (th - init_theta);

#endif

#ifdef STEP_DEBUG_IK_ROLL_PAIR_COUPLING
        // Penalize the normalized post-update sum (q1 + delta1 + q5 + delta5).
        Eigen::VectorXd roll_pair_vector = Eigen::VectorXd::Zero(dof);
        roll_pair_vector(1) = roll_pair_coefficient;
        roll_pair_vector(5) = roll_pair_coefficient;
        const double roll_pair_weight_squared =
            roll_pair_weight * roll_pair_weight;
        regularized_system += roll_pair_weight_squared
            * roll_pair_vector * roll_pair_vector.transpose();
        regularized_rhs -= roll_pair_weight_squared * roll_pair_vector
            * roll_pair_vector.dot(th);
#endif

#ifdef STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION
        const Eigen::VectorXd roll_amplitude_reference =
            Eigen::VectorXd::Zero(dof);
        const double roll_amplitude_weight_squared =
            roll_amplitude_weight * roll_amplitude_weight;
        regularized_system += roll_amplitude_weight_squared * roll_selection;
        regularized_rhs -= roll_amplitude_weight_squared * roll_selection
            * (th - roll_amplitude_reference);
#endif

        Eigen::LDLT<Eigen::MatrixXd> regularized_ldlt(regularized_system);
        Eigen::VectorXd regularized_delta_theta(dof);
        bool regularized_solve_succeeded =
            regularized_ldlt.info() == Eigen::Success;
        if (regularized_solve_succeeded) {
            regularized_delta_theta = regularized_ldlt.solve(regularized_rhs);
            regularized_solve_succeeded =
                regularized_ldlt.info() == Eigen::Success
                && regularized_delta_theta.allFinite();
        }
        if (!regularized_solve_succeeded) {
#ifdef STEP_DEBUG_SIMULATION
            last_RL_ldlt_fallback_used = true;
#endif
            regularized_delta_theta = J.inverse() * F;
        }
#elif defined(STEP_DEBUG_ADAPTIVE_DAMPED_IK)
        constexpr double adaptive_singular_value_threshold = 0.2;
        constexpr double adaptive_min_lambda = 1.0e-4;
        constexpr double adaptive_max_lambda = 0.05;
        double adaptive_lambda = adaptive_max_lambda;
        if (std::isfinite(min_singular_value)) {
            const double singularity_ratio = std::clamp(
                (adaptive_singular_value_threshold - min_singular_value)
                    / adaptive_singular_value_threshold,
                0.0,
                1.0
            );
            adaptive_lambda = adaptive_min_lambda
                + (adaptive_max_lambda - adaptive_min_lambda)
                    * singularity_ratio * singularity_ratio;
        }
        const Eigen::MatrixXd damped_system = J * J.transpose()
            + adaptive_lambda * adaptive_lambda
                * Eigen::MatrixXd::Identity(dof, dof);
        Inv_J = J.transpose() * damped_system.ldlt().solve(
            Eigen::MatrixXd::Identity(dof, dof)
        );
#ifdef STEP_DEBUG_SIMULATION
        last_RL_adaptive_lambda = adaptive_lambda;
#endif
#elif defined(STEP_DEBUG_DAMPED_IK)
        constexpr double damped_ik_lambda = 0.01;
        const Eigen::MatrixXd damping =
            damped_ik_lambda * damped_ik_lambda
            * Eigen::MatrixXd::Identity(dof, dof);
        Inv_J = J.transpose() * (J * J.transpose() + damping).inverse();
#else
        Inv_J = J.inverse();
#endif

        // Joint Angle Update
#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ITERATION_TRUST_REGION) \
    || defined(STEP_DEBUG_IK_SMALL_ROLL_BRANCH_TRACKING)
#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
        Eigen::VectorXd delta_theta = regularized_delta_theta;
#else
        Eigen::VectorXd delta_theta = Inv_J * F;
#endif
        const double raw_delta_norm = delta_theta.norm();
#ifdef STEP_DEBUG_SIMULATION
        last_RL_max_iteration_delta_norm = std::max(
            last_RL_max_iteration_delta_norm,
            raw_delta_norm
        );
#endif
#ifdef STEP_DEBUG_IK_ITERATION_TRUST_REGION
#ifdef STEP_DEBUG_IK_TRUST_REGION_NORM
        constexpr double max_step_norm =
            static_cast<double>(STEP_DEBUG_IK_TRUST_REGION_NORM);
#else
        constexpr double max_step_norm = 0.2;
#endif
        // This is an iteration trust region, not a frame-to-frame continuity limit.
        // A very small norm can increase non-convergence and trigger init_theta fallback.
        if (raw_delta_norm > max_step_norm && raw_delta_norm > 0.0) {
            delta_theta *= max_step_norm / raw_delta_norm;
        }
#endif
#ifdef STEP_DEBUG_SIMULATION
        last_RL_max_abs_delta_theta = delta_theta.cwiseAbs().maxCoeff();
        last_RL_delta_theta_1 = delta_theta(1);
        last_RL_delta_theta_5 = delta_theta(5);
#endif
#ifdef STEP_DEBUG_IK_SMALL_ROLL_BRANCH_TRACKING
        const SmallRollBranchCandidate selected_step = selectBranchTrackingStep(
            old_Q, delta_theta, init_theta, target_PR, link, BRP_RL_FK
        );
        delta_theta = selected_step.theta - old_Q;
#endif
        th = old_Q + delta_theta;
#else
#ifdef STEP_DEBUG_SIMULATION
        double iteration_delta_squared_norm = 0.0;
#endif
        for (k = 0; k < dof; ++k){
            double sum = 0.0;
            for (j = 0; j < dof; ++j)
                sum += Inv_J(k, j) * F(j);
#ifdef STEP_DEBUG_SIMULATION
            last_RL_max_abs_delta_theta =
                std::max(last_RL_max_abs_delta_theta, std::abs(sum));
            if (k == 1) last_RL_delta_theta_1 = sum;
            if (k == 5) last_RL_delta_theta_5 = sum;
            iteration_delta_squared_norm += sum * sum;
#endif
            th(k) = old_Q(k) + sum;
        }
#ifdef STEP_DEBUG_SIMULATION
        last_RL_max_iteration_delta_norm = std::max(
            last_RL_max_iteration_delta_norm,
            std::sqrt(iteration_delta_squared_norm)
        );
#endif
#endif

        // Knee, Joint3 음수 방지 
        if (th(3) < 0) th(3) = -th(3);
    }
}

void BRP_LL_FK(const VectorXd& th, const VectorXd& link, VectorXd& PR)
{
    double t1 = th(0), t2 = th(1), t3 = th(2), t4 = th(3), t5 = th(4), t6 = th(5);
    double L0 = link(0), L1 = link(1), L2 = link(2), L3 = link(3), L4 = link(4), L5 = link(5), L6 = link(6);

    double c1 = cos(t1), c2 = cos(t2), c3 = cos(t3), c4 = cos(t4), c5 = cos(t5), c6 = cos(t6);
    double s1 = sin(t1), s2 = sin(t2), s3 = sin(t3), s4 = sin(t4), s5 = sin(t5), s6 = sin(t6);

    // x, y, z position setup
    PR(0) = L5 * s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))- L4 * s4 * (c1 * c3 - s1 * s2 * s3)
          - L5 * c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2))- L3 * c1 * s3 - L4 * c4 * (c1 * s3 + c3 * s1 * s2)- L2 * s1 * s2
          - L6 * c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)))
          - L3 * c3 * s1 * s2 - L6 * c2 * s1 * s6;

    PR(1) = L0 - L4 * c4 * (s1 * s3 - c1 * c3 * s2) - L4 * s4 * (c3 * s1 + c1 * s2 * s3) - L5 * c5 * (s4 * (c3 * s1 + c1 * s2 * s3)
          + c4 * (s1 * s3 - c1 * c3 * s2)) + L2 * c1 * s2 + L5 * s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))
          - L3 * s1 * s3 - L6 * c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2)
          - c4 * (c3 * s1 + c1 * s2 * s3))) + L3 * c1 * c3 * s2 + L6 * c1 * c2 * s6;

    PR(2) = L6 * s2 * s6 - L2 * c2 - L6 * c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3))
          - L3 * c2 * c3 - L1 - L5 * c5 * (c2 * c3 * c4 - c2 * s3 * s4) + L5 * s5 * (c2 * c3 * s4 + c2 * c4 * s3) - L4 * c2 * c3 * c4 + L4 * c2 * s3 * s4;

    // Orientation Calculation (n, o, a vector)
    double nx = -c5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3)) - s5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2));
    double ny = -c5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3)) - s5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2));
    double nz = -c5 * (c2 * c3 * s4 + c2 * c4 * s3) - s5 * (c2 * c3 * c4 - c2 * s3 * s4);

    double ox = s6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) - c2 * c6 * s1;
    double oy = s6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) + c1 * c2 * c6;
    double oz = c6 * s2 + s6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3));

    double ax = c6 * (c5 * (s4 * (c1 * c3 - s1 * s2 * s3) + c4 * (c1 * s3 + c3 * s1 * s2)) - s5 * (s4 * (c1 * s3 + c3 * s1 * s2) - c4 * (c1 * c3 - s1 * s2 * s3))) + c2 * s1 * s6;
    double ay = c6 * (c5 * (s4 * (c3 * s1 + c1 * s2 * s3) + c4 * (s1 * s3 - c1 * c3 * s2)) - s5 * (s4 * (s1 * s3 - c1 * c3 * s2) - c4 * (c3 * s1 + c1 * s2 * s3))) - c1 * c2 * s6;
    double az = c6 * (c5 * (c2 * c3 * c4 - c2 * s3 * s4) - s5 * (c2 * c3 * s4 + c2 * c4 * s3)) - s2 * s6;

    // RPY 오일러 각
    PR(5) = atan2(ny, nx);                                                                 // Roll (z축)
    PR(4) = atan2(-nz, cos(PR(5)) * nx + sin(PR(5)) * ny);                                 // Pitch (y축)
    PR(3) = atan2(sin(PR(5)) * ax - cos(PR(5)) * ay, -sin(PR(5)) * ox + cos(PR(5)) * oy);  // Yaw (x축)
}


void BRP_LL_IK(const VectorXd& target_PR, const VectorXd& init_theta, const VectorXd& link, VectorXd& IK_theta)
{    
    const int dof = 6;
    Eigen::VectorXd th = init_theta;
    Eigen::VectorXd PR(dof), old_PR(dof), F(dof), old_Q(dof);
    Eigen::MatrixXd J(dof, dof), Inv_J(dof, dof), New_PR(dof, dof);
    Eigen::VectorXd New_Q4J(dof);
    double del_Q = 0.0001, ERR = 0.0;
    int iter, i, j, k;
    double sum = 0.0;

#ifdef STEP_DEBUG_SIMULATION
    last_LL_condition_number = std::numeric_limits<double>::infinity();
    last_LL_min_singular_value = 0.0;
    last_LL_iteration_count = 0;
    last_LL_final_ERR = std::numeric_limits<double>::infinity();
    last_LL_converged = false;
    last_LL_max_abs_delta_theta = 0.0;
    last_LL_delta_theta_1 = 0.0;
    last_LL_delta_theta_5 = 0.0;
    last_LL_adaptive_lambda = 0.0;
    last_LL_max_iteration_delta_norm = 0.0;
    last_LL_continuity_cost = 0.0;
    last_LL_pose_error = std::numeric_limits<double>::infinity();
    last_LL_joint_distance_from_init = 0.0;
    last_LL_roll_pair_sum = 0.0;
    last_LL_roll_pair_cost = 0.0;
    last_LL_ldlt_fallback_used = false;
    last_LL_convergence_fallback_used = false;
#endif

#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
#ifdef STEP_DEBUG_IK_CONTINUITY_REGULARIZATION
#ifdef STEP_DEBUG_IK_CONTINUITY_WEIGHT
    constexpr double continuity_weight =
        static_cast<double>(STEP_DEBUG_IK_CONTINUITY_WEIGHT);
#else
    constexpr double continuity_weight = 0.1;
#endif
#endif
#ifdef STEP_DEBUG_IK_ROLL_PAIR_COUPLING
#ifdef STEP_DEBUG_IK_ROLL_PAIR_WEIGHT
    constexpr double roll_pair_weight =
        static_cast<double>(STEP_DEBUG_IK_ROLL_PAIR_WEIGHT);
#elif defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION)
    constexpr double roll_pair_weight = continuity_weight;
#else
    constexpr double roll_pair_weight = 0.1;
#endif
    constexpr double roll_pair_coefficient = 0.70710678118654752440;
#endif
#ifdef STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION
#ifdef STEP_DEBUG_IK_ROLL_AMPLITUDE_WEIGHT
    constexpr double roll_amplitude_weight =
        static_cast<double>(STEP_DEBUG_IK_ROLL_AMPLITUDE_WEIGHT);
#else
    constexpr double roll_amplitude_weight = 0.5;
#endif
#endif
#ifdef STEP_DEBUG_IK_DAMPING_WEIGHT
    constexpr double damping_weight =
        static_cast<double>(STEP_DEBUG_IK_DAMPING_WEIGHT);
#else
    constexpr double damping_weight = 0.01;
#endif
#ifdef STEP_DEBUG_IK_POSITION_WEIGHT
    constexpr double position_weight =
        static_cast<double>(STEP_DEBUG_IK_POSITION_WEIGHT);
#else
    constexpr double position_weight = 1.0;
#endif
#ifdef STEP_DEBUG_IK_ORIENTATION_WEIGHT
    constexpr double orientation_weight =
        static_cast<double>(STEP_DEBUG_IK_ORIENTATION_WEIGHT);
#else
    constexpr double orientation_weight = 1.0;
#endif
    Eigen::VectorXd best_theta = init_theta;
    double best_pose_error = std::numeric_limits<double>::infinity();
#endif

    for (iter = 0; iter < 100; ++iter){
        old_Q = th;
        BRP_LL_FK(th, link, PR);
        F = target_PR - PR; // Error_vector
        ERR = F.norm();

#ifdef STEP_DEBUG_SIMULATION
        last_LL_iteration_count = iter + 1;
        last_LL_final_ERR = ERR;
#endif

#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
        if (std::isfinite(ERR) && ERR < best_pose_error) {
            best_pose_error = ERR;
            best_theta = th;
#ifdef STEP_DEBUG_SIMULATION
            const double roll_1_distance = th(1) - init_theta(1);
            const double roll_5_distance = th(5) - init_theta(5);
            const double roll_joint_distance = std::sqrt(
                roll_1_distance * roll_1_distance
                + roll_5_distance * roll_5_distance
            );
            last_LL_pose_error = ERR;
            last_LL_joint_distance_from_init = roll_joint_distance;
#ifdef STEP_DEBUG_IK_CONTINUITY_REGULARIZATION
            last_LL_continuity_cost = continuity_weight * continuity_weight
                * roll_joint_distance * roll_joint_distance;
#endif
#endif
        }
#endif

        if (ERR < 0.0001){
#ifdef STEP_DEBUG_IK_SMALL_ROLL_BRANCH_TRACKING
            IK_theta = selectSmallRollBranch(th, init_theta, ERR).theta;
#else
            IK_theta = th;
#endif
#ifdef STEP_DEBUG_IK_CONVERGENCE_GUARD
            constexpr double two_pi = 6.28318530717958647692;
            for (int joint_index = 0; joint_index < dof; ++joint_index) {
                IK_theta(joint_index) = init_theta(joint_index)
                    + std::remainder(
                        IK_theta(joint_index) - init_theta(joint_index),
                        two_pi
                    );
            }
#endif
#ifdef STEP_DEBUG_SIMULATION
            last_LL_converged = true;
#endif
#if defined(STEP_DEBUG_SIMULATION) \
    && defined(STEP_DEBUG_IK_ROLL_PAIR_COUPLING) \
    && (defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
        || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION))
            last_LL_roll_pair_sum = IK_theta(1) + IK_theta(5);
            const double normalized_roll_pair_sum =
                roll_pair_coefficient * last_LL_roll_pair_sum;
            last_LL_roll_pair_cost = roll_pair_weight * roll_pair_weight
                * normalized_roll_pair_sum * normalized_roll_pair_sum;
#endif
            break;
        }
        else if (iter == 99){
#ifdef STEP_DEBUG_IK_CONVERGENCE_GUARD
            IK_theta = init_theta;
#ifdef STEP_DEBUG_SIMULATION
            last_LL_convergence_fallback_used = true;
#endif
#elif defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
            IK_theta = best_theta;
#else
            IK_theta = init_theta;
#endif
#if defined(STEP_DEBUG_SIMULATION) \
    && defined(STEP_DEBUG_IK_ROLL_PAIR_COUPLING) \
    && (defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
        || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION))
            last_LL_roll_pair_sum = IK_theta(1) + IK_theta(5);
            const double normalized_roll_pair_sum =
                roll_pair_coefficient * last_LL_roll_pair_sum;
            last_LL_roll_pair_cost = roll_pair_weight * roll_pair_weight
                * normalized_roll_pair_sum * normalized_roll_pair_sum;
#endif
            break;
        }

        old_PR = PR;

        // Numerical Jacobian 
        for (i = 0; i < dof; ++i){
            New_Q4J = old_Q;
            New_Q4J(i) += del_Q;
            BRP_LL_FK(New_Q4J, link, PR);
            New_PR.col(i) = PR;
        }

        for (i = 0; i < dof; ++i)
            for (j = 0; j < dof; ++j)
                J(i, j) = (New_PR(i, j) - old_PR(i)) / del_Q;

#if defined(STEP_DEBUG_SIMULATION) || defined(STEP_DEBUG_ADAPTIVE_DAMPED_IK)
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
        const auto singular_values = svd.singularValues();
        const double max_singular_value = singular_values.maxCoeff();
        const double min_singular_value = singular_values.minCoeff();
#endif
#ifdef STEP_DEBUG_SIMULATION
        constexpr double singular_value_epsilon = 1.0e-12;
        last_LL_min_singular_value = min_singular_value;
        if (!std::isfinite(min_singular_value)
            || min_singular_value <= singular_value_epsilon) {
            last_LL_condition_number = std::numeric_limits<double>::infinity();
        } else {
            last_LL_condition_number = max_singular_value / min_singular_value;
        }
        last_LL_max_abs_delta_theta = 0.0;
#endif

        // Keep the original inverse by default; enable debug candidates explicitly.
#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
        Eigen::MatrixXd pose_weight_matrix = Eigen::MatrixXd::Zero(dof, dof);
        Eigen::MatrixXd roll_selection = Eigen::MatrixXd::Zero(dof, dof);
        for (int pose_index = 0; pose_index < 3; ++pose_index) {
            pose_weight_matrix(pose_index, pose_index) = position_weight;
        }
        for (int pose_index = 3; pose_index < dof; ++pose_index) {
            pose_weight_matrix(pose_index, pose_index) = orientation_weight;
        }
        roll_selection(1, 1) = 1.0;
        roll_selection(5, 5) = 1.0;

        const Eigen::MatrixXd weighted_jacobian = pose_weight_matrix * J;
        const Eigen::VectorXd weighted_error = pose_weight_matrix * F;
        Eigen::MatrixXd regularized_system =
            weighted_jacobian.transpose() * weighted_jacobian
            + damping_weight * damping_weight
                * Eigen::MatrixXd::Identity(dof, dof);
        Eigen::VectorXd regularized_rhs =
            weighted_jacobian.transpose() * weighted_error;

#ifdef STEP_DEBUG_IK_CONTINUITY_REGULARIZATION
        const double continuity_weight_squared =
            continuity_weight * continuity_weight;
        regularized_system += continuity_weight_squared * roll_selection;
        regularized_rhs -= continuity_weight_squared * roll_selection
            * (th - init_theta);

#endif

#ifdef STEP_DEBUG_IK_ROLL_PAIR_COUPLING
        // Penalize the normalized post-update sum (q1 + delta1 + q5 + delta5).
        Eigen::VectorXd roll_pair_vector = Eigen::VectorXd::Zero(dof);
        roll_pair_vector(1) = roll_pair_coefficient;
        roll_pair_vector(5) = roll_pair_coefficient;
        const double roll_pair_weight_squared =
            roll_pair_weight * roll_pair_weight;
        regularized_system += roll_pair_weight_squared
            * roll_pair_vector * roll_pair_vector.transpose();
        regularized_rhs -= roll_pair_weight_squared * roll_pair_vector
            * roll_pair_vector.dot(th);
#endif

#ifdef STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION
        const Eigen::VectorXd roll_amplitude_reference =
            Eigen::VectorXd::Zero(dof);
        const double roll_amplitude_weight_squared =
            roll_amplitude_weight * roll_amplitude_weight;
        regularized_system += roll_amplitude_weight_squared * roll_selection;
        regularized_rhs -= roll_amplitude_weight_squared * roll_selection
            * (th - roll_amplitude_reference);
#endif

        Eigen::LDLT<Eigen::MatrixXd> regularized_ldlt(regularized_system);
        Eigen::VectorXd regularized_delta_theta(dof);
        bool regularized_solve_succeeded =
            regularized_ldlt.info() == Eigen::Success;
        if (regularized_solve_succeeded) {
            regularized_delta_theta = regularized_ldlt.solve(regularized_rhs);
            regularized_solve_succeeded =
                regularized_ldlt.info() == Eigen::Success
                && regularized_delta_theta.allFinite();
        }
        if (!regularized_solve_succeeded) {
#ifdef STEP_DEBUG_SIMULATION
            last_LL_ldlt_fallback_used = true;
#endif
            regularized_delta_theta = J.inverse() * F;
        }
#elif defined(STEP_DEBUG_ADAPTIVE_DAMPED_IK)
        constexpr double adaptive_singular_value_threshold = 0.2;
        constexpr double adaptive_min_lambda = 1.0e-4;
        constexpr double adaptive_max_lambda = 0.05;
        double adaptive_lambda = adaptive_max_lambda;
        if (std::isfinite(min_singular_value)) {
            const double singularity_ratio = std::clamp(
                (adaptive_singular_value_threshold - min_singular_value)
                    / adaptive_singular_value_threshold,
                0.0,
                1.0
            );
            adaptive_lambda = adaptive_min_lambda
                + (adaptive_max_lambda - adaptive_min_lambda)
                    * singularity_ratio * singularity_ratio;
        }
        const Eigen::MatrixXd damped_system = J * J.transpose()
            + adaptive_lambda * adaptive_lambda
                * Eigen::MatrixXd::Identity(dof, dof);
        Inv_J = J.transpose() * damped_system.ldlt().solve(
            Eigen::MatrixXd::Identity(dof, dof)
        );
#ifdef STEP_DEBUG_SIMULATION
        last_LL_adaptive_lambda = adaptive_lambda;
#endif
#elif defined(STEP_DEBUG_DAMPED_IK)
        constexpr double damped_ik_lambda = 0.01;
        const Eigen::MatrixXd damping =
            damped_ik_lambda * damped_ik_lambda
            * Eigen::MatrixXd::Identity(dof, dof);
        Inv_J = J.transpose() * (J * J.transpose() + damping).inverse();
#else
        Inv_J = J.inverse();
#endif

        // Joint Angle Update
#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ITERATION_TRUST_REGION) \
    || defined(STEP_DEBUG_IK_SMALL_ROLL_BRANCH_TRACKING)
#if defined(STEP_DEBUG_IK_CONTINUITY_REGULARIZATION) \
    || defined(STEP_DEBUG_IK_ROLL_AMPLITUDE_REGULARIZATION)
        Eigen::VectorXd delta_theta = regularized_delta_theta;
#else
        Eigen::VectorXd delta_theta = Inv_J * F;
#endif
        const double raw_delta_norm = delta_theta.norm();
#ifdef STEP_DEBUG_SIMULATION
        last_LL_max_iteration_delta_norm = std::max(
            last_LL_max_iteration_delta_norm,
            raw_delta_norm
        );
#endif
#ifdef STEP_DEBUG_IK_ITERATION_TRUST_REGION
#ifdef STEP_DEBUG_IK_TRUST_REGION_NORM
        constexpr double max_step_norm =
            static_cast<double>(STEP_DEBUG_IK_TRUST_REGION_NORM);
#else
        constexpr double max_step_norm = 0.2;
#endif
        // This is an iteration trust region, not a frame-to-frame continuity limit.
        // A very small norm can increase non-convergence and trigger init_theta fallback.
        if (raw_delta_norm > max_step_norm && raw_delta_norm > 0.0) {
            delta_theta *= max_step_norm / raw_delta_norm;
        }
#endif
#ifdef STEP_DEBUG_SIMULATION
        last_LL_max_abs_delta_theta = delta_theta.cwiseAbs().maxCoeff();
        last_LL_delta_theta_1 = delta_theta(1);
        last_LL_delta_theta_5 = delta_theta(5);
#endif
#ifdef STEP_DEBUG_IK_SMALL_ROLL_BRANCH_TRACKING
        const SmallRollBranchCandidate selected_step = selectBranchTrackingStep(
            old_Q, delta_theta, init_theta, target_PR, link, BRP_LL_FK
        );
        delta_theta = selected_step.theta - old_Q;
#endif
        th = old_Q + delta_theta;
#else
#ifdef STEP_DEBUG_SIMULATION
        double iteration_delta_squared_norm = 0.0;
#endif
        for (k = 0; k < dof; ++k){
            double sum = 0.0;
            for (j = 0; j < dof; ++j)
                sum += Inv_J(k, j) * F(j);
#ifdef STEP_DEBUG_SIMULATION
            last_LL_max_abs_delta_theta =
                std::max(last_LL_max_abs_delta_theta, std::abs(sum));
            if (k == 1) last_LL_delta_theta_1 = sum;
            if (k == 5) last_LL_delta_theta_5 = sum;
            iteration_delta_squared_norm += sum * sum;
#endif
            th(k) = old_Q(k) + sum;
        }
#ifdef STEP_DEBUG_SIMULATION
        last_LL_max_iteration_delta_norm = std::max(
            last_LL_max_iteration_delta_norm,
            std::sqrt(iteration_delta_squared_norm)
        );
#endif
#endif

        // Knee, Joint3 음수 방지 
        if (th(3) < 0) th(3) = -th(3);
    }
}

}
