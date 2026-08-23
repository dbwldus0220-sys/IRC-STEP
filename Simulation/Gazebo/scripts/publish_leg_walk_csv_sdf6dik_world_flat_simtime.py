#!/usr/bin/env python3
"""Replay SDF-6D-IK walking with opt-in right-sole world-flat feedback."""

import argparse
import csv
import math
import threading
from pathlib import Path

import numpy as np
from google.protobuf import symbol_database
from gz.msgs10 import (
    header_pb2,
    odometry_pb2,
    pose_pb2,
    quaternion_pb2,
    time_pb2,
    twist_pb2,
    vector3d_pb2,
)
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node
from scipy.spatial.transform import Rotation

from gazebo_leg_transport import LEG_JOINT_TOPICS, GazeboDoublePublishers
from generate_walk_csv_sdf_6d_ik import (
    CANDIDATE_B,
    DEFAULT_SDF,
    load_leg_chains,
    pose_errors,
    solve_leg_frame,
)


DEFAULT_DT = 0.01
DEFAULT_HOLD_BEFORE = 1.0
DEFAULT_REPLAY_END = 1.44
DEFAULT_HOLD_AFTER = 1.0
DEFAULT_ODOM_TOPIC = "/step/base_odometry"
DEFAULT_RIGHT_CONTACT_TOPIC = "/step/right_sole_contacts"
DEFAULT_LEG_JOINT_STATE_TOPIC = "/step/leg_joint_states"
RIGHT_SOLE_COLLISION_NAME = "right_sole_box_collision"
VELOCITY_WARNING_RAD_S = 4.71238898
ORIENTATION_CACHE_EPS_RAD = 1e-6
CORRECTION_RELEASE_TOLERANCE_M = 1e-9

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_CSV = (
    REPO_ROOT / "Dynamics" / "walk_forward_gazebo_sdf6dik_candidateB_ref.csv"
)
DEFAULT_PLANNER_CSV = REPO_ROOT / "Dynamics" / (
    "walk_forward_debug_slow_y_scale002_continuation5_baseline_jumpguard_"
    "th005_lpf_a050_pitchlpf_a080_long.csv"
)
DEFAULT_COMMAND_LOG = (
    SCRIPT_DIR.parent / "logs" / "walk_sdf6dik_world_flat_commands.csv"
)

_SYMBOLS = symbol_database.Default()
for _message_class in (
    time_pb2.Time,
    header_pb2.Header,
    header_pb2.Header.Map,
    vector3d_pb2.Vector3d,
    quaternion_pb2.Quaternion,
    pose_pb2.Pose,
    twist_pb2.Twist,
    odometry_pb2.Odometry,
):
    _SYMBOLS.RegisterMessage(_message_class)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--hold-before", type=float, default=DEFAULT_HOLD_BEFORE)
    parser.add_argument(
        "--replay-end", type=float, default=DEFAULT_REPLAY_END,
        help="trajectory time to replay through, in simulation seconds",
    )
    parser.add_argument("--hold-after", type=float, default=DEFAULT_HOLD_AFTER)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--command-log", type=Path, default=DEFAULT_COMMAND_LOG)
    parser.add_argument(
        "--world-flat-leg", choices=("right", "left", "both"), required=True,
        help="leg or legs receiving world-flat feedback",
    )
    parser.add_argument(
        "--world-flat-start", type=float, required=True,
        help="inclusive trajectory-time start of world-flat feedback",
    )
    parser.add_argument(
        "--world-flat-end", type=float, required=True,
        help="inclusive trajectory-time end of world-flat feedback",
    )
    parser.add_argument("--world-flat-ramp-in", type=float, required=True)
    parser.add_argument("--world-flat-ramp-out", type=float, required=True)
    parser.add_argument(
        "--swing-world-z-leg", choices=("right",),
        help="swing leg receiving world-relative Z compensation",
    )
    parser.add_argument("--swing-world-z-start", type=float)
    parser.add_argument("--swing-world-z-end", type=float)
    parser.add_argument("--swing-world-z-ramp-in", type=float)
    parser.add_argument("--swing-world-z-ramp-out", type=float)
    parser.add_argument(
        "--swing-world-z-max-correction-release-speed-mps", type=float,
        help="optional maximum swing world-Z correction release speed",
    )
    parser.add_argument(
        "--max-feedback-joint-speed-deg-s", type=float,
        help="optional per-leg feedback command rate limit",
    )
    parser.add_argument(
        "--right-swing-lateral-scale", type=float,
        help=(
            "optional scale in (0, 1] for RIGHT nominal base-frame X "
            "motion after its reference time"
        ),
    )
    parser.add_argument(
        "--right-swing-lateral-reference-time", type=float,
        help="trajectory time defining the unscaled RIGHT X reference",
    )
    parser.add_argument("--lateral-balance-planner-csv", type=Path)
    parser.add_argument("--lateral-balance-start", type=float)
    parser.add_argument("--lateral-balance-end", type=float)
    parser.add_argument("--lateral-balance-kp", type=float)
    parser.add_argument("--lateral-balance-kd", type=float)
    parser.add_argument("--lateral-balance-max-offset-m", type=float)
    parser.add_argument("--lateral-balance-ramp-in", type=float)
    parser.add_argument("--lateral-balance-ramp-out", type=float)
    parser.add_argument(
        "--lateral-balance-support-handoff", action="store_true",
    )
    parser.add_argument(
        "--lateral-balance-support-handoff-ramp-s", type=float, default=0.20,
    )
    parser.add_argument("--pitch-balance-start", type=float)
    parser.add_argument("--pitch-balance-end", type=float)
    parser.add_argument("--pitch-balance-kp", type=float)
    parser.add_argument("--pitch-balance-kd", type=float)
    parser.add_argument("--pitch-balance-max-angle-deg", type=float)
    parser.add_argument("--pitch-balance-ramp-in", type=float)
    parser.add_argument("--pitch-balance-ramp-out", type=float)
    parser.add_argument(
        "--pitch-balance-support-handoff", action="store_true",
        help="handoff pitch balance from LEFT to RIGHT after confirmed touchdown",
    )
    parser.add_argument(
        "--pitch-balance-support-handoff-ramp-s", type=float, default=0.08,
    )
    parser.add_argument(
        "--pitch-balance-right-support-sign", type=int,
        choices=(-1, 1), default=1,
    )
    parser.add_argument(
        "--pitch-balance-right-max-angle-deg", type=float, default=0.5,
    )
    parser.add_argument(
        "--pitch-balance-right-rate-limit-deg-s", type=float,
        help="optional RIGHT pitch-balance handoff angular rate limit",
    )
    parser.add_argument("--left-support-impact-anchor", action="store_true")
    parser.add_argument(
        "--left-support-impact-anchor-hold-s", type=float, default=0.10,
    )
    parser.add_argument(
        "--left-support-impact-anchor-release-s", type=float, default=0.20,
    )
    parser.add_argument(
        "--touchdown-support-hold", action="store_true",
        help="enable contact-latched right sole world-pose hold",
    )
    parser.add_argument(
        "--touchdown-right-contact-topic",
        default=DEFAULT_RIGHT_CONTACT_TOPIC,
    )
    parser.add_argument(
        "--touchdown-leg-joint-state-topic",
        default=DEFAULT_LEG_JOINT_STATE_TOPIC,
    )
    parser.add_argument(
        "--touchdown-right-fz-threshold", type=float, default=20.0,
    )
    parser.add_argument(
        "--touchdown-confirm-s", type=float, default=0.02,
    )
    parser.add_argument(
        "--touchdown-left-feedback-ramp-out", type=float, default=0.05,
    )
    parser.add_argument(
        "--touchdown-z-arrest", action="store_true",
        help="arrest downward RIGHT world-Z target at first contact",
    )
    parser.add_argument(
        "--touchdown-z-arrest-threshold-n", type=float, default=20.0,
    )
    parser.add_argument(
        "--touchdown-z-arrest-confirm-s", type=float, default=0.02,
    )
    parser.add_argument(
        "--touchdown-z-arrest-settle-depth-m", type=float, default=0.0,
        help="maximum downward RIGHT world-Z settle after confirmed touchdown",
    )
    parser.add_argument(
        "--touchdown-z-arrest-settle-speed-mps", type=float, default=0.0,
        help="downward RIGHT world-Z settle speed after confirmed touchdown",
    )
    parser.add_argument(
        "--touchdown-z-arrest-post-settle-release-s", type=float,
    )
    parser.add_argument(
        "--left-support-z-hold-until-right-confirmed", action="store_true",
        help="hold only LEFT sole world-Z until stable RIGHT touchdown",
    )
    parser.add_argument(
        "--left-support-z-release-s", type=float, default=0.08,
        help="smooth LEFT world-Z release duration after confirmation",
    )
    parser.add_argument(
        "--left-support-z-hold-through-replay", action="store_true",
        help="keep the latched LEFT sole world-Z through REPLAY",
    )
    parser.add_argument(
        "--fore-aft-sign-test-correction-deg", type=float,
        help=(
            "fixed RIGHT support-foot world-X orientation correction after "
            "stable touchdown (limited to +/-0.25 deg)"
        ),
    )
    parser.add_argument(
        "--fore-aft-sign-test-ramp-in-s", type=float, default=0.05,
        help="smoothstep ramp-in after stable RIGHT touchdown",
    )
    parser.add_argument("--right-touchdown-flatten-correction-deg", type=float)
    parser.add_argument("--right-touchdown-flatten-start", type=float)
    parser.add_argument(
        "--right-touchdown-flatten-ramp-in-s", type=float, default=0.05,
    )
    parser.add_argument(
        "--right-touchdown-flatten-ramp-out-s", type=float, default=0.08,
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    swing_values = (
        args.swing_world_z_leg,
        args.swing_world_z_start,
        args.swing_world_z_end,
        args.swing_world_z_ramp_in,
        args.swing_world_z_ramp_out,
    )
    if any(value is not None for value in swing_values) and not all(
        value is not None for value in swing_values
    ):
        parser.error(
            "--swing-world-z-leg, --swing-world-z-start, and "
            "--swing-world-z-end must be specified together"
        )
    numeric = [
        args.dt, args.hold_before, args.replay_end, args.hold_after,
        args.world_flat_start, args.world_flat_end,
        args.world_flat_ramp_in, args.world_flat_ramp_out,
    ]
    if args.swing_world_z_start is not None:
        numeric.extend((
            args.swing_world_z_start, args.swing_world_z_end,
            args.swing_world_z_ramp_in, args.swing_world_z_ramp_out,
        ))
    if args.max_feedback_joint_speed_deg_s is not None:
        numeric.append(args.max_feedback_joint_speed_deg_s)
    lateral_scale_values = (
        args.right_swing_lateral_scale,
        args.right_swing_lateral_reference_time,
    )
    if any(value is not None for value in lateral_scale_values) and not all(
        value is not None for value in lateral_scale_values
    ):
        parser.error(
            "--right-swing-lateral-scale and "
            "--right-swing-lateral-reference-time must be specified together"
        )
    if args.right_swing_lateral_scale is not None:
        if not all(math.isfinite(value) for value in lateral_scale_values):
            parser.error("RIGHT swing lateral arguments must be finite")
        if not 0.0 < args.right_swing_lateral_scale <= 1.0:
            parser.error("--right-swing-lateral-scale must be in (0, 1]")
        if args.right_swing_lateral_reference_time < 0.0:
            parser.error(
                "--right-swing-lateral-reference-time must be non-negative"
            )
        if args.dt <= 0.0:
            parser.error("--dt must be positive")
        reference_frame = round(
            args.right_swing_lateral_reference_time / args.dt
        )
        if not math.isclose(
            reference_frame * args.dt,
            args.right_swing_lateral_reference_time,
            abs_tol=1e-12,
        ):
            parser.error(
                "--right-swing-lateral-reference-time must align with --dt"
            )
        if args.right_swing_lateral_reference_time > args.replay_end:
            parser.error(
                "--right-swing-lateral-reference-time must not exceed "
                "--replay-end"
            )
    if args.swing_world_z_max_correction_release_speed_mps is not None:
        numeric.append(args.swing_world_z_max_correction_release_speed_mps)
    lateral_values = (
        args.lateral_balance_planner_csv,
        args.lateral_balance_start, args.lateral_balance_end,
        args.lateral_balance_kp, args.lateral_balance_kd,
        args.lateral_balance_max_offset_m,
        args.lateral_balance_ramp_in, args.lateral_balance_ramp_out,
    )
    if any(value is not None for value in lateral_values) and not all(
        value is not None for value in lateral_values
    ):
        parser.error("all --lateral-balance-* options must be specified together")
    if args.lateral_balance_start is not None:
        numeric.extend(lateral_values[1:])
    pitch_values = (
        args.pitch_balance_start, args.pitch_balance_end,
        args.pitch_balance_kp, args.pitch_balance_kd,
        args.pitch_balance_max_angle_deg,
        args.pitch_balance_ramp_in, args.pitch_balance_ramp_out,
    )
    if any(value is not None for value in pitch_values) and not all(
        value is not None for value in pitch_values
    ):
        parser.error("all --pitch-balance-* options must be specified together")
    if args.pitch_balance_start is not None:
        numeric.extend(pitch_values)
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all numeric arguments must be finite")
    if args.dt <= 0.0:
        parser.error("--dt must be positive")
    if min(args.hold_before, args.replay_end, args.hold_after,
           args.world_flat_start, args.world_flat_end) < 0.0:
        parser.error("time arguments must be non-negative")
    if args.world_flat_end < args.world_flat_start:
        parser.error("--world-flat-end must be >= --world-flat-start")
    if min(args.world_flat_ramp_in, args.world_flat_ramp_out) < 0.0:
        parser.error("world-flat ramp durations must be non-negative")
    if args.world_flat_start + args.world_flat_ramp_in > args.world_flat_end:
        parser.error("world-flat ramp-in must finish no later than its end")
    if args.swing_world_z_start is not None:
        if min(args.swing_world_z_start, args.swing_world_z_end) < 0.0:
            parser.error("swing world-Z times must be non-negative")
        if args.swing_world_z_end < args.swing_world_z_start:
            parser.error("--swing-world-z-end must be >= --swing-world-z-start")
        if min(args.swing_world_z_ramp_in, args.swing_world_z_ramp_out) < 0.0:
            parser.error("swing world-Z ramp durations must be non-negative")
        if args.swing_world_z_start + args.swing_world_z_ramp_in > args.swing_world_z_end:
            parser.error("swing world-Z ramp-in must finish no later than its end")
    if (
        args.max_feedback_joint_speed_deg_s is not None
        and args.max_feedback_joint_speed_deg_s <= 0.0
    ):
        parser.error("--max-feedback-joint-speed-deg-s must be positive")
    if (
        args.swing_world_z_max_correction_release_speed_mps is not None
        and args.swing_world_z_max_correction_release_speed_mps <= 0.0
    ):
        parser.error(
            "--swing-world-z-max-correction-release-speed-mps must be positive"
        )
    if (
        args.swing_world_z_max_correction_release_speed_mps is not None
        and args.swing_world_z_start is None
    ):
        parser.error(
            "--swing-world-z-max-correction-release-speed-mps requires "
            "swing-world-Z"
        )
    if args.lateral_balance_start is not None:
        if min(
            args.lateral_balance_start, args.lateral_balance_end,
            args.lateral_balance_kp, args.lateral_balance_kd,
            args.lateral_balance_ramp_in, args.lateral_balance_ramp_out,
        ) < 0.0:
            parser.error("lateral balance times and gains must be non-negative")
        if args.lateral_balance_max_offset_m <= 0.0:
            parser.error("--lateral-balance-max-offset-m must be positive")
        if args.lateral_balance_end < args.lateral_balance_start:
            parser.error("lateral balance end must be >= start")
        if (
            args.lateral_balance_start + args.lateral_balance_ramp_in
            > args.lateral_balance_end
        ):
            parser.error("lateral balance ramp-in must finish by its end")
    if not math.isfinite(args.lateral_balance_support_handoff_ramp_s):
        parser.error("lateral balance support handoff ramp must be finite")
    if args.lateral_balance_support_handoff_ramp_s < 0.0:
        parser.error(
            "--lateral-balance-support-handoff-ramp-s must be non-negative"
        )
    if args.lateral_balance_support_handoff and not args.touchdown_z_arrest:
        parser.error(
            "--lateral-balance-support-handoff requires --touchdown-z-arrest"
        )
    if args.pitch_balance_start is not None:
        if min(
            args.pitch_balance_start, args.pitch_balance_end,
            args.pitch_balance_kp, args.pitch_balance_kd,
            args.pitch_balance_ramp_in, args.pitch_balance_ramp_out,
        ) < 0.0:
            parser.error("pitch balance times and gains must be non-negative")
        if args.pitch_balance_max_angle_deg <= 0.0:
            parser.error("--pitch-balance-max-angle-deg must be positive")
        if args.pitch_balance_end < args.pitch_balance_start:
            parser.error("pitch balance end must be >= start")
        if (
            args.pitch_balance_start + args.pitch_balance_ramp_in
            > args.pitch_balance_end
        ):
            parser.error("pitch balance ramp-in must finish by its end")
        if args.world_flat_leg in ("left", "both"):
            parser.error(
                "pitch balance cannot be combined with left world-flat; "
                "use --world-flat-leg right"
            )
    handoff_numeric = (
        args.pitch_balance_support_handoff_ramp_s,
        args.pitch_balance_right_max_angle_deg,
    )
    if not all(math.isfinite(value) for value in handoff_numeric):
        parser.error("pitch balance support handoff arguments must be finite")
    if args.pitch_balance_support_handoff_ramp_s < 0.0:
        parser.error(
            "--pitch-balance-support-handoff-ramp-s must be non-negative"
        )
    if args.pitch_balance_right_max_angle_deg <= 0.0:
        parser.error("--pitch-balance-right-max-angle-deg must be positive")
    if (
        args.pitch_balance_right_rate_limit_deg_s is not None
        and (
            not math.isfinite(args.pitch_balance_right_rate_limit_deg_s)
            or args.pitch_balance_right_rate_limit_deg_s <= 0.0
        )
    ):
        parser.error(
            "--pitch-balance-right-rate-limit-deg-s must be finite and positive"
        )
    impact_anchor_times = (
        args.left_support_impact_anchor_hold_s,
        args.left_support_impact_anchor_release_s,
    )
    if not all(math.isfinite(value) for value in impact_anchor_times):
        parser.error("LEFT support impact anchor times must be finite")
    if min(impact_anchor_times) < 0.0:
        parser.error("LEFT support impact anchor times must be non-negative")
    if args.left_support_impact_anchor and not args.touchdown_z_arrest:
        parser.error(
            "--left-support-impact-anchor requires --touchdown-z-arrest"
        )
    if args.pitch_balance_support_handoff:
        if args.pitch_balance_start is None:
            parser.error(
                "--pitch-balance-support-handoff requires pitch balance"
            )
        if not args.touchdown_z_arrest:
            parser.error(
                "--pitch-balance-support-handoff requires --touchdown-z-arrest"
            )
    touchdown_numeric = (
        args.touchdown_right_fz_threshold,
        args.touchdown_confirm_s,
        args.touchdown_left_feedback_ramp_out,
        args.touchdown_z_arrest_threshold_n,
        args.touchdown_z_arrest_confirm_s,
        args.touchdown_z_arrest_settle_depth_m,
        args.touchdown_z_arrest_settle_speed_mps,
        args.left_support_z_release_s,
    )
    if not all(math.isfinite(value) for value in touchdown_numeric):
        parser.error("touchdown numeric arguments must be finite")
    if args.touchdown_right_fz_threshold <= 0.0:
        parser.error("--touchdown-right-fz-threshold must be positive")
    if args.touchdown_confirm_s < 0.0:
        parser.error("--touchdown-confirm-s must be non-negative")
    if args.touchdown_left_feedback_ramp_out < 0.0:
        parser.error("--touchdown-left-feedback-ramp-out must be non-negative")
    if args.touchdown_z_arrest_threshold_n <= 0.0:
        parser.error("--touchdown-z-arrest-threshold-n must be positive")
    if args.touchdown_z_arrest_confirm_s < 0.0:
        parser.error("--touchdown-z-arrest-confirm-s must be non-negative")
    if args.touchdown_z_arrest_settle_depth_m < 0.0:
        parser.error(
            "--touchdown-z-arrest-settle-depth-m must be non-negative"
        )
    if args.touchdown_z_arrest_settle_speed_mps < 0.0:
        parser.error(
            "--touchdown-z-arrest-settle-speed-mps must be non-negative"
        )
    if args.touchdown_z_arrest_post_settle_release_s is not None:
        if (
            not math.isfinite(args.touchdown_z_arrest_post_settle_release_s)
            or args.touchdown_z_arrest_post_settle_release_s <= 0.0
        ):
            parser.error(
                "--touchdown-z-arrest-post-settle-release-s must be finite "
                "and positive"
            )
        if not args.touchdown_z_arrest:
            parser.error(
                "--touchdown-z-arrest-post-settle-release-s requires "
                "--touchdown-z-arrest"
            )
        if (
            args.touchdown_z_arrest_settle_depth_m <= 0.0
            or args.touchdown_z_arrest_settle_speed_mps <= 0.0
        ):
            parser.error(
                "--touchdown-z-arrest-post-settle-release-s requires "
                "positive touchdown settle depth and speed"
            )
    if not 0.02 <= args.left_support_z_release_s <= 0.20:
        parser.error("--left-support-z-release-s must be in [0.02, 0.20]")
    if not math.isfinite(args.fore_aft_sign_test_ramp_in_s):
        parser.error("--fore-aft-sign-test-ramp-in-s must be finite")
    if args.fore_aft_sign_test_ramp_in_s < 0.0:
        parser.error("--fore-aft-sign-test-ramp-in-s must be non-negative")
    if args.fore_aft_sign_test_correction_deg is not None:
        if not math.isfinite(args.fore_aft_sign_test_correction_deg):
            parser.error(
                "--fore-aft-sign-test-correction-deg must be finite"
            )
        if abs(args.fore_aft_sign_test_correction_deg) > 0.25:
            parser.error(
                "--fore-aft-sign-test-correction-deg must be in "
                "[-0.25, +0.25]"
            )
        if (
            abs(args.fore_aft_sign_test_correction_deg) > 0.0
            and not args.touchdown_z_arrest
        ):
            parser.error(
                "--fore-aft-sign-test-correction-deg requires "
                "--touchdown-z-arrest for stable-support confirmation"
            )
    flatten_numeric = (
        args.right_touchdown_flatten_ramp_in_s,
        args.right_touchdown_flatten_ramp_out_s,
    )
    if not all(math.isfinite(value) for value in flatten_numeric):
        parser.error("RIGHT touchdown flatten ramp arguments must be finite")
    if min(flatten_numeric) < 0.0:
        parser.error("RIGHT touchdown flatten ramps must be non-negative")
    flatten_correction = args.right_touchdown_flatten_correction_deg
    flatten_enabled = flatten_correction is not None and flatten_correction != 0.0
    if flatten_correction is not None:
        if not math.isfinite(flatten_correction):
            parser.error("RIGHT touchdown flatten correction must be finite")
        if not -1.5 <= flatten_correction <= 0.0:
            parser.error(
                "--right-touchdown-flatten-correction-deg must be in [-1.5, 0]"
            )
    if flatten_enabled:
        if args.right_touchdown_flatten_start is None:
            parser.error(
                "RIGHT touchdown flatten correction requires its start time"
            )
        if not math.isfinite(args.right_touchdown_flatten_start):
            parser.error("RIGHT touchdown flatten start must be finite")
        if args.right_touchdown_flatten_start < 0.0:
            parser.error("RIGHT touchdown flatten start must be non-negative")
        if not args.touchdown_z_arrest:
            parser.error(
                "RIGHT touchdown flatten correction requires --touchdown-z-arrest"
            )
        if (
            args.fore_aft_sign_test_correction_deg is not None
            and args.fore_aft_sign_test_correction_deg != 0.0
        ):
            parser.error(
                "RIGHT touchdown flatten correction and fore-aft sign-test "
                "cannot both be nonzero"
            )
    if args.touchdown_z_arrest and args.touchdown_support_hold:
        parser.error(
            "--touchdown-z-arrest and --touchdown-support-hold are mutually "
            "exclusive"
        )
    if args.touchdown_z_arrest and args.swing_world_z_start is None:
        parser.error("--touchdown-z-arrest requires swing-world-Z")
    if (
        args.left_support_z_hold_until_right_confirmed
        and not args.touchdown_z_arrest
    ):
        parser.error(
            "--left-support-z-hold-until-right-confirmed requires "
            "--touchdown-z-arrest"
        )
    if (
        args.left_support_z_hold_through_replay
        and not args.left_support_z_hold_until_right_confirmed
    ):
        parser.error(
            "--left-support-z-hold-through-replay requires "
            "--left-support-z-hold-until-right-confirmed"
        )
    if (
        args.touchdown_z_arrest
        and args.swing_world_z_max_correction_release_speed_mps is None
    ):
        parser.error(
            "--touchdown-z-arrest requires correction release speed for "
            "continuous false-contact recovery"
        )
    if args.touchdown_support_hold and args.swing_world_z_start is None:
        parser.error(
            "--touchdown-support-hold requires the right swing-world-Z options "
            "so contact detection can be armed after swing starts"
        )
    return args


def quaternion_rotation(quaternion):
    values = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm <= 0.0:
        return None
    return Rotation.from_quat(values / norm).as_matrix()


class SimulationClock:
    def __init__(self, topic):
        self.condition = threading.Condition()
        self.simulation_time = None
        self.base_rotation = None
        self.base_position = np.full(3, math.nan)
        self.base_x = math.nan
        self.base_vx = math.nan
        self.base_pitch_rate = math.nan
        self.callback = self.on_odometry
        self.node = Node()
        if self.node.subscribe(odometry_pb2.Odometry, topic, self.callback) is False:
            raise RuntimeError(f"failed to subscribe to {topic}")

    def on_odometry(self, message):
        stamp = message.header.stamp
        orientation = message.pose.orientation
        simulation_time = float(stamp.sec) + float(stamp.nsec) * 1e-9
        base_rotation = quaternion_rotation((
            orientation.x, orientation.y, orientation.z, orientation.w,
        ))
        base_x = float(message.pose.position.x)
        base_position = np.array((
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ), dtype=float)
        base_vx = float(message.twist.linear.x)
        base_pitch_rate = float(message.twist.angular.y)
        with self.condition:
            self.simulation_time = simulation_time
            self.base_rotation = base_rotation
            self.base_position = base_position
            self.base_x = base_x
            self.base_vx = base_vx
            self.base_pitch_rate = base_pitch_rate
            self.condition.notify_all()

    def wait_for_time_after(self, previous_time, timeout=None):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: not (
                    self.simulation_time is None
                    or (previous_time is not None and self.simulation_time <= previous_time)
                ),
                timeout=timeout,
            )
            rotation = (
                None if self.base_rotation is None else self.base_rotation.copy()
            )
            return (
                self.simulation_time, rotation, self.base_position.copy(),
                self.base_x, self.base_vx, self.base_pitch_rate, ready,
            )


def joint_name_from_command_topic(topic):
    return topic.removeprefix("/step/").removesuffix("/cmd_pos")


RIGHT_JOINT_NAMES = tuple(
    joint_name_from_command_topic(LEG_JOINT_TOPICS[f"RL{index}_wrap"])
    for index in range(6)
)
LEFT_JOINT_NAMES = tuple(
    joint_name_from_command_topic(LEG_JOINT_TOPICS[f"LL{index}_wrap"])
    for index in range(6)
)


class TouchdownMeasurements:
    """Read right contact force and optional measured touchdown joints."""

    def __init__(self, contact_topic, joint_state_topic,
                 require_joint_state=True):
        self.lock = threading.Lock()
        self.right_fz = math.nan
        self.contact_sequence = 0
        self.joint_positions = {}
        self.node = Node()
        subscriptions = [
            self.node.subscribe(Contacts, contact_topic, self.on_contacts),
        ]
        if require_joint_state:
            subscriptions.append(self.node.subscribe(
                Model, joint_state_topic, self.on_joint_state,
            ))
        if not all(subscriptions):
            raise RuntimeError("failed to subscribe to touchdown measurements")

    def on_contacts(self, message):
        total_fz = 0.0
        found = False
        for contact in message.contact:
            collision1_is_sole = (
                RIGHT_SOLE_COLLISION_NAME in contact.collision1.name
            )
            collision2_is_sole = (
                RIGHT_SOLE_COLLISION_NAME in contact.collision2.name
            )
            if not collision1_is_sole and not collision2_is_sole:
                continue
            found = True
            for wrench in contact.wrench:
                force = (
                    wrench.body_1_wrench.force
                    if collision1_is_sole else wrench.body_2_wrench.force
                )
                total_fz += float(force.z)
        with self.lock:
            self.right_fz = total_fz if found else 0.0
            self.contact_sequence += 1

    def on_joint_state(self, message):
        positions = {}
        for joint in message.joint:
            name = joint.name.replace("/", "::").rsplit("::", 1)[-1]
            positions[name] = float(joint.axis1.position)
        with self.lock:
            self.joint_positions.update(positions)

    def snapshot(self):
        with self.lock:
            right_q = np.array([
                self.joint_positions.get(name, math.nan)
                for name in RIGHT_JOINT_NAMES
            ])
            left_q = np.array([
                self.joint_positions.get(name, math.nan)
                for name in LEFT_JOINT_NAMES
            ])
            return self.right_fz, self.contact_sequence, right_q, left_q


def load_targets(path):
    required = [f"RL{i}" for i in range(6)] + [f"LL{i}" for i in range(6)]
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"CSV is empty: {path}")
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise RuntimeError(f"missing CSV columns: {', '.join(missing)}")
    targets = []
    for index, row in enumerate(rows):
        frame = {}
        for side in ("RL", "LL"):
            for joint in range(6):
                key = f"{side}{joint}"
                value = float(row[key])
                if not math.isfinite(value):
                    raise RuntimeError(f"non-finite target at row {index}, {key}")
                frame[f"{key}_wrap"] = value
        targets.append(frame)
    return targets


def load_planner_lateral_reference(path, dt):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or "COM_y" not in rows[0]:
        raise RuntimeError(f"planner CSV missing COM_y: {path}")
    com_y = np.array([float(row["COM_y"]) for row in rows])
    if not np.all(np.isfinite(com_y)):
        raise RuntimeError(f"non-finite planner COM_y: {path}")
    displacement = com_y - com_y[0]
    velocity = np.empty_like(com_y)
    if len(com_y) == 1:
        velocity[0] = 0.0
    else:
        velocity[0] = (com_y[1] - com_y[0]) / dt
        velocity[-1] = (com_y[-1] - com_y[-2]) / dt
        if len(com_y) > 2:
            velocity[1:-1] = (com_y[2:] - com_y[:-2]) / (2.0 * dt)
    return displacement, velocity


def leg_vector(targets, prefix):
    return np.array([targets[f"{prefix}{joint}_wrap"] for joint in range(6)])


def set_leg_vector(targets, prefix, values):
    result = dict(targets)
    for joint, value in enumerate(values):
        result[f"{prefix}{joint}_wrap"] = float(value)
    return result


def limit_feedback_joint_rate(requested_q, previous_published_q,
                              max_speed_deg_s, dt):
    requested_delta = requested_q - previous_published_q
    requested_speed = np.max(np.abs(requested_delta)) / dt
    if max_speed_deg_s is None:
        return requested_q.copy(), 0, requested_speed, requested_speed
    max_delta = math.radians(max_speed_deg_s) * dt
    limited_delta = np.clip(requested_delta, -max_delta, max_delta)
    published_q = previous_published_q + limited_delta
    limited_count = int(np.count_nonzero(
        np.abs(requested_delta) > max_delta + 1e-15
    ))
    published_speed = np.max(np.abs(limited_delta)) / dt
    return published_q, limited_count, requested_speed, published_speed


def rotation_error_angle(target, actual):
    return np.linalg.norm(
        Rotation.from_matrix(target.T @ actual).as_rotvec()
    )


def smoothstep(unit_value):
    value = max(0.0, min(1.0, unit_value))
    return 3.0 * value * value - 2.0 * value * value * value


def release_touchdown_downward_block(air_world_z, original_target_world_z,
                                     release_beta):
    blocked_before = max(0.0, original_target_world_z - air_world_z)
    if release_beta <= 0.0:
        return original_target_world_z, blocked_before, blocked_before
    blocked_after = (1.0 - release_beta) * blocked_before
    return air_world_z + blocked_after, blocked_before, blocked_after


def feedback_beta(phase, trajectory_time, start, end, ramp_in, ramp_out):
    if phase != "REPLAY" or start is None:
        return 0.0
    if trajectory_time < start or trajectory_time > end + ramp_out:
        return 0.0
    if ramp_in > 0.0 and trajectory_time < start + ramp_in:
        return smoothstep((trajectory_time - start) / ramp_in)
    if trajectory_time <= end:
        return 1.0
    if ramp_out <= 0.0:
        return 0.0
    return 1.0 - smoothstep((trajectory_time - end) / ramp_out)


def fore_aft_sign_test_beta(trajectory_time, confirmed_time, ramp_in):
    if not math.isfinite(confirmed_time) or trajectory_time < confirmed_time:
        return 0.0
    if ramp_in <= 0.0:
        return 1.0
    return smoothstep((trajectory_time - confirmed_time) / ramp_in)


def pitch_balance_support_handoff(trajectory_time, touchdown_z_arrest,
                                  enabled, ramp_s):
    if not enabled:
        return 0.0, "DISABLED"
    if (
        touchdown_z_arrest is None
        or touchdown_z_arrest.state != "TOUCHDOWN_CONFIRMED"
        or not math.isfinite(touchdown_z_arrest.confirmed_time)
    ):
        state = (
            "WAITING_FOR_TOUCHDOWN"
            if touchdown_z_arrest is None else touchdown_z_arrest.state
        )
        return 0.0, state
    elapsed = trajectory_time - touchdown_z_arrest.confirmed_time
    beta = 1.0 if ramp_s <= 0.0 else smoothstep(elapsed / ramp_s)
    return beta, ("RIGHT_SUPPORT" if beta >= 1.0 else "HANDOFF_RAMP")


def lateral_balance_support_handoff(trajectory_time, touchdown_z_arrest,
                                    enabled, ramp_s):
    if not enabled:
        return 0.0, "DISABLED"
    if (
        touchdown_z_arrest is None
        or touchdown_z_arrest.state != "TOUCHDOWN_CONFIRMED"
        or not math.isfinite(touchdown_z_arrest.confirmed_time)
    ):
        return 0.0, "WAITING_FOR_TOUCHDOWN"
    elapsed = trajectory_time - touchdown_z_arrest.confirmed_time
    beta = 1.0 if ramp_s <= 0.0 else smoothstep(elapsed / ramp_s)
    return beta, ("RIGHT_SUPPORT" if beta >= 1.0 else "HANDOFF_RAMP")


def split_lateral_support_offset(lateral_offset, handoff_beta, enabled):
    if not enabled:
        return lateral_offset, 0.0
    return (
        (1.0 - handoff_beta) * lateral_offset,
        handoff_beta * lateral_offset,
    )


def propose_angular_rate_limited(target, previous, rate_limit_deg_s, dt):
    if rate_limit_deg_s is None:
        return target, False
    max_delta = math.radians(rate_limit_deg_s) * dt
    delta = target - previous
    applied_delta = max(-max_delta, min(max_delta, delta))
    output = previous + applied_delta
    return output, abs(applied_delta - delta) > 1e-15


def touchdown_flatten_diagnostics(trajectory_time, requested_correction,
                                  start, ramp_in, ramp_out,
                                  touchdown_z_arrest):
    enabled = requested_correction != 0.0
    activation_beta = 0.0
    if enabled and trajectory_time >= start:
        activation_beta = (
            1.0 if ramp_in <= 0.0
            else smoothstep((trajectory_time - start) / ramp_in)
        )
    confirmed = (
        enabled
        and touchdown_z_arrest.state == "TOUCHDOWN_CONFIRMED"
        and math.isfinite(touchdown_z_arrest.confirmed_time)
    )
    release_beta = 0.0
    if confirmed:
        release_beta = (
            1.0 if ramp_out <= 0.0
            else smoothstep(
                (trajectory_time - touchdown_z_arrest.confirmed_time)
                / ramp_out
            )
        )
    beta = activation_beta * (1.0 - release_beta)
    if not enabled:
        state = "DISABLED"
    elif release_beta >= 1.0:
        state = "RELEASED"
    elif confirmed:
        state = "RELEASING"
    elif activation_beta >= 1.0:
        state = "HOLDING"
    elif activation_beta > 0.0:
        state = "RAMP_IN"
    else:
        state = "WAITING"
    return {
        "enabled": enabled, "requested": requested_correction,
        "beta": beta, "applied": beta * requested_correction,
        "state": state, "release_beta": release_beta,
    }


def world_flat_orientation(world_reference, nominal_world_rotation):
    """Keep the flat reference tilt while following nominal world yaw."""
    reference_heading = world_reference[:2, 0]
    nominal_heading = nominal_world_rotation[:2, 0]
    reference_norm = np.linalg.norm(reference_heading)
    nominal_norm = np.linalg.norm(nominal_heading)
    if reference_norm <= 1e-12 or nominal_norm <= 1e-12:
        return world_reference.copy()
    reference_heading = reference_heading / reference_norm
    nominal_heading = nominal_heading / nominal_norm
    yaw_delta = math.atan2(
        reference_heading[0] * nominal_heading[1]
        - reference_heading[1] * nominal_heading[0],
        np.dot(reference_heading, nominal_heading),
    )
    return Rotation.from_euler("z", yaw_delta).as_matrix() @ world_reference


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def apply_world_x_yaw_anchor(target, base_rotation, base_position,
                             anchor_world_x, anchor_world_yaw, beta):
    world_position = base_position + base_rotation @ target[:3, 3]
    world_rpy = Rotation.from_matrix(
        base_rotation @ target[:3, :3]
    ).as_euler("xyz")
    diagnostics = {
        "target_world_x_before": world_position[0],
        "target_world_x_after": world_position[0],
        "target_world_yaw_before": world_rpy[2],
        "target_world_yaw_after": world_rpy[2],
    }
    if beta <= 0.0:
        return target.copy(), diagnostics

    anchored = target.copy()
    final_world_position = world_position.copy()
    final_world_position[0] = (
        beta * anchor_world_x + (1.0 - beta) * world_position[0]
    )
    final_world_yaw = world_rpy[2] + beta * wrap_to_pi(
        anchor_world_yaw - world_rpy[2]
    )
    final_world_rotation = Rotation.from_euler(
        "xyz", [world_rpy[0], world_rpy[1], final_world_yaw]
    ).as_matrix()
    anchored[:3, 3] = base_rotation.T @ (
        final_world_position - base_position
    )
    anchored[:3, :3] = base_rotation.T @ final_world_rotation
    diagnostics.update({
        "target_world_x_after": final_world_position[0],
        "target_world_yaw_after": final_world_yaw,
    })
    return anchored, diagnostics


class LeftSupportImpactAnchor:
    def __init__(self, enabled, hold_s, release_s, chain):
        self.enabled = enabled
        self.hold_s = hold_s
        self.release_s = release_s
        self.chain = chain
        self.anchor_world_x = math.nan
        self.anchor_world_yaw = math.nan
        self.start_trajectory_time = math.nan

    def try_latch(self, touchdown_z_arrest, measured_left_q,
                  base_rotation, base_position):
        if (
            not self.enabled
            or math.isfinite(self.start_trajectory_time)
            or touchdown_z_arrest is None
            or touchdown_z_arrest.state != "TOUCHDOWN_CONFIRMED"
            or not math.isfinite(touchdown_z_arrest.confirmed_time)
            or not np.all(np.isfinite(measured_left_q))
        ):
            return
        measured_pose = self.chain.forward(measured_left_q)
        measured_world_position = (
            base_position + base_rotation @ measured_pose[:3, 3]
        )
        measured_world_rotation = base_rotation @ measured_pose[:3, :3]
        self.anchor_world_x = measured_world_position[0]
        self.anchor_world_yaw = Rotation.from_matrix(
            measured_world_rotation
        ).as_euler("xyz")[2]
        self.start_trajectory_time = touchdown_z_arrest.confirmed_time

    def beta(self, trajectory_time):
        if not math.isfinite(self.start_trajectory_time):
            return 0.0
        elapsed = max(0.0, trajectory_time - self.start_trajectory_time)
        if elapsed <= self.hold_s:
            return 1.0
        if self.release_s <= 0.0:
            return 0.0
        return 1.0 - smoothstep((elapsed - self.hold_s) / self.release_s)

    def state(self, trajectory_time):
        if not self.enabled:
            return "DISABLED"
        if not math.isfinite(self.start_trajectory_time):
            return "WAITING_FOR_TOUCHDOWN"
        beta = self.beta(trajectory_time)
        if beta <= 0.0:
            return "RELEASED"
        if beta >= 1.0:
            return "HOLDING"
        return "RELEASING"

    def diagnostics(self, trajectory_time):
        return {
            "enabled": self.enabled,
            "state": self.state(trajectory_time),
            "beta": self.beta(trajectory_time),
            "anchor_world_x": self.anchor_world_x,
            "anchor_world_yaw": self.anchor_world_yaw,
            "start_trajectory_time": self.start_trajectory_time,
            "target_world_x_before": math.nan,
            "target_world_x_after": math.nan,
            "target_world_yaw_before": math.nan,
            "target_world_yaw_after": math.nan,
        }


class WorldFlatRightSolver:
    def __init__(self, chain, world_reference, dt, initial_base_rotation,
                 initial_right_position, initial_left_position,
                 leg_name="right", lateral_scale=None,
                 lateral_reference_frame=None, lateral_reference_x=None):
        self.chain = chain
        self.leg_name = leg_name
        self.world_reference = world_reference
        self.dt = dt
        self.initial_right_position = initial_right_position.copy()
        self.initial_left_position = initial_left_position.copy()
        self.initial_world_relative_z = (
            initial_base_rotation
            @ (initial_right_position - initial_left_position)
        )[2]
        self.previous_solution = None
        self.cached_frame = None
        self.cached_base_rotation = None
        self.cached_sim_time = None
        self.cached_result = None
        self.previous_successful_correction = None
        self.correction_release_state = "NORMAL"
        self.correction_release_limited_frame_count = 0
        self.lateral_scale = lateral_scale
        self.lateral_reference_frame = lateral_reference_frame
        self.lateral_reference_x = lateral_reference_x

    def _can_reuse(self, frame, base_rotation, simulation_time):
        if self.cached_result is None or frame != self.cached_frame:
            return False
        base_change = rotation_error_angle(
            self.cached_base_rotation, base_rotation,
        )
        if base_change <= ORIENTATION_CACHE_EPS_RAD:
            return True
        return simulation_time - self.cached_sim_time < self.dt - 1e-12

    def solve(self, frame, simulation_time, base_rotation, nominal_q,
              nominal_left_q, world_flat_beta, swing_world_z_beta,
              left_chain, support_x_offset=0.0,
              support_pitch_correction=0.0,
              support_pitch_is_right=False,
              world_x_orientation_correction=0.0,
              correction_release_speed_mps=None,
              correction_release_active=False, base_position=None,
              touchdown_z_arrest=None, support_z_hold=None,
              support_z_trajectory_time=None, impact_anchor=None,
              impact_anchor_trajectory_time=None):
        if self._can_reuse(frame, base_rotation, simulation_time):
            return self.cached_result

        nominal_pose = self.chain.forward(nominal_q)
        nominal_left_pose = left_chain.forward(nominal_left_q)
        target = nominal_pose.copy()
        if (
            self.lateral_scale is not None
            and self.lateral_scale < 1.0
            and frame >= self.lateral_reference_frame
        ):
            target[0, 3] = self.lateral_reference_x + self.lateral_scale * (
                nominal_pose[0, 3] - self.lateral_reference_x
            )
        target[0, 3] += support_x_offset
        nominal_world_rotation = base_rotation @ nominal_pose[:3, :3]
        world_target_rotation = base_rotation @ target[:3, :3]
        if world_flat_beta > 0.0:
            flat_world_rotation = world_flat_orientation(
                self.world_reference, nominal_world_rotation,
            )
            flat_base_rotation = base_rotation.T @ flat_world_rotation
            relative_rotation = Rotation.from_matrix(
                nominal_pose[:3, :3].T @ flat_base_rotation
            ).as_rotvec()
            target[:3, :3] = (
                nominal_pose[:3, :3]
                @ Rotation.from_rotvec(
                    world_flat_beta * relative_rotation
                ).as_matrix()
            )
            world_target_rotation = base_rotation @ target[:3, :3]
        # Pitch balance is defined about the support-foot base-frame +Y axis.
        # Pre-multiply the composed base-frame target so RIGHT world-flat and
        # this correction both survive in the single final IK target.
        if abs(support_pitch_correction) > 0.0:
            target[:3, :3] = (
                Rotation.from_rotvec(
                    [0.0, support_pitch_correction, 0.0]
                ).as_matrix()
                @ target[:3, :3]
            )
            world_target_rotation = base_rotation @ target[:3, :3]
        world_x_before = world_target_rotation.copy()
        if abs(world_x_orientation_correction) > 0.0:
            # Left multiplication applies the delta about the fixed world X
            # axis. Convert the composed world target back to the base frame
            # expected by the leg IK without changing target translation.
            world_target_rotation = (
                Rotation.from_rotvec(
                    [world_x_orientation_correction, 0.0, 0.0]
                ).as_matrix()
                @ world_target_rotation
            )
            target[:3, :3] = base_rotation.T @ world_target_rotation
        before_error = rotation_error_angle(
            world_target_rotation, nominal_world_rotation,
        )

        right_position = nominal_pose[:3, 3]
        left_position = nominal_left_pose[:3, 3]
        relative = right_position - left_position
        delta_z_motion = (
            right_position[2] - self.initial_right_position[2]
            - (left_position[2] - self.initial_left_position[2])
        )
        desired_relative_world_z = self.initial_world_relative_z + delta_z_motion
        predicted_before = (base_rotation @ relative)[2]
        tilt_r20_dx = base_rotation[2, 0] * relative[0]
        tilt_r21_dy = base_rotation[2, 1] * relative[1]
        z_valid = True
        correction_desired = swing_world_z_beta * (
            desired_relative_world_z - predicted_before
        )
        correction_output = correction_desired
        correction_diagnostics = {
            "active": False,
            "state": "NORMAL",
            "raw": desired_relative_world_z - predicted_before,
            "desired": correction_desired,
            "previous": math.nan,
            "output": correction_output,
            "delta": math.nan,
            "velocity": math.nan,
            "limited": False,
            "limited_frame_count": self.correction_release_limited_frame_count,
        }
        release_tail_active = (
            correction_release_speed_mps is not None
            and self.correction_release_state == "RELEASE_TAIL"
            and self.previous_successful_correction is not None
            and abs(self.previous_successful_correction)
            > CORRECTION_RELEASE_TOLERANCE_M
        )
        correction_active = (
            swing_world_z_beta > 0.0
            or release_tail_active
            or (
                correction_release_active
                and self.previous_successful_correction is not None
                and abs(self.previous_successful_correction)
                > CORRECTION_RELEASE_TOLERANCE_M
            )
        )
        if correction_active:
            if abs(base_rotation[2, 2]) <= 1e-6:
                z_valid = False
                print(
                    f"[WARNING] swing world-Z R22 too small at frame {frame}; "
                    "using nominal Z"
                )
            elif correction_release_speed_mps is None:
                compensated_z = left_position[2] + (
                    desired_relative_world_z - tilt_r20_dx - tilt_r21_dy
                ) / base_rotation[2, 2]
                target[2, 3] = right_position[2] + swing_world_z_beta * (
                    compensated_z - right_position[2]
                )
            else:
                previous = self.previous_successful_correction
                release_limited = (
                    previous is not None
                    and (correction_release_active or release_tail_active)
                    and abs(correction_desired) < abs(previous) - 1e-15
                )
                if release_limited:
                    max_delta = correction_release_speed_mps * self.dt
                    correction_output = max(
                        previous - max_delta,
                        min(previous + max_delta, correction_desired),
                    )
                target[2, 3] = (
                    right_position[2]
                    + correction_output / base_rotation[2, 2]
                )
                correction_diagnostics = {
                    "active": release_limited,
                    "state": (
                        "RELEASE_TAIL"
                        if swing_world_z_beta <= 0.0
                        and abs(correction_output)
                        > CORRECTION_RELEASE_TOLERANCE_M
                        else "NORMAL"
                    ),
                    "raw": desired_relative_world_z - predicted_before,
                    "desired": correction_desired,
                    "previous": math.nan if previous is None else previous,
                    "output": correction_output,
                    "delta": (
                        math.nan if previous is None
                        else correction_output - previous
                    ),
                    "velocity": (
                        math.nan if previous is None
                        else (correction_output - previous) / self.dt
                    ),
                    "limited": release_limited and abs(
                        correction_output - correction_desired
                    ) > 1e-12,
                    "limited_frame_count": (
                        self.correction_release_limited_frame_count
                    ),
                }
        arrest_diagnostics = None
        if touchdown_z_arrest is not None:
            if base_position is None:
                raise RuntimeError("touchdown Z arrest requires base position")
            if abs(base_rotation[2, 2]) <= 1e-6:
                z_valid = False
                print(
                    f"[WARNING] touchdown Z arrest R22 too small at frame "
                    f"{frame}; using the air target"
                )
            else:
                air_world_z = (
                    base_position[2] + (base_rotation @ target[:3, 3])[2]
                )
                arrest_diagnostics = touchdown_z_arrest.propose(
                    air_world_z, support_z_trajectory_time
                )
                target_world_z = arrest_diagnostics["target_world_z"]
                if arrest_diagnostics["blocked"] > 0.0:
                    target[2, 3] = (
                        target_world_z - base_position[2]
                        - base_rotation[2, 0] * target[0, 3]
                        - base_rotation[2, 1] * target[1, 3]
                    ) / base_rotation[2, 2]
        support_z_diagnostics = None
        if support_z_hold is not None:
            if base_position is None:
                raise RuntimeError("support Z hold requires base position")
            nominal_world_z = (
                base_position[2] + (base_rotation @ target[:3, 3])[2]
            )
            support_z_diagnostics = support_z_hold.propose(
                support_z_trajectory_time, nominal_world_z,
            )
            if support_z_diagnostics["active"]:
                final_world_z = support_z_diagnostics["final_world_z"]
                if abs(base_rotation[2, 2]) <= 1e-6:
                    z_valid = False
                else:
                    target[2, 3] = (
                        final_world_z - base_position[2]
                        - base_rotation[2, 0] * target[0, 3]
                        - base_rotation[2, 1] * target[1, 3]
                    ) / base_rotation[2, 2]
        impact_anchor_diagnostics = None
        if impact_anchor is not None:
            impact_anchor_diagnostics = impact_anchor.diagnostics(
                impact_anchor_trajectory_time
            )
            if impact_anchor_diagnostics["beta"] > 0.0:
                target, applied_diagnostics = apply_world_x_yaw_anchor(
                    target, base_rotation, base_position,
                    impact_anchor.anchor_world_x,
                    impact_anchor.anchor_world_yaw,
                    impact_anchor_diagnostics["beta"],
                )
                impact_anchor_diagnostics.update(applied_diagnostics)
                world_target_rotation = base_rotation @ target[:3, :3]
        target_relative = target[:3, 3] - left_position
        predicted_after_target = (base_rotation @ target_relative)[2]

        if swing_world_z_beta > 0.0 and not z_valid and world_flat_beta <= 0.0:
            result = self._result(
                nominal_q, False, nominal_pose, nominal_left_pose, target,
                base_rotation, world_target_rotation, before_error,
                desired_relative_world_z, predicted_before,
                predicted_before, tilt_r20_dx, tilt_r21_dy,
            )
            result["fore_aft_world_x_correction"] = (
                world_x_orientation_correction
            )
            result["world_x_orientation_before"] = world_x_before
            result["world_x_orientation_after"] = (
                world_target_rotation.copy()
            )
            self._cache(frame, simulation_time, base_rotation, result)
            return result
        seed = (
            nominal_q if self.previous_solution is None
            else self.previous_solution
        )
        try:
            solve_result = solve_leg_frame(self.chain, target, seed)
            candidate = solve_result.x
            valid = (
                solve_result.success
                and np.all(np.isfinite(candidate))
                and np.all(candidate >= self.chain.lower - 1e-10)
                and np.all(candidate <= self.chain.upper + 1e-10)
            )
        except Exception as error:  # Keep nominal commands on solver failure.
            print(
                f"[WARNING] {self.leg_name} 6D IK exception at frame "
                f"{frame}: {error}"
            )
            solve_result = None
            candidate = nominal_q
            valid = False

        solved_q = candidate if valid else nominal_q
        if not valid:
            print(
                f"[WARNING] {self.leg_name} 6D IK failed at frame {frame}; "
                "using nominal CSV"
            )
        else:
            self.previous_solution = candidate.copy()

        predicted_after = (
            predicted_after_target if valid else predicted_before
        )
        result = self._result(
            solved_q, valid, nominal_pose, nominal_left_pose, target,
            base_rotation, world_target_rotation, before_error,
            desired_relative_world_z, predicted_before, predicted_after,
            tilt_r20_dx, tilt_r21_dy,
        )
        result["world_flat_success"] = bool(valid and world_flat_beta > 0.0)
        result["swing_world_z_success"] = bool(
            valid and z_valid and swing_world_z_beta > 0.0
        )
        result["lateral_balance_success"] = bool(
            valid and abs(support_x_offset) > 0.0
        )
        result["pitch_balance_success"] = bool(
            valid and abs(support_pitch_correction) > 0.0
        )
        result["pitch_target_delta"] = support_pitch_correction
        result["right_pitch_balance_success"] = bool(
            valid and support_pitch_is_right
            and abs(support_pitch_correction) > 0.0
        )
        result["fore_aft_world_x_correction"] = (
            world_x_orientation_correction
        )
        result["world_x_orientation_before"] = world_x_before
        result["world_x_orientation_after"] = world_target_rotation.copy()
        result["nominal_base_pitch"] = Rotation.from_matrix(
            nominal_pose[:3, :3]
        ).as_euler("xyz")[1]
        result["target_base_pitch"] = Rotation.from_matrix(
            target[:3, :3]
        ).as_euler("xyz")[1]
        result["world_flat_beta"] = world_flat_beta
        result["swing_world_z_beta"] = swing_world_z_beta
        if valid and correction_release_speed_mps is not None and z_valid:
            if correction_diagnostics["limited"]:
                self.correction_release_limited_frame_count += 1
            correction_diagnostics["limited_frame_count"] = (
                self.correction_release_limited_frame_count
            )
            if (
                swing_world_z_beta <= 0.0
                and abs(correction_output)
                <= CORRECTION_RELEASE_TOLERANCE_M
            ):
                correction_output = 0.0
                self.previous_successful_correction = None
                self.correction_release_state = "NORMAL"
                correction_diagnostics["output"] = 0.0
                correction_diagnostics["state"] = "NORMAL"
            else:
                self.previous_successful_correction = correction_output
                self.correction_release_state = correction_diagnostics["state"]
        result["correction_release"] = correction_diagnostics
        if valid and arrest_diagnostics is not None:
            touchdown_z_arrest.commit_successful_target(
                arrest_diagnostics["target_world_z"]
            )
        if valid and support_z_diagnostics is not None:
            support_z_hold.commit_successful_target(
                support_z_diagnostics["final_world_z"]
            )
        result["touchdown_z_arrest"] = arrest_diagnostics
        result["left_support_z_hold"] = support_z_diagnostics
        result["left_support_impact_anchor"] = impact_anchor_diagnostics
        self._cache(frame, simulation_time, base_rotation, result)
        return result

    def _result(self, solved_q, valid, nominal_pose, nominal_left_pose, target,
                base_rotation, world_target_rotation, before_error,
                desired_relative_world_z, predicted_before, predicted_after,
                tilt_r20_dx, tilt_r21_dy):
        solved_pose = self.chain.forward(solved_q)
        position_error, orientation_error = pose_errors(solved_pose, target)
        after_error = rotation_error_angle(
            world_target_rotation, base_rotation @ solved_pose[:3, :3],
        )
        desired_rpy = Rotation.from_matrix(target[:3, :3]).as_euler("xyz")
        return {
            "q": solved_q.copy(),
            "success": bool(valid),
            "world_flat_success": False,
            "swing_world_z_success": False,
            "nominal_pos_err_mm": 1000.0 * np.linalg.norm(
                nominal_pose[:3, 3] - target[:3, 3]
            ),
            "world_ori_before_deg": math.degrees(before_error),
            "world_ori_after_deg": math.degrees(after_error),
            "ik_pos_err_mm": 1000.0 * np.linalg.norm(position_error),
            "ik_ori_err_deg": math.degrees(np.linalg.norm(orientation_error)),
            "desired_rpy": desired_rpy,
            "world_target_rpy": Rotation.from_matrix(
                world_target_rotation
            ).as_euler("xyz"),
            "nominal_relative_world_z": (
                nominal_pose[2, 3] - nominal_left_pose[2, 3]
            ),
            "desired_relative_world_z": desired_relative_world_z,
            "predicted_relative_world_z_before": predicted_before,
            "predicted_relative_world_z_after": predicted_after,
            "world_z_compensation": target[2, 3] - nominal_pose[2, 3],
            "target_base_z": target[2, 3],
            "tilt_r20_dx": tilt_r20_dx,
            "tilt_r21_dy": tilt_r21_dy,
            "lateral_target_x": target[0, 3],
        }

    def _cache(self, frame, simulation_time, base_rotation, result):
        self.cached_frame = frame
        self.cached_base_rotation = base_rotation.copy()
        self.cached_sim_time = simulation_time
        self.cached_result = result


class LeftSupportZHold:
    """Hold one latched LEFT sole world-Z through stable RIGHT touchdown."""

    def __init__(self, release_duration, touchdown_z_arrest,
                 hold_through_replay=False):
        self.release_duration = release_duration
        self.touchdown_z_arrest = touchdown_z_arrest
        self.hold_through_replay = hold_through_replay
        self.hold_world_z = None
        self.hold_start_time = math.nan
        self.release_start_time = math.nan
        self.previous_final_world_z = None

    def propose(self, trajectory_time, nominal_world_z):
        touchdown_z_arrest = self.touchdown_z_arrest
        if self.hold_world_z is None and touchdown_z_arrest.unloaded_once:
            self.hold_world_z = (
                nominal_world_z
                if self.previous_final_world_z is None
                else self.previous_final_world_z
            )
            self.hold_start_time = trajectory_time
        confirmed = (
            touchdown_z_arrest.state == "TOUCHDOWN_CONFIRMED"
            and math.isfinite(touchdown_z_arrest.confirmed_time)
        )
        if (
            confirmed and not self.hold_through_replay
            and not math.isfinite(self.release_start_time)
        ):
            self.release_start_time = touchdown_z_arrest.confirmed_time
        beta = 0.0
        release_active = False
        active = self.hold_world_z is not None
        if active and math.isfinite(self.release_start_time):
            beta = smoothstep(
                (trajectory_time - self.release_start_time)
                / self.release_duration
            )
            release_active = beta < 1.0
        final_world_z = (
            nominal_world_z if not active
            else (1.0 - beta) * self.hold_world_z + beta * nominal_world_z
        )
        return {
            "enabled": True,
            "active": active and beta < 1.0,
            "hold_world_z": (
                math.nan if self.hold_world_z is None else self.hold_world_z
            ),
            "release_active": release_active,
            "release_beta": beta,
            "nominal_world_z": nominal_world_z,
            "final_world_z": final_world_z,
            "hold_start_time": self.hold_start_time,
            "release_start_time": self.release_start_time,
        }

    def commit_successful_target(self, final_world_z):
        self.previous_final_world_z = final_world_z


class TouchdownZArrest:
    """Block only additional downward RIGHT world-Z motion after contact."""

    def __init__(self, threshold_n, confirm_duration, arm_trajectory_time,
                 recovery_speed_mps, dt, settle_depth_m=0.0,
                 settle_speed_mps=0.0, post_settle_release_s=None):
        self.threshold_n = threshold_n
        self.confirm_duration = confirm_duration
        self.arm_trajectory_time = arm_trajectory_time
        self.recovery_speed_mps = recovery_speed_mps
        self.dt = dt
        self.settle_depth_m = settle_depth_m
        self.settle_speed_mps = settle_speed_mps
        self.post_settle_release_s = post_settle_release_s
        self.state = "SWING"
        self.unloaded_once = False
        self.last_contact_sequence = None
        self.candidate_started_at = None
        self.first_crossing_time = math.nan
        self.confirmed_time = math.nan
        self.hold_world_z = None
        self.previous_successful_world_z = None
        self.false_contact_count = 0
        self.pending_recovery_hold = None
        self.pending_recovery_complete = False
        self.settle_floor_world_z = None
        self.pending_settle_hold = None
        self.post_settle_release_start_time = math.nan

    @property
    def settle_enabled(self):
        return self.settle_depth_m > 0.0 and self.settle_speed_mps > 0.0

    @property
    def active(self):
        return self.state in (
            "FIRST_CONTACT_CANDIDATE", "TOUCHDOWN_CONFIRMED",
            "FALSE_CONTACT_RECOVERY",
        )

    def update_contact(self, sim_time, trajectory_time, right_fz,
                       contact_sequence):
        if contact_sequence == self.last_contact_sequence:
            return
        self.last_contact_sequence = contact_sequence
        if trajectory_time + 1e-12 < self.arm_trajectory_time:
            return
        if math.isfinite(right_fz) and right_fz < self.threshold_n:
            self.unloaded_once = True
            if self.state == "FIRST_CONTACT_CANDIDATE":
                self.state = "FALSE_CONTACT_RECOVERY"
                self.candidate_started_at = None
                self.false_contact_count += 1
            return
        if not self.unloaded_once or not math.isfinite(right_fz):
            return
        if right_fz < self.threshold_n:
            return
        if self.state in ("SWING", "FALSE_CONTACT_RECOVERY"):
            if self.previous_successful_world_z is None:
                return
            self.state = "FIRST_CONTACT_CANDIDATE"
            self.candidate_started_at = sim_time
            self.first_crossing_time = trajectory_time
            self.hold_world_z = self.previous_successful_world_z
            return
        if (
            self.state == "FIRST_CONTACT_CANDIDATE"
            and sim_time - self.candidate_started_at + 1e-12
            >= self.confirm_duration
        ):
            # At confirmed touchdown, re-latch the last successfully
            # commanded RIGHT world-Z.  This keeps the support foot at
            # a continuous world-frame height instead of following later
            # base motion upward.
            if self.previous_successful_world_z is not None:
                self.hold_world_z = self.previous_successful_world_z
            self.settle_floor_world_z = (
                self.hold_world_z - self.settle_depth_m
                if self.settle_enabled and self.hold_world_z is not None
                else None
            )
            self.state = "TOUCHDOWN_CONFIRMED"
            self.confirmed_time = trajectory_time

    def propose(self, air_world_z, trajectory_time):
        self.pending_recovery_hold = None
        self.pending_recovery_complete = False
        self.pending_settle_hold = None
        if self.state == "FIRST_CONTACT_CANDIDATE":
            # Before touchdown is confirmed, only block additional
            # downward penetration.
            target = max(air_world_z, self.hold_world_z)
        elif self.state == "TOUCHDOWN_CONFIRMED":
            # Once touchdown is confirmed, keep RIGHT sole world-Z
            # latched instead of allowing the moving base frame to lift
            # the support foot back off the floor.
            if (
                self.settle_enabled
                and self.hold_world_z is not None
                and self.settle_floor_world_z is not None
            ):
                max_delta = self.settle_speed_mps * self.dt
                proposed_hold = max(
                    self.settle_floor_world_z,
                    self.hold_world_z - max_delta,
                )
                target = proposed_hold
                self.pending_settle_hold = proposed_hold
            else:
                target = (
                    air_world_z
                    if self.hold_world_z is None
                    else self.hold_world_z
                )
        elif self.state == "FALSE_CONTACT_RECOVERY":
            max_delta = self.recovery_speed_mps * self.dt
            proposed_hold = max(
                self.hold_world_z - max_delta,
                min(self.hold_world_z + max_delta, air_world_z),
            )
            target = max(air_world_z, proposed_hold)
            self.pending_recovery_hold = proposed_hold
            self.pending_recovery_complete = (
                abs(target - air_world_z) <= CORRECTION_RELEASE_TOLERANCE_M
            )
        else:
            target = air_world_z
        original_target = target
        if (
            self.post_settle_release_s is not None
            and not math.isfinite(self.post_settle_release_start_time)
            and self.state == "TOUCHDOWN_CONFIRMED"
            and self.settle_enabled
            and self.hold_world_z is not None
            and self.settle_floor_world_z is not None
            and self.hold_world_z - self.settle_floor_world_z <= 1e-9
        ):
            self.post_settle_release_start_time = trajectory_time
        release_beta = 0.0
        if math.isfinite(self.post_settle_release_start_time):
            release_beta = smoothstep(
                (trajectory_time - self.post_settle_release_start_time)
                / self.post_settle_release_s
            )
        (
            target,
            original_blocked,
            blocked_after_release,
        ) = release_touchdown_downward_block(
            air_world_z, original_target, release_beta
        )
        if self.post_settle_release_s is None:
            release_state = "DISABLED"
        elif not math.isfinite(self.post_settle_release_start_time):
            release_state = "WAITING_FOR_SETTLE"
        elif release_beta >= 1.0:
            release_state = "RELEASED"
        else:
            release_state = "RELEASING"
        return {
            "state": self.state,
            "active": self.active,
            "air_world_z": air_world_z,
            "target_world_z": target,
            "blocked": blocked_after_release,
            "hold_world_z": (
                math.nan if self.hold_world_z is None else self.hold_world_z
            ),
            "post_settle_release_enabled": (
                self.post_settle_release_s is not None
            ),
            "post_settle_release_state": release_state,
            "post_settle_release_start_time": (
                self.post_settle_release_start_time
            ),
            "post_settle_release_beta": release_beta,
            "blocked_before_release": original_blocked,
            "blocked_after_release": blocked_after_release,
            "target_before_release_world_z": original_target,
            "target_after_release_world_z": target,
        }

    def commit_successful_target(self, target_world_z):
        self.previous_successful_world_z = target_world_z
        if (
            self.state == "TOUCHDOWN_CONFIRMED"
            and self.pending_settle_hold is not None
        ):
            self.hold_world_z = self.pending_settle_hold
        if self.state == "FALSE_CONTACT_RECOVERY":
            if self.pending_recovery_hold is not None:
                self.hold_world_z = self.pending_recovery_hold
            if self.pending_recovery_complete:
                self.state = "SWING"
                self.hold_world_z = None
        self.pending_recovery_hold = None
        self.pending_recovery_complete = False
        self.pending_settle_hold = None

    def diagnostics(self, right_fz, solve_result):
        target = (
            None if solve_result is None
            else solve_result.get("touchdown_z_arrest")
        ) or {}
        return {
            "state": self.state,
            "active": self.active,
            "right_fz": right_fz,
            "unloaded_once": self.unloaded_once,
            "first_crossing_time": self.first_crossing_time,
            "confirmed_time": self.confirmed_time,
            "hold_world_z": (
                math.nan if self.hold_world_z is None else self.hold_world_z
            ),
            "settle_enabled": self.settle_enabled,
            "settle_floor_world_z": (
                math.nan
                if self.settle_floor_world_z is None
                else self.settle_floor_world_z
            ),
            "settle_applied_m": (
                0.0
                if self.settle_floor_world_z is None
                or self.hold_world_z is None
                else max(
                    0.0,
                    self.settle_floor_world_z + self.settle_depth_m
                    - self.hold_world_z,
                )
            ),
            "air_world_z": target.get("air_world_z", math.nan),
            "target_world_z": target.get("target_world_z", math.nan),
            "blocked": target.get("blocked", 0.0),
            "false_contact_count": self.false_contact_count,
            "post_settle_release_enabled": target.get(
                "post_settle_release_enabled",
                self.post_settle_release_s is not None,
            ),
            "post_settle_release_state": target.get(
                "post_settle_release_state",
                "DISABLED"
                if self.post_settle_release_s is None
                else "WAITING_FOR_SETTLE",
            ),
            "post_settle_release_start_time": target.get(
                "post_settle_release_start_time",
                self.post_settle_release_start_time,
            ),
            "post_settle_release_beta": target.get(
                "post_settle_release_beta", 0.0
            ),
            "blocked_before_release": target.get(
                "blocked_before_release", 0.0
            ),
            "blocked_after_release": target.get(
                "blocked_after_release", 0.0
            ),
            "target_before_release_world_z": target.get(
                "target_before_release_world_z", math.nan
            ),
            "target_after_release_world_z": target.get(
                "target_after_release_world_z", math.nan
            ),
        }


class RightSupportHold:
    """Latch measured touchdown pose and keep it fixed in the world frame."""

    def __init__(self, chain, fz_threshold, confirm_duration, arm_trajectory_time,
                 left_ramp_out_duration):
        self.chain = chain
        self.fz_threshold = fz_threshold
        self.confirm_duration = confirm_duration
        self.arm_trajectory_time = arm_trajectory_time
        self.left_ramp_out_duration = left_ramp_out_duration
        self.state = "SWING"
        self.above_threshold_since = None
        self.contact_released_after_swing_start = False
        self.last_contact_sequence = None
        self.touchdown_sim_time = math.nan
        self.touchdown_trajectory_time = math.nan
        self.touchdown_right_fz = math.nan
        self.world_pose = None
        self.left_offset_at_touchdown = 0.0
        self.previous_successful_q = None
        self.warned_missing_joints = False

    @property
    def active(self):
        return self.state != "SWING"

    def update(self, sim_time, trajectory_time, right_fz, contact_sequence,
               measured_q,
               base_rotation, base_position, left_lateral_offset):
        if self.active:
            return False
        # Only new contact messages advance the simulation-time debounce. This
        # prevents one retained high-force sample from confirming touchdown.
        if contact_sequence == self.last_contact_sequence:
            return False
        self.last_contact_sequence = contact_sequence
        if trajectory_time + 1e-12 < self.arm_trajectory_time:
            self.above_threshold_since = None
            return False
        if not self.contact_released_after_swing_start:
            if math.isfinite(right_fz) and right_fz < self.fz_threshold:
                self.contact_released_after_swing_start = True
            return False
        if not math.isfinite(right_fz) or right_fz < self.fz_threshold:
            self.above_threshold_since = None
            return False
        if self.above_threshold_since is None:
            self.above_threshold_since = sim_time
        if sim_time - self.above_threshold_since + 1e-12 < self.confirm_duration:
            return False
        if not np.all(np.isfinite(measured_q)):
            if not self.warned_missing_joints:
                print(
                    "[WARNING] touchdown force confirmed but measured right "
                    "joint positions are incomplete; remaining in SWING"
                )
                self.warned_missing_joints = True
            return False

        base_pose = self.chain.forward(measured_q)
        self.world_pose = np.eye(4)
        self.world_pose[:3, :3] = base_rotation @ base_pose[:3, :3]
        self.world_pose[:3, 3] = (
            base_position + base_rotation @ base_pose[:3, 3]
        )
        self.touchdown_sim_time = sim_time
        self.touchdown_trajectory_time = trajectory_time
        self.touchdown_right_fz = right_fz
        self.left_offset_at_touchdown = left_lateral_offset
        self.previous_successful_q = measured_q.copy()
        self.state = "TOUCHDOWN_CONFIRMED"
        return True

    def left_handoff(self, sim_time):
        if not self.active:
            return 1.0, math.nan, math.nan
        if self.left_ramp_out_duration <= 0.0:
            beta = 0.0
        else:
            elapsed = max(0.0, sim_time - self.touchdown_sim_time)
            beta = 1.0 - smoothstep(elapsed / self.left_ramp_out_duration)
        return (
            beta,
            self.left_offset_at_touchdown,
            beta * self.left_offset_at_touchdown,
        )

    def solve(self, base_rotation, base_position, measured_q, fallback_q):
        target = np.eye(4)
        target[:3, :3] = base_rotation.T @ self.world_pose[:3, :3]
        target[:3, 3] = base_rotation.T @ (
            self.world_pose[:3, 3] - base_position
        )
        seed = (
            self.previous_successful_q
            if self.previous_successful_q is not None else fallback_q
        )
        try:
            solve_result = solve_leg_frame(self.chain, target, seed)
            candidate = solve_result.x
            valid = (
                solve_result.success
                and np.all(np.isfinite(candidate))
                and np.all(candidate >= self.chain.lower - 1e-10)
                and np.all(candidate <= self.chain.upper + 1e-10)
            )
        except Exception as error:
            print(f"[WARNING] right support-hold IK exception: {error}")
            candidate = fallback_q
            valid = False
        if valid:
            solved_q = candidate
            self.previous_successful_q = candidate.copy()
        else:
            # Preserve the last valid support solution; before one exists,
            # retain the last safely published right-leg command.
            solved_q = (
                self.previous_successful_q.copy()
                if self.previous_successful_q is not None else fallback_q.copy()
            )
            print(
                "[WARNING] right support-hold IK failed; retaining previous "
                "successful support command"
            )

        solved_pose = self.chain.forward(solved_q)
        solved_world_position = (
            base_position + base_rotation @ solved_pose[:3, 3]
        )
        solved_world_rotation = base_rotation @ solved_pose[:3, :3]
        actual_world_position = np.full(3, math.nan)
        actual_world_rotation = None
        if np.all(np.isfinite(measured_q)):
            actual_pose = self.chain.forward(measured_q)
            actual_world_position = (
                base_position + base_rotation @ actual_pose[:3, 3]
            )
            actual_world_rotation = base_rotation @ actual_pose[:3, :3]
        diagnostic_position = (
            actual_world_position
            if np.all(np.isfinite(actual_world_position))
            else solved_world_position
        )
        diagnostic_rotation = (
            actual_world_rotation
            if actual_world_rotation is not None else solved_world_rotation
        )
        return {
            "q": solved_q.copy(),
            "success": bool(valid),
            "target_base_pose": target,
            "target_world_position": self.world_pose[:3, 3].copy(),
            "actual_or_fk_world_position": diagnostic_position,
            "world_pos_error_mm": 1000.0 * np.linalg.norm(
                diagnostic_position - self.world_pose[:3, 3]
            ),
            "world_ori_error_deg": math.degrees(rotation_error_angle(
                self.world_pose[:3, :3], diagnostic_rotation,
            )),
        }

    def finish_confirmation_frame(self):
        if self.state == "TOUCHDOWN_CONFIRMED":
            self.state = "RIGHT_SUPPORT_HOLD"


def command_tuples(targets):
    return [
        (key, topic, targets[key], targets[key])
        for key, topic in LEG_JOINT_TOPICS.items()
    ]


def publish_targets(publishers, targets):
    publishers.publish(command_tuples(targets))


def log_columns():
    return (
        ["publish_index", "simulation_time", "trajectory_time", "source_frame", "phase"]
        + [f"RL{i}" for i in range(6)]
        + [f"LL{i}" for i in range(6)]
        + [
            "base_roll", "base_pitch", "base_yaw",
            "world_flat_active", "world_flat_solver_success",
            "swing_world_z_active", "swing_world_z_solver_success",
            "swing_world_z_correction_release_limiter_active",
            "swing_world_z_correction_release_state",
            "right_world_z_raw_correction_m",
            "right_world_z_desired_correction_m",
            "right_world_z_previous_correction_m",
            "right_world_z_output_correction_m",
            "right_world_z_correction_delta_m",
            "right_world_z_correction_velocity_mps",
            "right_world_z_correction_release_limited",
            "right_world_z_correction_release_limited_frame_count",
            "touchdown_z_arrest_state", "touchdown_z_arrest_active",
            "touchdown_z_arrest_right_contact_fz",
            "touchdown_z_arrest_unloaded_once",
            "touchdown_z_arrest_first_crossing_time",
            "touchdown_z_arrest_confirmed_time",
            "touchdown_z_arrest_hold_world_z_m",
            "touchdown_z_arrest_settle_enabled",
            "touchdown_z_arrest_settle_floor_world_z_m",
            "touchdown_z_arrest_settle_applied_m",
            "touchdown_z_arrest_post_settle_release_enabled",
            "touchdown_z_arrest_post_settle_release_state",
            "touchdown_z_arrest_post_settle_release_start_time",
            "touchdown_z_arrest_post_settle_release_beta",
            "touchdown_z_arrest_blocked_before_release_m",
            "touchdown_z_arrest_blocked_after_release_m",
            "touchdown_z_arrest_target_before_release_world_z_m",
            "touchdown_z_arrest_target_after_release_world_z_m",
            "right_air_world_z_desired_m",
            "right_touchdown_world_z_target_m",
            "right_world_z_downward_blocked_m",
            "touchdown_z_arrest_false_contact_count",
            "left_support_z_hold_enabled", "left_support_z_hold_active",
            "left_support_z_hold_world_z",
            "left_support_z_release_active", "left_support_z_release_beta",
            "left_support_z_nominal_world_z", "left_support_z_final_world_z",
            "left_support_z_hold_start_time",
            "left_support_z_release_start_time",
            "left_support_impact_anchor_enabled",
            "left_support_impact_anchor_state",
            "left_support_impact_anchor_beta",
            "left_support_impact_anchor_world_x_m",
            "left_support_impact_target_world_x_before_m",
            "left_support_impact_target_world_x_after_m",
            "left_support_impact_anchor_world_yaw_deg",
            "left_support_impact_target_world_yaw_before_deg",
            "left_support_impact_target_world_yaw_after_deg",
            "left_support_impact_anchor_start_trajectory_time",
            "right_nominal_pos_err_mm",
            "right_world_ori_err_before_deg", "right_world_ori_err_after_deg",
            "right_ik_pos_err_mm", "right_ik_ori_err_deg",
            "desired_base_relative_roll", "desired_base_relative_pitch",
            "desired_base_relative_yaw", "max_joint_delta_deg",
            "max_joint_velocity_deg_s",
            "right_nominal_relative_world_z_mm",
            "right_desired_relative_world_z_mm",
            "right_predicted_relative_world_z_before_mm",
            "right_predicted_relative_world_z_after_mm",
            "right_world_z_compensation_mm", "right_target_base_z_mm",
            "tilt_term_r20_dx_mm", "tilt_term_r21_dy_mm",
            "feedback_rate_limit_active",
            "feedback_rate_limited_joint_count",
            "feedback_max_requested_speed_deg_s",
            "feedback_max_published_speed_deg_s",
            "left_world_ori_err_before_deg", "left_world_ori_err_after_deg",
            "left_world_flat_solver_success",
            "right_world_flat_solver_success",
            "left_feedback_rate_limited_joint_count",
            "right_feedback_rate_limited_joint_count",
            "left_feedback_max_requested_speed_deg_s",
            "right_feedback_max_requested_speed_deg_s",
            "left_feedback_max_published_speed_deg_s",
            "right_feedback_max_published_speed_deg_s",
            "lateral_balance_active", "planner_lateral_dx_m",
            "planner_lateral_vx_mps", "base_dx_actual_m",
            "base_vx_actual_mps", "lateral_position_error_m",
            "lateral_velocity_error_mps",
            "lateral_support_x_raw_offset_m",
            "lateral_support_x_applied_offset_m", "lateral_balance_beta",
            "lateral_balance_support_handoff_enabled",
            "lateral_balance_support_handoff_beta",
            "lateral_balance_support_state",
            "left_lateral_support_x_applied_offset_m",
            "right_lateral_support_x_applied_offset_m",
            "left_lateral_solver_success", "left_lateral_target_x_m",
            "left_lateral_ik_pos_err_mm",
            "right_lateral_solver_success", "right_lateral_target_x_m",
            "touchdown_state", "touchdown_detected", "touchdown_right_fz",
            "touchdown_sim_time", "touchdown_trajectory_time",
            "right_support_hold_active",
            "right_support_target_world_x", "right_support_target_world_y",
            "right_support_target_world_z",
            "right_support_actual_or_fk_world_x",
            "right_support_actual_or_fk_world_y",
            "right_support_actual_or_fk_world_z",
            "right_support_world_pos_error_mm",
            "right_support_world_ori_error_deg",
            "left_touchdown_handoff_beta",
            "left_lateral_offset_before_handoff_m",
            "left_lateral_offset_after_handoff_m",
            "right_support_solver_success",
            "pitch_balance_active", "base_pitch_reference_rad",
            "base_pitch_actual_rad", "base_pitch_error_rad",
            "base_pitch_rate_actual_rad_s", "base_pitch_rate_error_rad_s",
            "pitch_balance_raw_correction_rad",
            "pitch_balance_applied_correction_rad", "pitch_balance_beta",
            "left_pitch_balance_solver_success",
            "left_nominal_base_pitch_deg", "left_target_base_pitch_deg",
            "left_pitch_target_delta_deg",
            "pitch_balance_support_handoff_enabled",
            "pitch_balance_support_handoff_beta",
            "pitch_balance_support_state",
            "left_pitch_balance_handoff_applied_deg",
            "right_pitch_balance_handoff_raw_deg",
            "right_pitch_balance_handoff_target_deg",
            "right_pitch_balance_handoff_applied_deg",
            "right_pitch_balance_handoff_rate_limited",
            "right_pitch_balance_handoff_rate_limit_deg_s",
            "right_pitch_balance_support_sign",
            "right_pitch_balance_solver_success",
            "fore_aft_sign_test_enabled",
            "fore_aft_sign_test_support_confirmed",
            "fore_aft_sign_test_requested_deg",
            "fore_aft_sign_test_beta",
            "fore_aft_sign_test_applied_deg",
            "right_target_world_x_rotation_before_deg",
            "right_target_world_x_rotation_after_deg",
            "right_touchdown_flatten_enabled",
            "right_touchdown_flatten_requested_deg",
            "right_touchdown_flatten_beta",
            "right_touchdown_flatten_applied_deg",
            "right_touchdown_flatten_state",
            "right_touchdown_flatten_release_beta",
            "right_target_world_roll_before_flatten_deg",
            "right_target_world_roll_after_flatten_deg",
        ]
    )


def finite_text(value):
    return f"{value:+.12f}" if math.isfinite(value) else "nan"


def write_log(writer, publish_index, simulation_time, trajectory_time,
              source_frame, phase, targets, base_rpy, world_flat_active,
              swing_world_z_active, solve_result,
              max_delta_deg, max_velocity_deg_s, rate_limit_diagnostics=None,
              left_solve_result=None, left_rate_diagnostics=None,
              lateral_diagnostics=None, touchdown_diagnostics=None,
              pitch_diagnostics=None, touchdown_z_arrest_diagnostics=None,
              fore_aft_sign_test_diagnostics=None,
              left_support_z_diagnostics=None,
              touchdown_flatten_diagnostics_row=None,
              pitch_handoff_diagnostics=None,
              left_support_impact_anchor_diagnostics=None):
    desired_rpy = (
        np.full(3, math.nan) if solve_result is None
        else solve_result["desired_rpy"]
    )
    row = {
        "publish_index": publish_index,
        "simulation_time": finite_text(simulation_time),
        "trajectory_time": f"{trajectory_time:.9f}",
        "source_frame": source_frame,
        "phase": phase,
        "base_roll": finite_text(base_rpy[0]),
        "base_pitch": finite_text(base_rpy[1]),
        "base_yaw": finite_text(base_rpy[2]),
        "world_flat_active": int(world_flat_active),
        "world_flat_solver_success": int(
            solve_result is not None and solve_result["world_flat_success"]
        ),
        "swing_world_z_active": int(swing_world_z_active),
        "swing_world_z_solver_success": int(
            solve_result is not None and solve_result["swing_world_z_success"]
        ),
        "right_nominal_pos_err_mm": finite_text(
            math.nan if solve_result is None
            else solve_result["nominal_pos_err_mm"]
        ),
        "right_world_ori_err_before_deg": finite_text(
            math.nan if solve_result is None
            else solve_result["world_ori_before_deg"]
        ),
        "right_world_ori_err_after_deg": finite_text(
            math.nan if solve_result is None
            else solve_result["world_ori_after_deg"]
        ),
        "right_ik_pos_err_mm": finite_text(
            math.nan if solve_result is None else solve_result["ik_pos_err_mm"]
        ),
        "right_ik_ori_err_deg": finite_text(
            math.nan if solve_result is None else solve_result["ik_ori_err_deg"]
        ),
        "desired_base_relative_roll": finite_text(desired_rpy[0]),
        "desired_base_relative_pitch": finite_text(desired_rpy[1]),
        "desired_base_relative_yaw": finite_text(desired_rpy[2]),
        "max_joint_delta_deg": finite_text(max_delta_deg),
        "max_joint_velocity_deg_s": finite_text(max_velocity_deg_s),
    }
    correction_release = (
        None if solve_result is None
        else solve_result.get("correction_release")
    ) or {
        "active": False,
        "state": "NORMAL",
        "raw": math.nan,
        "desired": math.nan,
        "previous": math.nan,
        "output": math.nan,
        "delta": math.nan,
        "velocity": math.nan,
        "limited": False,
        "limited_frame_count": 0,
    }
    row.update({
        "swing_world_z_correction_release_limiter_active": int(
            correction_release["active"]
        ),
        "swing_world_z_correction_release_state": correction_release["state"],
        "right_world_z_raw_correction_m": finite_text(
            correction_release["raw"]
        ),
        "right_world_z_desired_correction_m": finite_text(
            correction_release["desired"]
        ),
        "right_world_z_previous_correction_m": finite_text(
            correction_release["previous"]
        ),
        "right_world_z_output_correction_m": finite_text(
            correction_release["output"]
        ),
        "right_world_z_correction_delta_m": finite_text(
            correction_release["delta"]
        ),
        "right_world_z_correction_velocity_mps": finite_text(
            correction_release["velocity"]
        ),
        "right_world_z_correction_release_limited": int(
            correction_release["limited"]
        ),
        "right_world_z_correction_release_limited_frame_count": (
            correction_release["limited_frame_count"]
        ),
    })
    arrest = touchdown_z_arrest_diagnostics or {
        "state": "DISABLED", "active": False, "right_fz": math.nan,
        "unloaded_once": False, "first_crossing_time": math.nan,
        "confirmed_time": math.nan, "hold_world_z": math.nan,
        "settle_enabled": False, "settle_floor_world_z": math.nan,
        "settle_applied_m": 0.0,
        "air_world_z": math.nan, "target_world_z": math.nan,
        "blocked": 0.0, "false_contact_count": 0,
        "post_settle_release_enabled": False,
        "post_settle_release_state": "DISABLED",
        "post_settle_release_start_time": math.nan,
        "post_settle_release_beta": 0.0,
        "blocked_before_release": 0.0,
        "blocked_after_release": 0.0,
        "target_before_release_world_z": math.nan,
        "target_after_release_world_z": math.nan,
    }
    row.update({
        "touchdown_z_arrest_state": arrest["state"],
        "touchdown_z_arrest_active": int(arrest["active"]),
        "touchdown_z_arrest_right_contact_fz": finite_text(arrest["right_fz"]),
        "touchdown_z_arrest_unloaded_once": int(arrest["unloaded_once"]),
        "touchdown_z_arrest_first_crossing_time": finite_text(
            arrest["first_crossing_time"]
        ),
        "touchdown_z_arrest_confirmed_time": finite_text(
            arrest["confirmed_time"]
        ),
        "touchdown_z_arrest_hold_world_z_m": finite_text(
            arrest["hold_world_z"]
        ),
        "touchdown_z_arrest_settle_enabled": int(arrest["settle_enabled"]),
        "touchdown_z_arrest_settle_floor_world_z_m": finite_text(
            arrest["settle_floor_world_z"]
        ),
        "touchdown_z_arrest_settle_applied_m": finite_text(
            arrest["settle_applied_m"]
        ),
        "touchdown_z_arrest_post_settle_release_enabled": int(
            arrest["post_settle_release_enabled"]
        ),
        "touchdown_z_arrest_post_settle_release_state": (
            arrest["post_settle_release_state"]
        ),
        "touchdown_z_arrest_post_settle_release_start_time": finite_text(
            arrest["post_settle_release_start_time"]
        ),
        "touchdown_z_arrest_post_settle_release_beta": finite_text(
            arrest["post_settle_release_beta"]
        ),
        "touchdown_z_arrest_blocked_before_release_m": finite_text(
            arrest["blocked_before_release"]
        ),
        "touchdown_z_arrest_blocked_after_release_m": finite_text(
            arrest["blocked_after_release"]
        ),
        "touchdown_z_arrest_target_before_release_world_z_m": finite_text(
            arrest["target_before_release_world_z"]
        ),
        "touchdown_z_arrest_target_after_release_world_z_m": finite_text(
            arrest["target_after_release_world_z"]
        ),
        "right_air_world_z_desired_m": finite_text(arrest["air_world_z"]),
        "right_touchdown_world_z_target_m": finite_text(
            arrest["target_world_z"]
        ),
        "right_world_z_downward_blocked_m": finite_text(arrest["blocked"]),
        "touchdown_z_arrest_false_contact_count": arrest["false_contact_count"],
    })
    support_z = left_support_z_diagnostics or {
        "enabled": False, "active": False, "hold_world_z": math.nan,
        "release_active": False, "release_beta": 0.0,
        "nominal_world_z": math.nan, "final_world_z": math.nan,
        "hold_start_time": math.nan, "release_start_time": math.nan,
    }
    row.update({
        "left_support_z_hold_enabled": int(support_z["enabled"]),
        "left_support_z_hold_active": int(support_z["active"]),
        "left_support_z_hold_world_z": finite_text(support_z["hold_world_z"]),
        "left_support_z_release_active": int(support_z["release_active"]),
        "left_support_z_release_beta": finite_text(support_z["release_beta"]),
        "left_support_z_nominal_world_z": finite_text(
            support_z["nominal_world_z"]
        ),
        "left_support_z_final_world_z": finite_text(support_z["final_world_z"]),
        "left_support_z_hold_start_time": finite_text(
            support_z["hold_start_time"]
        ),
        "left_support_z_release_start_time": finite_text(
            support_z["release_start_time"]
        ),
    })
    rate_limit_diagnostics = rate_limit_diagnostics or {
        "active": False,
        "limited_joint_count": 0,
        "requested_speed_deg_s": 0.0,
        "published_speed_deg_s": 0.0,
    }
    row.update({
        "feedback_rate_limit_active": int(rate_limit_diagnostics["active"]),
        "feedback_rate_limited_joint_count": (
            rate_limit_diagnostics["limited_joint_count"]
        ),
        "feedback_max_requested_speed_deg_s": finite_text(
            rate_limit_diagnostics["requested_speed_deg_s"]
        ),
        "feedback_max_published_speed_deg_s": finite_text(
            rate_limit_diagnostics["published_speed_deg_s"]
        ),
    })
    lateral_diagnostics = lateral_diagnostics or {
        "active": False,
        "planner_dx": math.nan, "planner_vx": math.nan,
        "base_dx": math.nan, "base_vx": math.nan,
        "position_error": math.nan, "velocity_error": math.nan,
        "raw_offset": math.nan, "applied_offset": math.nan,
        "beta": 0.0, "handoff_enabled": False,
        "handoff_beta": 0.0, "handoff_state": "DISABLED",
        "left_applied_offset": math.nan, "right_applied_offset": 0.0,
    }
    row.update({
        "lateral_balance_active": int(lateral_diagnostics["active"]),
        "planner_lateral_dx_m": finite_text(lateral_diagnostics["planner_dx"]),
        "planner_lateral_vx_mps": finite_text(lateral_diagnostics["planner_vx"]),
        "base_dx_actual_m": finite_text(lateral_diagnostics["base_dx"]),
        "base_vx_actual_mps": finite_text(lateral_diagnostics["base_vx"]),
        "lateral_position_error_m": finite_text(
            lateral_diagnostics["position_error"]
        ),
        "lateral_velocity_error_mps": finite_text(
            lateral_diagnostics["velocity_error"]
        ),
        "lateral_support_x_raw_offset_m": finite_text(
            lateral_diagnostics["raw_offset"]
        ),
        "lateral_support_x_applied_offset_m": finite_text(
            lateral_diagnostics["applied_offset"]
        ),
        "lateral_balance_beta": finite_text(lateral_diagnostics["beta"]),
        "lateral_balance_support_handoff_enabled": int(
            lateral_diagnostics["handoff_enabled"]
        ),
        "lateral_balance_support_handoff_beta": finite_text(
            lateral_diagnostics["handoff_beta"]
        ),
        "lateral_balance_support_state": lateral_diagnostics["handoff_state"],
        "left_lateral_support_x_applied_offset_m": finite_text(
            lateral_diagnostics["left_applied_offset"]
        ),
        "right_lateral_support_x_applied_offset_m": finite_text(
            lateral_diagnostics["right_applied_offset"]
        ),
        "left_lateral_solver_success": int(
            left_solve_result is not None
            and left_solve_result.get("lateral_balance_success", False)
        ),
        "left_lateral_target_x_m": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["lateral_target_x"]
        ),
        "left_lateral_ik_pos_err_mm": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["ik_pos_err_mm"]
        ),
        "right_lateral_solver_success": int(
            solve_result is not None
            and solve_result.get("lateral_balance_success", False)
        ),
        "right_lateral_target_x_m": finite_text(
            math.nan if solve_result is None
            else solve_result["lateral_target_x"]
        ),
    })
    touchdown_diagnostics = touchdown_diagnostics or {
        "state": "DISABLED", "detected": False, "right_fz": math.nan,
        "sim_time": math.nan, "trajectory_time": math.nan,
        "support_active": False, "support_result": None,
        "left_handoff_beta": 1.0,
        "left_offset_before": math.nan, "left_offset_after": math.nan,
    }
    support_result = touchdown_diagnostics["support_result"]
    target_world = (
        np.full(3, math.nan) if support_result is None
        else support_result["target_world_position"]
    )
    actual_world = (
        np.full(3, math.nan) if support_result is None
        else support_result["actual_or_fk_world_position"]
    )
    row.update({
        "touchdown_state": touchdown_diagnostics["state"],
        "touchdown_detected": int(touchdown_diagnostics["detected"]),
        "touchdown_right_fz": finite_text(touchdown_diagnostics["right_fz"]),
        "touchdown_sim_time": finite_text(touchdown_diagnostics["sim_time"]),
        "touchdown_trajectory_time": finite_text(
            touchdown_diagnostics["trajectory_time"]
        ),
        "right_support_hold_active": int(
            touchdown_diagnostics["support_active"]
        ),
        "right_support_target_world_x": finite_text(target_world[0]),
        "right_support_target_world_y": finite_text(target_world[1]),
        "right_support_target_world_z": finite_text(target_world[2]),
        "right_support_actual_or_fk_world_x": finite_text(actual_world[0]),
        "right_support_actual_or_fk_world_y": finite_text(actual_world[1]),
        "right_support_actual_or_fk_world_z": finite_text(actual_world[2]),
        "right_support_world_pos_error_mm": finite_text(
            math.nan if support_result is None
            else support_result["world_pos_error_mm"]
        ),
        "right_support_world_ori_error_deg": finite_text(
            math.nan if support_result is None
            else support_result["world_ori_error_deg"]
        ),
        "left_touchdown_handoff_beta": finite_text(
            touchdown_diagnostics["left_handoff_beta"]
        ),
        "left_lateral_offset_before_handoff_m": finite_text(
            touchdown_diagnostics["left_offset_before"]
        ),
        "left_lateral_offset_after_handoff_m": finite_text(
            touchdown_diagnostics["left_offset_after"]
        ),
        "right_support_solver_success": int(
            support_result is not None and support_result["success"]
        ),
    })
    pitch_diagnostics = pitch_diagnostics or {
        "active": False, "reference": math.nan, "actual": math.nan,
        "error": math.nan, "rate": math.nan, "rate_error": math.nan,
        "raw_correction": math.nan, "applied_correction": math.nan,
        "beta": 0.0,
    }
    row.update({
        "pitch_balance_active": int(pitch_diagnostics["active"]),
        "base_pitch_reference_rad": finite_text(pitch_diagnostics["reference"]),
        "base_pitch_actual_rad": finite_text(pitch_diagnostics["actual"]),
        "base_pitch_error_rad": finite_text(pitch_diagnostics["error"]),
        "base_pitch_rate_actual_rad_s": finite_text(pitch_diagnostics["rate"]),
        "base_pitch_rate_error_rad_s": finite_text(
            pitch_diagnostics["rate_error"]
        ),
        "pitch_balance_raw_correction_rad": finite_text(
            pitch_diagnostics["raw_correction"]
        ),
        "pitch_balance_applied_correction_rad": finite_text(
            pitch_diagnostics["applied_correction"]
        ),
        "pitch_balance_beta": finite_text(pitch_diagnostics["beta"]),
        "left_pitch_balance_solver_success": int(
            left_solve_result is not None
            and left_solve_result.get("pitch_balance_success", False)
        ),
        "left_nominal_base_pitch_deg": finite_text(
            math.nan if left_solve_result is None
            else math.degrees(left_solve_result.get("nominal_base_pitch", math.nan))
        ),
        "left_target_base_pitch_deg": finite_text(
            math.nan if left_solve_result is None
            else math.degrees(left_solve_result.get("target_base_pitch", math.nan))
        ),
        "left_pitch_target_delta_deg": finite_text(
            math.nan if left_solve_result is None
            else math.degrees(left_solve_result.get("pitch_target_delta", math.nan))
        ),
    })
    pitch_handoff = pitch_handoff_diagnostics or {
        "enabled": False, "beta": 0.0, "state": "DISABLED",
        "left_applied": 0.0, "right_raw": 0.0, "right_target": 0.0,
        "right_applied": 0.0, "right_rate_limited": False,
        "right_rate_limit_deg_s": None, "right_sign": 1,
    }
    row.update({
        "pitch_balance_support_handoff_enabled": int(
            pitch_handoff["enabled"]
        ),
        "pitch_balance_support_handoff_beta": finite_text(
            pitch_handoff["beta"]
        ),
        "pitch_balance_support_state": pitch_handoff["state"],
        "left_pitch_balance_handoff_applied_deg": finite_text(
            math.degrees(pitch_handoff["left_applied"])
        ),
        "right_pitch_balance_handoff_raw_deg": finite_text(
            math.degrees(pitch_handoff["right_raw"])
        ),
        "right_pitch_balance_handoff_target_deg": finite_text(
            math.degrees(pitch_handoff["right_target"])
        ),
        "right_pitch_balance_handoff_applied_deg": finite_text(
            math.degrees(pitch_handoff["right_applied"])
        ),
        "right_pitch_balance_handoff_rate_limited": int(
            pitch_handoff["right_rate_limited"]
        ),
        "right_pitch_balance_handoff_rate_limit_deg_s": finite_text(
            math.nan
            if pitch_handoff["right_rate_limit_deg_s"] is None
            else pitch_handoff["right_rate_limit_deg_s"]
        ),
        "right_pitch_balance_support_sign": pitch_handoff["right_sign"],
        "right_pitch_balance_solver_success": int(
            solve_result is not None
            and solve_result.get("right_pitch_balance_success", False)
        ),
    })
    impact_anchor = (
        None if left_solve_result is None
        else left_solve_result.get("left_support_impact_anchor")
    ) or left_support_impact_anchor_diagnostics or {
        "enabled": False, "state": "DISABLED", "beta": 0.0,
        "anchor_world_x": math.nan, "anchor_world_yaw": math.nan,
        "start_trajectory_time": math.nan,
        "target_world_x_before": math.nan,
        "target_world_x_after": math.nan,
        "target_world_yaw_before": math.nan,
        "target_world_yaw_after": math.nan,
    }
    row.update({
        "left_support_impact_anchor_enabled": int(impact_anchor["enabled"]),
        "left_support_impact_anchor_state": impact_anchor["state"],
        "left_support_impact_anchor_beta": finite_text(impact_anchor["beta"]),
        "left_support_impact_anchor_world_x_m": finite_text(
            impact_anchor["anchor_world_x"]
        ),
        "left_support_impact_target_world_x_before_m": finite_text(
            impact_anchor["target_world_x_before"]
        ),
        "left_support_impact_target_world_x_after_m": finite_text(
            impact_anchor["target_world_x_after"]
        ),
        "left_support_impact_anchor_world_yaw_deg": finite_text(
            math.degrees(impact_anchor["anchor_world_yaw"])
        ),
        "left_support_impact_target_world_yaw_before_deg": finite_text(
            math.degrees(impact_anchor["target_world_yaw_before"])
        ),
        "left_support_impact_target_world_yaw_after_deg": finite_text(
            math.degrees(impact_anchor["target_world_yaw_after"])
        ),
        "left_support_impact_anchor_start_trajectory_time": finite_text(
            impact_anchor["start_trajectory_time"]
        ),
    })
    fore_aft = fore_aft_sign_test_diagnostics or {
        "enabled": False, "support_confirmed": False,
        "requested": 0.0, "beta": 0.0, "applied": 0.0,
    }
    before_world_x = math.nan
    after_world_x = math.nan
    if solve_result is not None:
        before_world_x = Rotation.from_matrix(
            solve_result["world_x_orientation_before"]
        ).as_euler("xyz")[0]
        after_world_x = Rotation.from_matrix(
            solve_result["world_x_orientation_after"]
        ).as_euler("xyz")[0]
    row.update({
        "fore_aft_sign_test_enabled": int(fore_aft["enabled"]),
        "fore_aft_sign_test_support_confirmed": int(
            fore_aft["support_confirmed"]
        ),
        "fore_aft_sign_test_requested_deg": finite_text(
            math.degrees(fore_aft["requested"])
        ),
        "fore_aft_sign_test_beta": finite_text(fore_aft["beta"]),
        "fore_aft_sign_test_applied_deg": finite_text(
            math.degrees(fore_aft["applied"])
        ),
        "right_target_world_x_rotation_before_deg": finite_text(
            math.degrees(before_world_x)
        ),
        "right_target_world_x_rotation_after_deg": finite_text(
            math.degrees(after_world_x)
        ),
    })
    flatten = touchdown_flatten_diagnostics_row or {
        "enabled": False, "requested": 0.0, "beta": 0.0,
        "applied": 0.0, "state": "DISABLED", "release_beta": 0.0,
    }
    row.update({
        "right_touchdown_flatten_enabled": int(flatten["enabled"]),
        "right_touchdown_flatten_requested_deg": finite_text(
            math.degrees(flatten["requested"])
        ),
        "right_touchdown_flatten_beta": finite_text(flatten["beta"]),
        "right_touchdown_flatten_applied_deg": finite_text(
            math.degrees(flatten["applied"])
        ),
        "right_touchdown_flatten_state": flatten["state"],
        "right_touchdown_flatten_release_beta": finite_text(
            flatten["release_beta"]
        ),
        "right_target_world_roll_before_flatten_deg": finite_text(
            math.degrees(before_world_x)
        ),
        "right_target_world_roll_after_flatten_deg": finite_text(
            math.degrees(after_world_x)
        ),
    })
    left_rate_diagnostics = left_rate_diagnostics or {
        "limited_joint_count": 0,
        "requested_speed_deg_s": 0.0,
        "published_speed_deg_s": 0.0,
    }
    row.update({
        "left_world_ori_err_before_deg": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["world_ori_before_deg"]
        ),
        "left_world_ori_err_after_deg": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["world_ori_after_deg"]
        ),
        "left_world_flat_solver_success": int(
            left_solve_result is not None
            and left_solve_result["world_flat_success"]
        ),
        "right_world_flat_solver_success": int(
            solve_result is not None and solve_result["world_flat_success"]
        ),
        "left_feedback_rate_limited_joint_count": (
            left_rate_diagnostics["limited_joint_count"]
        ),
        "right_feedback_rate_limited_joint_count": (
            rate_limit_diagnostics["limited_joint_count"]
        ),
        "left_feedback_max_requested_speed_deg_s": finite_text(
            left_rate_diagnostics["requested_speed_deg_s"]
        ),
        "right_feedback_max_requested_speed_deg_s": finite_text(
            rate_limit_diagnostics["requested_speed_deg_s"]
        ),
        "left_feedback_max_published_speed_deg_s": finite_text(
            left_rate_diagnostics["published_speed_deg_s"]
        ),
        "right_feedback_max_published_speed_deg_s": finite_text(
            rate_limit_diagnostics["published_speed_deg_s"]
        ),
    })
    diagnostic_keys = {
        "right_nominal_relative_world_z_mm": "nominal_relative_world_z",
        "right_desired_relative_world_z_mm": "desired_relative_world_z",
        "right_predicted_relative_world_z_before_mm": (
            "predicted_relative_world_z_before"
        ),
        "right_predicted_relative_world_z_after_mm": (
            "predicted_relative_world_z_after"
        ),
        "right_world_z_compensation_mm": "world_z_compensation",
        "right_target_base_z_mm": "target_base_z",
        "tilt_term_r20_dx_mm": "tilt_r20_dx",
        "tilt_term_r21_dy_mm": "tilt_r21_dy",
    }
    for column, key in diagnostic_keys.items():
        row[column] = finite_text(
            math.nan if solve_result is None else 1000.0 * solve_result[key]
        )
    for side in ("RL", "LL"):
        for joint in range(6):
            row[f"{side}{joint}"] = finite_text(targets[f"{side}{joint}_wrap"])
    writer.writerow(row)


class CommandSafetyMonitor:
    def __init__(self):
        self.previous = None
        self.previous_change_time = None

    def check(self, targets, simulation_time):
        current = np.array([targets[key] for key in LEG_JOINT_TOPICS])
        if self.previous is None:
            self.previous = current
            self.previous_change_time = simulation_time
            return 0.0, 0.0
        delta = np.max(np.abs(current - self.previous))
        max_velocity = 0.0
        if delta > 1e-14:
            elapsed = simulation_time - self.previous_change_time
            if elapsed > 0.0:
                max_velocity = delta / elapsed
            self.previous = current
            self.previous_change_time = simulation_time
            if max_velocity > VELOCITY_WARNING_RAD_S:
                print(
                    "[WARNING] command velocity exceeds 270 deg/s: "
                    f"{math.degrees(max_velocity):.3f} deg/s"
                )
        return math.degrees(delta), math.degrees(max_velocity)


def nominal_validation(targets, chains):
    expected = np.concatenate((CANDIDATE_B["right"], CANDIDATE_B["left"]))
    actual = np.concatenate((leg_vector(targets[0], "RL"), leg_vector(targets[0], "LL")))
    frame_zero_exact = np.array_equal(actual, expected)
    poses_finite = True
    for frame in targets:
        poses_finite = poses_finite and np.all(np.isfinite(
            chains["right"].forward(leg_vector(frame, "RL"))
        ))
        poses_finite = poses_finite and np.all(np.isfinite(
            chains["left"].forward(leg_vector(frame, "LL"))
        ))
    return frame_zero_exact, poses_finite


def main():
    args = parse_args()
    targets = load_targets(args.csv)
    right_lateral_reference_frame = None
    right_lateral_reference_x = None
    if args.right_swing_lateral_scale is not None:
        right_lateral_reference_frame = round(
            args.right_swing_lateral_reference_time / args.dt
        )
        if right_lateral_reference_frame >= len(targets):
            raise RuntimeError(
                "RIGHT swing lateral reference frame is outside the CSV"
            )
    planner_dx = None
    planner_vx = None
    if args.lateral_balance_planner_csv is not None:
        planner_dx, planner_vx = load_planner_lateral_reference(
            args.lateral_balance_planner_csv, args.dt,
        )
        if len(planner_dx) < len(targets):
            raise RuntimeError("planner CSV is shorter than nominal command CSV")
    chains = load_leg_chains(args.sdf)
    if right_lateral_reference_frame is not None:
        right_lateral_reference_x = chains["right"].forward(
            leg_vector(
                targets[right_lateral_reference_frame], "RL"
            )
        )[0, 3]
    frame_zero_exact, nominal_poses_finite = nominal_validation(targets, chains)
    replay_last_frame = min(
        int(math.floor(args.replay_end / args.dt + 1e-12)), len(targets) - 1,
    )
    print("[STEP SDF6D WORLD-FLAT REPLAY]")
    print(f"CSV={args.csv}")
    print(f"source rows={len(targets)}")
    print(f"SDF={args.sdf}")
    print(f"hold-before={args.hold_before:.3f} sim-s")
    print(f"replay=0.000~{replay_last_frame * args.dt:.3f} sim-s")
    print(f"hold-after={args.hold_after:.3f} sim-s")
    print(
        f"world-flat leg={args.world_flat_leg} "
        f"window={args.world_flat_start:.3f}~{args.world_flat_end:.3f} sim-s "
        f"ramp-in/out={args.world_flat_ramp_in:.3f}/"
        f"{args.world_flat_ramp_out:.3f} sim-s"
    )
    if planner_dx is None:
        print("lateral-balance=disabled")
    else:
        print(
            f"lateral-balance planner={args.lateral_balance_planner_csv} "
            f"window={args.lateral_balance_start:.3f}~"
            f"{args.lateral_balance_end:.3f} sim-s "
            f"Kp/Kd={args.lateral_balance_kp:.6f}/"
            f"{args.lateral_balance_kd:.6f} "
            f"max-offset={args.lateral_balance_max_offset_m:.6f} m"
        )
    if args.lateral_balance_support_handoff:
        print(
            "lateral-balance-support-handoff=enabled "
            f"ramp={args.lateral_balance_support_handoff_ramp_s:.3f} sim-s"
        )
    else:
        print("lateral-balance-support-handoff=disabled")
    if args.pitch_balance_start is None:
        print("pitch-balance=disabled")
    else:
        print(
            f"pitch-balance window={args.pitch_balance_start:.3f}~"
            f"{args.pitch_balance_end:.3f} sim-s "
            f"Kp/Kd={args.pitch_balance_kp:.6f}/"
            f"{args.pitch_balance_kd:.6f} s "
            f"max-angle={args.pitch_balance_max_angle_deg:.3f} deg "
            f"ramp-in/out={args.pitch_balance_ramp_in:.3f}/"
            f"{args.pitch_balance_ramp_out:.3f} sim-s"
        )
    if args.pitch_balance_support_handoff:
        print(
            "pitch-balance-support-handoff=enabled "
            f"ramp={args.pitch_balance_support_handoff_ramp_s:.3f} sim-s "
            f"RIGHT-sign={args.pitch_balance_right_support_sign:+d} "
            f"RIGHT-max={args.pitch_balance_right_max_angle_deg:.3f} deg"
        )
    else:
        print("pitch-balance-support-handoff=disabled")
    if (
        args.pitch_balance_support_handoff
        and args.pitch_balance_right_rate_limit_deg_s is not None
    ):
        print(
            "RIGHT-pitch-balance-handoff-rate-limit=enabled "
            f"rate={args.pitch_balance_right_rate_limit_deg_s:.3f} deg/s"
        )
    else:
        print("RIGHT-pitch-balance-handoff-rate-limit=disabled")
    if args.left_support_impact_anchor:
        print(
            "LEFT-support-impact-anchor=enabled "
            f"hold={args.left_support_impact_anchor_hold_s:.3f} s "
            f"release={args.left_support_impact_anchor_release_s:.3f} s"
        )
    else:
        print("LEFT-support-impact-anchor=disabled")
    if args.swing_world_z_leg is None:
        print("swing-world-Z=disabled")
    else:
        print(
            f"swing-world-Z leg={args.swing_world_z_leg} "
            f"window={args.swing_world_z_start:.3f}~"
            f"{args.swing_world_z_end:.3f} sim-s "
            f"ramp-in/out={args.swing_world_z_ramp_in:.3f}/"
            f"{args.swing_world_z_ramp_out:.3f} sim-s"
        )
    print(
        "swing-world-Z-max-correction-release-speed="
        + (
            "disabled"
            if args.swing_world_z_max_correction_release_speed_mps is None
            else (
                f"{args.swing_world_z_max_correction_release_speed_mps:.3f} "
                "m/s"
            )
        )
    )
    print(
        "max-feedback-joint-speed="
        + (
            "disabled"
            if args.max_feedback_joint_speed_deg_s is None
            else f"{args.max_feedback_joint_speed_deg_s:.3f} deg/s"
        )
    )
    if args.right_swing_lateral_scale is not None:
        print(
            "RIGHT-swing-lateral-scale="
            f"{args.right_swing_lateral_scale:.6f} "
            f"reference={args.right_swing_lateral_reference_time:.3f} sim-s "
            f"reference-X={right_lateral_reference_x:.9f} m"
        )
    if args.touchdown_support_hold:
        print(
            "touchdown-support-hold=enabled "
            f"contact={args.touchdown_right_contact_topic} "
            f"joint-state={args.touchdown_leg_joint_state_topic} "
            f"Fz>={args.touchdown_right_fz_threshold:.3f} N for "
            f"{args.touchdown_confirm_s:.3f} sim-s "
            f"left-ramp-out={args.touchdown_left_feedback_ramp_out:.3f} sim-s"
        )
    else:
        print("touchdown-support-hold=disabled")
    if args.touchdown_z_arrest:
        print(
            "touchdown-Z-arrest=enabled "
            f"Fz>={args.touchdown_z_arrest_threshold_n:.3f} N "
            f"confirm={args.touchdown_z_arrest_confirm_s:.3f} sim-s "
            f"settle-depth={args.touchdown_z_arrest_settle_depth_m:.6f} m "
            f"settle-speed={args.touchdown_z_arrest_settle_speed_mps:.6f} m/s"
        )
    else:
        print("touchdown-Z-arrest=disabled")
    if args.touchdown_z_arrest_post_settle_release_s is None:
        print("touchdown-Z-arrest-post-settle-release=disabled")
    else:
        print(
            "touchdown-Z-arrest-post-settle-release=enabled "
            f"duration={args.touchdown_z_arrest_post_settle_release_s:.3f} "
            "sim-s"
        )
    print(
        "LEFT-support-Z-hold="
        f"{'enabled' if args.left_support_z_hold_until_right_confirmed else 'disabled'} "
        f"release={args.left_support_z_release_s:.3f} sim-s "
        f"through-replay={int(args.left_support_z_hold_through_replay)}"
    )
    fore_aft_requested_deg = (
        0.0 if args.fore_aft_sign_test_correction_deg is None
        else args.fore_aft_sign_test_correction_deg
    )
    fore_aft_sign_test_enabled = abs(fore_aft_requested_deg) > 0.0
    if fore_aft_sign_test_enabled:
        print(
            "fore-aft-sign-test=enabled "
            f"world-X={fore_aft_requested_deg:+.3f} deg "
            f"ramp-in={args.fore_aft_sign_test_ramp_in_s:.3f} sim-s"
        )
    else:
        print("fore-aft-sign-test=disabled")
    touchdown_flatten_requested_deg = (
        0.0 if args.right_touchdown_flatten_correction_deg is None
        else args.right_touchdown_flatten_correction_deg
    )
    touchdown_flatten_enabled = touchdown_flatten_requested_deg != 0.0
    if touchdown_flatten_enabled:
        print(
            "RIGHT-touchdown-flatten=enabled "
            f"world-X={touchdown_flatten_requested_deg:+.3f} deg "
            f"start={args.right_touchdown_flatten_start:.3f} sim-s "
            f"ramp-in/out={args.right_touchdown_flatten_ramp_in_s:.3f}/"
            f"{args.right_touchdown_flatten_ramp_out_s:.3f} sim-s"
        )
    else:
        print("RIGHT-touchdown-flatten=disabled")
    print(f"frame0 Candidate B exact={'YES' if frame_zero_exact else 'NO'}")
    print(f"nominal SDF FK finite={'YES' if nominal_poses_finite else 'NO'}")
    if not frame_zero_exact or not nominal_poses_finite:
        raise RuntimeError("nominal CSV/SDF static validation failed")

    if args.dry_run:
        print("world-flat runtime solve requires live base odometry")
        print("[DRY RUN COMPLETE]")
        return

    args.command_log.parent.mkdir(parents=True, exist_ok=True)
    publishers = GazeboDoublePublishers(list(LEG_JOINT_TOPICS.values()), False)
    clock = SimulationClock(args.odom_topic)
    touchdown_measurements = (
        TouchdownMeasurements(
            args.touchdown_right_contact_topic,
            args.touchdown_leg_joint_state_topic,
            require_joint_state=(
                args.touchdown_support_hold
                or args.left_support_impact_anchor
            ),
        )
        if args.touchdown_support_hold or args.touchdown_z_arrest else None
    )

    with args.command_log.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=log_columns())
        writer.writeheader()
        publish_targets(publishers, targets[0])
        write_log(
            writer, 0, math.nan, 0.0, 0, "INITIAL_PRELOAD", targets[0],
            np.full(3, math.nan), False, False, None, 0.0, 0.0,
        )
        stream.flush()

        previous_sim_time = None
        while True:
            (
                sim_time, base_rotation, base_position, base_x, base_vx,
                odom_pitch_rate, ready,
            ) = clock.wait_for_time_after(
                previous_sim_time, timeout=5.0,
            )
            if ready and sim_time is not None and base_rotation is not None:
                start_sim_time = sim_time
                break
            print("[WAIT] no valid simulation-time/base-orientation update yet")

        initial_right_pose = chains["right"].forward(CANDIDATE_B["right"])
        initial_left_pose = chains["left"].forward(CANDIDATE_B["left"])
        right_solver = WorldFlatRightSolver(
            chains["right"], base_rotation @ initial_right_pose[:3, :3],
            args.dt, base_rotation,
            initial_right_pose[:3, 3], initial_left_pose[:3, 3],
            leg_name="right",
            lateral_scale=args.right_swing_lateral_scale,
            lateral_reference_frame=right_lateral_reference_frame,
            lateral_reference_x=right_lateral_reference_x,
        )
        left_solver = WorldFlatRightSolver(
            chains["left"], base_rotation @ initial_left_pose[:3, :3],
            args.dt, base_rotation,
            initial_left_pose[:3, 3], initial_right_pose[:3, 3],
            leg_name="left",
        )
        touchdown_hold = (
            RightSupportHold(
                chains["right"], args.touchdown_right_fz_threshold,
                args.touchdown_confirm_s, args.swing_world_z_start,
                args.touchdown_left_feedback_ramp_out,
            )
            if args.touchdown_support_hold else None
        )
        touchdown_z_arrest = (
            TouchdownZArrest(
                args.touchdown_z_arrest_threshold_n,
                args.touchdown_z_arrest_confirm_s,
                args.swing_world_z_start,
                args.swing_world_z_max_correction_release_speed_mps,
                args.dt,
                args.touchdown_z_arrest_settle_depth_m,
                args.touchdown_z_arrest_settle_speed_mps,
                args.touchdown_z_arrest_post_settle_release_s,
            )
            if args.touchdown_z_arrest else None
        )
        left_support_z_hold = (
            LeftSupportZHold(
                args.left_support_z_release_s, touchdown_z_arrest,
                hold_through_replay=args.left_support_z_hold_through_replay,
            )
            if args.left_support_z_hold_until_right_confirmed else None
        )
        left_support_impact_anchor = LeftSupportImpactAnchor(
            args.left_support_impact_anchor,
            args.left_support_impact_anchor_hold_s,
            args.left_support_impact_anchor_release_s,
            chains["left"],
        )
        safety = CommandSafetyMonitor()
        safety.check(targets[0], start_sim_time)
        last_published_targets = dict(targets[0])
        print(f"[START] sim_time={start_sim_time:.9f}")

        publish_index = 1
        next_frame = 0
        replay_finished_at = None
        feedback_release_pending = {"RL": False, "LL": False}
        replay_base_x = None
        replay_base_pitch = None
        derivative_previous_pitch = None
        derivative_previous_time = None
        previous_successful_right_pitch_correction = 0.0
        while True:
            (
                sim_time, base_rotation, base_position, base_x, base_vx,
                odom_pitch_rate, ready,
            ) = clock.wait_for_time_after(
                previous_sim_time, timeout=5.0,
            )
            if not ready:
                print("[WAIT] simulation time not advancing")
                continue
            previous_sim_time = sim_time
            if base_rotation is None:
                print("[WARNING] invalid base quaternion; retaining prior command")
                continue
            base_rpy = Rotation.from_matrix(base_rotation).as_euler("xyz")
            if math.isfinite(odom_pitch_rate):
                base_pitch_rate = odom_pitch_rate
            elif (
                derivative_previous_pitch is not None
                and sim_time > derivative_previous_time
            ):
                base_pitch_rate = (
                    base_rpy[1] - derivative_previous_pitch
                ) / (sim_time - derivative_previous_time)
            else:
                base_pitch_rate = 0.0
            derivative_previous_pitch = base_rpy[1]
            derivative_previous_time = sim_time
            elapsed = sim_time - start_sim_time
            if elapsed < args.hold_before:
                continue
            trajectory_elapsed = elapsed - args.hold_before

            while (
                next_frame <= replay_last_frame
                and trajectory_elapsed + 1e-9 >= next_frame * args.dt
            ):
                trajectory_time = next_frame * args.dt
                nominal = targets[next_frame]
                if replay_base_x is None:
                    replay_base_x = base_x
                if replay_base_pitch is None:
                    replay_base_pitch = base_rpy[1]
                world_flat_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.world_flat_start, args.world_flat_end,
                    args.world_flat_ramp_in, args.world_flat_ramp_out,
                )
                swing_world_z_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.swing_world_z_start, args.swing_world_z_end,
                    args.swing_world_z_ramp_in, args.swing_world_z_ramp_out,
                )
                world_flat_active = world_flat_beta > 0.0
                swing_world_z_active = swing_world_z_beta > 0.0
                right_world_beta = (
                    world_flat_beta
                    if args.world_flat_leg in ("right", "both") else 0.0
                )
                left_world_beta = (
                    world_flat_beta
                    if args.world_flat_leg in ("left", "both") else 0.0
                )
                lateral_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.lateral_balance_start, args.lateral_balance_end,
                    args.lateral_balance_ramp_in, args.lateral_balance_ramp_out,
                )
                lateral_active = lateral_beta > 0.0
                lateral_diagnostics = None
                lateral_offset = 0.0
                if planner_dx is not None:
                    base_dx = base_x - replay_base_x
                    position_error = base_dx - planner_dx[next_frame]
                    velocity_error = base_vx - planner_vx[next_frame]
                    raw_offset = (
                        args.lateral_balance_kp * position_error
                        + args.lateral_balance_kd * velocity_error
                    )
                    clamped_offset = max(
                        -args.lateral_balance_max_offset_m,
                        min(args.lateral_balance_max_offset_m, raw_offset),
                    )
                    lateral_offset = lateral_beta * clamped_offset
                    lateral_diagnostics = {
                        "active": lateral_active,
                        "planner_dx": planner_dx[next_frame],
                        "planner_vx": planner_vx[next_frame],
                        "base_dx": base_dx, "base_vx": base_vx,
                        "position_error": position_error,
                        "velocity_error": velocity_error,
                        "raw_offset": raw_offset,
                        "applied_offset": lateral_offset,
                        "beta": lateral_beta,
                    }
                lateral_controller_offset = lateral_offset
                pitch_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.pitch_balance_start, args.pitch_balance_end,
                    args.pitch_balance_ramp_in, args.pitch_balance_ramp_out,
                )
                pitch_active = pitch_beta > 0.0
                pitch_correction = 0.0
                pitch_diagnostics = None
                if args.pitch_balance_start is not None:
                    pitch_error = base_rpy[1] - replay_base_pitch
                    pitch_rate_error = base_pitch_rate
                    pitch_raw = (
                        args.pitch_balance_kp * pitch_error
                        + args.pitch_balance_kd * pitch_rate_error
                    )
                    max_pitch = math.radians(
                        args.pitch_balance_max_angle_deg
                    )
                    pitch_clamped = max(
                        -max_pitch, min(max_pitch, pitch_raw)
                    )
                    pitch_correction = pitch_beta * pitch_clamped
                    pitch_diagnostics = {
                        "active": pitch_active,
                        "reference": replay_base_pitch,
                        "actual": base_rpy[1], "error": pitch_error,
                        "rate": base_pitch_rate,
                        "rate_error": pitch_rate_error,
                        "raw_correction": pitch_raw,
                        "applied_correction": pitch_correction,
                        "beta": pitch_beta,
                    }
                right_fz = math.nan
                measured_right_q = np.full(6, math.nan)
                measured_left_q = np.full(6, math.nan)
                touchdown_detected_now = False
                if touchdown_measurements is not None:
                    (
                        right_fz, contact_sequence, measured_right_q,
                        measured_left_q,
                    ) = touchdown_measurements.snapshot()
                    if touchdown_hold is not None:
                        touchdown_detected_now = touchdown_hold.update(
                            sim_time, trajectory_time, right_fz,
                            contact_sequence, measured_right_q,
                            base_rotation, base_position, lateral_offset,
                        )
                    if touchdown_z_arrest is not None:
                        touchdown_z_arrest.update_contact(
                            sim_time, trajectory_time, right_fz,
                            contact_sequence,
                        )
                    left_support_impact_anchor.try_latch(
                        touchdown_z_arrest, measured_left_q,
                        base_rotation, base_position,
                    )
                left_support_impact_anchor_beta = (
                    left_support_impact_anchor.beta(trajectory_time)
                )
                (
                    lateral_handoff_beta,
                    lateral_support_state,
                ) = lateral_balance_support_handoff(
                    trajectory_time, touchdown_z_arrest,
                    args.lateral_balance_support_handoff,
                    args.lateral_balance_support_handoff_ramp_s,
                )
                fore_aft_support_confirmed = (
                    touchdown_z_arrest is not None
                    and touchdown_z_arrest.state == "TOUCHDOWN_CONFIRMED"
                    and math.isfinite(touchdown_z_arrest.confirmed_time)
                )
                fore_aft_beta = 0.0
                if fore_aft_sign_test_enabled and fore_aft_support_confirmed:
                    fore_aft_beta = fore_aft_sign_test_beta(
                        trajectory_time,
                        touchdown_z_arrest.confirmed_time,
                        args.fore_aft_sign_test_ramp_in_s,
                    )
                fore_aft_requested = math.radians(fore_aft_requested_deg)
                fore_aft_applied = fore_aft_beta * fore_aft_requested
                fore_aft_diagnostics = {
                    "enabled": fore_aft_sign_test_enabled,
                    "support_confirmed": fore_aft_support_confirmed,
                    "requested": fore_aft_requested,
                    "beta": fore_aft_beta,
                    "applied": fore_aft_applied,
                }
                flatten_diagnostics = touchdown_flatten_diagnostics(
                    trajectory_time,
                    math.radians(touchdown_flatten_requested_deg),
                    args.right_touchdown_flatten_start,
                    args.right_touchdown_flatten_ramp_in_s,
                    args.right_touchdown_flatten_ramp_out_s,
                    touchdown_z_arrest,
                ) if touchdown_flatten_enabled else None
                touchdown_flatten_applied = (
                    0.0 if flatten_diagnostics is None
                    else flatten_diagnostics["applied"]
                )
                pitch_handoff_beta, pitch_support_state = (
                    pitch_balance_support_handoff(
                        trajectory_time, touchdown_z_arrest,
                        args.pitch_balance_support_handoff,
                        args.pitch_balance_support_handoff_ramp_s,
                    )
                )
                left_pitch_correction = pitch_correction * (
                    1.0 - pitch_handoff_beta
                )
                right_pitch_raw = (
                    args.pitch_balance_right_support_sign * pitch_correction
                )
                right_pitch_max = math.radians(
                    args.pitch_balance_right_max_angle_deg
                )
                right_pitch_clamped = max(
                    -right_pitch_max, min(right_pitch_max, right_pitch_raw)
                )
                right_pitch_target = (
                    pitch_handoff_beta * right_pitch_clamped
                )
                right_pitch_rate_limit = (
                    args.pitch_balance_right_rate_limit_deg_s
                    if args.pitch_balance_support_handoff else None
                )
                (
                    right_pitch_correction,
                    right_pitch_rate_limited,
                ) = propose_angular_rate_limited(
                    right_pitch_target,
                    previous_successful_right_pitch_correction,
                    right_pitch_rate_limit,
                    args.dt,
                )
                if (
                    right_pitch_rate_limit is not None
                    and right_pitch_target == 0.0
                    and right_pitch_correction == 0.0
                ):
                    previous_successful_right_pitch_correction = 0.0
                pitch_handoff_diagnostics = {
                    "enabled": args.pitch_balance_support_handoff,
                    "beta": pitch_handoff_beta,
                    "state": pitch_support_state,
                    "left_applied": left_pitch_correction,
                    "right_raw": right_pitch_raw,
                    "right_target": right_pitch_target,
                    "right_applied": right_pitch_correction,
                    "right_rate_limited": right_pitch_rate_limited,
                    "right_rate_limit_deg_s": right_pitch_rate_limit,
                    "right_sign": args.pitch_balance_right_support_sign,
                }
                left_handoff_beta = 1.0
                left_offset_before_handoff = math.nan
                left_offset_after_handoff = math.nan
                if touchdown_hold is not None and touchdown_hold.active:
                    (
                        left_handoff_beta, left_offset_before_handoff,
                        left_offset_after_handoff,
                    ) = touchdown_hold.left_handoff(sim_time)
                    lateral_offset = left_offset_after_handoff
                    lateral_active = abs(lateral_offset) > 1e-12
                    if lateral_diagnostics is not None:
                        lateral_diagnostics["active"] = lateral_active
                        lateral_diagnostics["applied_offset"] = lateral_offset
                        lateral_diagnostics["beta"] = left_handoff_beta
                lateral_handoff_source = (
                    lateral_controller_offset
                    if args.lateral_balance_support_handoff
                    else lateral_offset
                )
                (
                    left_lateral_offset,
                    right_lateral_offset,
                ) = split_lateral_support_offset(
                    lateral_handoff_source,
                    lateral_handoff_beta,
                    args.lateral_balance_support_handoff,
                )
                if (
                    lateral_diagnostics is None
                    and args.lateral_balance_support_handoff
                ):
                    lateral_diagnostics = {
                        "active": False,
                        "planner_dx": math.nan, "planner_vx": math.nan,
                        "base_dx": math.nan, "base_vx": math.nan,
                        "position_error": math.nan,
                        "velocity_error": math.nan,
                        "raw_offset": math.nan,
                        "applied_offset": lateral_controller_offset,
                        "beta": lateral_beta,
                    }
                if lateral_diagnostics is not None:
                    if args.lateral_balance_support_handoff:
                        lateral_diagnostics["active"] = lateral_active
                        lateral_diagnostics["applied_offset"] = (
                            lateral_controller_offset
                        )
                        lateral_diagnostics["beta"] = lateral_beta
                    lateral_diagnostics.update({
                        "handoff_enabled": (
                            args.lateral_balance_support_handoff
                        ),
                        "handoff_beta": lateral_handoff_beta,
                        "handoff_state": lateral_support_state,
                        "left_applied_offset": left_lateral_offset,
                        "right_applied_offset": right_lateral_offset,
                    })
                solve_result = None
                support_result = None
                left_solve_result = None
                final = nominal
                rate_diagnostics = None
                left_rate_diagnostics = None
                if touchdown_hold is not None and touchdown_hold.active:
                    support_result = touchdown_hold.solve(
                        base_rotation, base_position, measured_right_q,
                        leg_vector(last_published_targets, "RL"),
                    )
                    requested_q = support_result["q"]
                    (
                        published_q, limited_count,
                        requested_speed, published_speed,
                    ) = limit_feedback_joint_rate(
                        requested_q,
                        leg_vector(last_published_targets, "RL"),
                        args.max_feedback_joint_speed_deg_s,
                        args.dt,
                    )
                    rate_diagnostics = {
                        "active": args.max_feedback_joint_speed_deg_s is not None,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                    final = set_leg_vector(nominal, "RL", published_q)
                    feedback_release_pending["RL"] = True
                    world_flat_active = left_world_beta > 0.0
                    swing_world_z_active = False
                elif (
                    right_world_beta > 0.0
                    or swing_world_z_active
                    or right_solver.correction_release_state == "RELEASE_TAIL"
                    or fore_aft_sign_test_enabled
                    or touchdown_flatten_enabled
                    or abs(right_pitch_correction) > 0.0
                    or abs(right_lateral_offset) > 1e-12
                ):
                    solve_result = right_solver.solve(
                        next_frame, sim_time, base_rotation,
                        leg_vector(nominal, "RL"),
                        leg_vector(nominal, "LL"),
                        right_world_beta, swing_world_z_beta,
                        chains["left"],
                        support_x_offset=right_lateral_offset,
                        support_pitch_correction=right_pitch_correction,
                        support_pitch_is_right=True,
                        world_x_orientation_correction=(
                            fore_aft_applied + touchdown_flatten_applied
                        ),
                        correction_release_speed_mps=(
                            args.swing_world_z_max_correction_release_speed_mps
                        ),
                        correction_release_active=(
                            args.swing_world_z_end is not None
                            and trajectory_time > args.swing_world_z_end
                        ),
                        base_position=base_position,
                        touchdown_z_arrest=touchdown_z_arrest,
                        support_z_trajectory_time=trajectory_time,
                    )
                    if (
                        solve_result["success"]
                        and right_pitch_rate_limit is not None
                    ):
                        previous_successful_right_pitch_correction = (
                            right_pitch_correction
                        )
                    requested_q = solve_result["q"]
                    published_q = requested_q
                    limited_count = 0
                    requested_speed = 0.0
                    published_speed = 0.0
                    limiter_active = (
                        solve_result["success"]
                        and args.max_feedback_joint_speed_deg_s is not None
                    )
                    if solve_result["success"]:
                        (
                            published_q, limited_count,
                            requested_speed, published_speed,
                        ) = limit_feedback_joint_rate(
                            requested_q,
                            leg_vector(last_published_targets, "RL"),
                            args.max_feedback_joint_speed_deg_s,
                            args.dt,
                        )
                    rate_diagnostics = {
                        "active": limiter_active,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                    final = set_leg_vector(nominal, "RL", published_q)
                    feedback_release_pending["RL"] = (
                        np.max(np.abs(published_q - leg_vector(nominal, "RL")))
                        > 1e-12
                    )
                elif (
                    feedback_release_pending["RL"]
                    and args.max_feedback_joint_speed_deg_s is not None
                ):
                    requested_q = leg_vector(nominal, "RL")
                    (
                        published_q, limited_count,
                        requested_speed, published_speed,
                    ) = limit_feedback_joint_rate(
                        requested_q,
                        leg_vector(last_published_targets, "RL"),
                        args.max_feedback_joint_speed_deg_s,
                        args.dt,
                    )
                    final = set_leg_vector(nominal, "RL", published_q)
                    feedback_release_pending["RL"] = limited_count > 0
                    rate_diagnostics = {
                        "active": True,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }

                if (
                    left_world_beta > 0.0
                    or abs(left_lateral_offset) > 1e-12 or pitch_active
                    or left_support_z_hold is not None
                    or left_support_impact_anchor_beta > 0.0
                ):
                    left_solve_result = left_solver.solve(
                        next_frame, sim_time, base_rotation,
                        leg_vector(nominal, "LL"),
                        leg_vector(nominal, "RL"),
                        left_world_beta, 0.0,
                        chains["right"],
                        support_x_offset=left_lateral_offset,
                        support_pitch_correction=left_pitch_correction,
                        base_position=base_position,
                        support_z_hold=left_support_z_hold,
                        support_z_trajectory_time=trajectory_time,
                        impact_anchor=left_support_impact_anchor,
                        impact_anchor_trajectory_time=trajectory_time,
                    )
                    requested_q = left_solve_result["q"]
                    published_q = requested_q
                    limited_count = 0
                    requested_speed = 0.0
                    published_speed = 0.0
                    limiter_active = (
                        left_solve_result["success"]
                        and args.max_feedback_joint_speed_deg_s is not None
                    )
                    if left_solve_result["success"]:
                        (
                            published_q, limited_count,
                            requested_speed, published_speed,
                        ) = limit_feedback_joint_rate(
                            requested_q,
                            leg_vector(last_published_targets, "LL"),
                            args.max_feedback_joint_speed_deg_s,
                            args.dt,
                        )
                    left_rate_diagnostics = {
                        "active": limiter_active,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                    final = set_leg_vector(final, "LL", published_q)
                    feedback_release_pending["LL"] = (
                        np.max(np.abs(published_q - leg_vector(nominal, "LL")))
                        > 1e-12
                    )
                elif (
                    feedback_release_pending["LL"]
                    and args.max_feedback_joint_speed_deg_s is not None
                ):
                    requested_q = leg_vector(nominal, "LL")
                    (
                        published_q, limited_count,
                        requested_speed, published_speed,
                    ) = limit_feedback_joint_rate(
                        requested_q,
                        leg_vector(last_published_targets, "LL"),
                        args.max_feedback_joint_speed_deg_s,
                        args.dt,
                    )
                    final = set_leg_vector(final, "LL", published_q)
                    feedback_release_pending["LL"] = limited_count > 0
                    left_rate_diagnostics = {
                        "active": True,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                if not all(math.isfinite(value) for value in final.values()):
                    raise RuntimeError("refusing to publish non-finite joint target")
                max_delta, max_velocity = safety.check(final, sim_time)
                publish_targets(publishers, final)
                last_published_targets = dict(final)
                write_log(
                    writer, publish_index, sim_time, trajectory_time,
                    next_frame, "REPLAY", final, base_rpy,
                    world_flat_active, swing_world_z_active,
                    solve_result, max_delta, max_velocity, rate_diagnostics,
                    left_solve_result, left_rate_diagnostics,
                    lateral_diagnostics,
                    {
                        "state": (
                            "DISABLED" if touchdown_hold is None
                            else touchdown_hold.state
                        ),
                        "detected": touchdown_detected_now,
                        "right_fz": right_fz,
                        "sim_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_sim_time
                        ),
                        "trajectory_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_trajectory_time
                        ),
                        "support_active": (
                            touchdown_hold is not None and touchdown_hold.active
                        ),
                        "support_result": support_result,
                        "left_handoff_beta": left_handoff_beta,
                        "left_offset_before": left_offset_before_handoff,
                        "left_offset_after": left_offset_after_handoff,
                    },
                    pitch_diagnostics,
                    (
                        None if touchdown_z_arrest is None
                        else touchdown_z_arrest.diagnostics(
                            right_fz, solve_result
                        )
                    ),
                    fore_aft_sign_test_diagnostics=fore_aft_diagnostics,
                    left_support_z_diagnostics=(
                        None if left_solve_result is None
                        else left_solve_result.get("left_support_z_hold")
                    ),
                    touchdown_flatten_diagnostics_row=flatten_diagnostics,
                    pitch_handoff_diagnostics=pitch_handoff_diagnostics,
                    left_support_impact_anchor_diagnostics=(
                        left_support_impact_anchor.diagnostics(trajectory_time)
                    ),
                )
                stream.flush()
                if touchdown_detected_now:
                    print(
                        f"[TOUCHDOWN] sim={sim_time:.3f}s "
                        f"traj={trajectory_time:.3f}s Fz={right_fz:.3f} N"
                    )
                if touchdown_hold is not None:
                    touchdown_hold.finish_confirmation_frame()
                if next_frame % 10 == 0 or next_frame == replay_last_frame:
                    print(
                        f"[REPLAY] frame={next_frame:4d} traj={trajectory_time:.2f}s "
                        f"sim={sim_time:.3f}s "
                        f"world_flat={int(world_flat_active)} "
                        f"swing_world_z={int(swing_world_z_active)}"
                    )
                publish_index += 1
                next_frame += 1

            if next_frame > replay_last_frame and replay_finished_at is None:
                replay_finished_at = sim_time
                print(f"[REPLAY COMPLETE] holding frame {replay_last_frame}")
            if replay_finished_at is not None:
                if sim_time - replay_finished_at >= args.hold_after:
                    break
                trajectory_time = replay_last_frame * args.dt
                world_flat_beta = feedback_beta(
                    "HOLD_AFTER", trajectory_time,
                    args.world_flat_start, args.world_flat_end,
                    args.world_flat_ramp_in, args.world_flat_ramp_out,
                )
                swing_world_z_beta = feedback_beta(
                    "HOLD_AFTER", trajectory_time,
                    args.swing_world_z_start, args.swing_world_z_end,
                    args.swing_world_z_ramp_in, args.swing_world_z_ramp_out,
                )
                world_flat_active = world_flat_beta > 0.0
                swing_world_z_active = swing_world_z_beta > 0.0
                solve_result = None
                final = dict(last_published_targets)
                if not all(math.isfinite(value) for value in final.values()):
                    raise RuntimeError("refusing to publish non-finite joint target")
                max_delta, max_velocity = safety.check(final, sim_time)
                publish_targets(publishers, final)
                write_log(
                    writer, publish_index, sim_time, trajectory_time,
                    replay_last_frame, "HOLD_AFTER", final, base_rpy,
                    world_flat_active, swing_world_z_active,
                    solve_result, max_delta, max_velocity,
                    touchdown_diagnostics={
                        "state": (
                            "DISABLED" if touchdown_hold is None
                            else touchdown_hold.state
                        ),
                        "detected": False,
                        "right_fz": math.nan,
                        "sim_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_sim_time
                        ),
                        "trajectory_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_trajectory_time
                        ),
                        "support_active": False,
                        "support_result": None,
                        "left_handoff_beta": 0.0,
                        "left_offset_before": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.left_offset_at_touchdown
                        ),
                        "left_offset_after": 0.0,
                    },
                    touchdown_z_arrest_diagnostics=(
                        None if touchdown_z_arrest is None
                        else touchdown_z_arrest.diagnostics(
                            math.nan, solve_result
                        )
                    ),
                )
                stream.flush()
                publish_index += 1

    print("[DONE]")
    print(f"command log={args.command_log}")


if __name__ == "__main__":
    main()
