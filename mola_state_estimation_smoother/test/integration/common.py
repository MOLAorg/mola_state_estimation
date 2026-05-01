# Common utilities for integration tests of mola_state_estimation_smoother.
#
# Geodetic convention:
#   flat-Earth approximation, valid for trajectories shorter than ~100 m around the
#   test origin (Almeria, matching existing C++ unit tests).
#
# IMU quaternion convention (imu_attitude_azimuth_offset_deg=0):
#   The MOLA smoother internally computes ENU_yaw = quat_yaw + 90 deg + offset.
#   With offset=0, quat_yaw must equal (vehicle_ENU_yaw - 90 deg).
#   See also test-static-gnss-imu-orientation.cpp, vehicleToAzimuth formula.
import logging
import math
import time

import numpy as np

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s: %(message)s')

# Geodetic test origin  (must match static_test_smoother_params.yaml fixed_geo_reference)
GT_LAT0_DEG = 36.8407
GT_LON0_DEG = -2.4093
GT_ALT0_M = 100.0

# Ground truth for static scenario: 30 m East, 50 m North, yaw=60 deg ENU
STATIC_GT_X = 30.0
STATIC_GT_Y = 50.0
STATIC_GT_Z = 0.0
STATIC_GT_YAW_RAD = math.radians(60.0)

# Moving trajectory parameters: sinusoidal zigzag at 1 m/s
MOVING_V = 1.0        # forward speed [m/s]
MOVING_A = 2.0        # lateral amplitude [m]
MOVING_OMEGA = 0.5    # lateral frequency [rad/s]
MOVING_DURATION_S = 60.0


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def wrap_pi(angle):
    """Wrap angle (radians) to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def quaternion_from_euler_zyx(roll, pitch, yaw):
    """ZYX Euler -> quaternion, returns (x, y, z, w)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def imu_quat_from_enu_pose(gt_yaw_enu, gt_pitch=0.0, gt_roll=0.0):
    """
    Build IMU orientation quaternion compatible with
    imu_attitude_azimuth_offset_deg=0.

    The smoother recovers ENU yaw via: ENU_yaw = quat_yaw + pi/2 + offset.
    With offset=0 we must publish: quat_yaw = gt_yaw_enu - pi/2.
    """
    imu_yaw = gt_yaw_enu - math.pi / 2
    return quaternion_from_euler_zyx(gt_roll, gt_pitch, imu_yaw)


def imu_linear_acc_body(gt_yaw, gt_pitch=0.0, gt_roll=0.0,
                        world_accel_x=0.0, world_accel_y=0.0):
    """
    Specific force measured by IMU in body frame (m/s²).
    Equals (vehicle translational accel - gravity) rotated into body frame.
    For a flat, slowly-accelerating robot this is close to (0, 0, +9.81).
    """
    g_world = np.array([0.0, 0.0, -9.81])
    a_world = np.array([world_accel_x, world_accel_y, 0.0])
    sf_world = a_world - g_world  # specific force in world frame

    # Rotation matrix world -> body (ZYX): R^T of body->world
    cy, sy = math.cos(gt_yaw), math.sin(gt_yaw)
    cp, sp = math.cos(gt_pitch), math.sin(gt_pitch)
    cr, sr = math.cos(gt_roll), math.sin(gt_roll)
    # R_body_to_world (standard ZYX):
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])
    return R.T @ sf_world  # world to body


# ---------------------------------------------------------------------------
# Ground-truth trajectories
# ---------------------------------------------------------------------------

def static_gt_pose():
    """Return (x, y, z, yaw, pitch, roll) for the static scenario."""
    return STATIC_GT_X, STATIC_GT_Y, STATIC_GT_Z, STATIC_GT_YAW_RAD, 0.0, 0.0


def moving_gt_pose(t):
    """Return (x, y, z, yaw, pitch, roll) at trajectory time t [s]."""
    x = MOVING_V * t
    y = MOVING_A * math.sin(MOVING_OMEGA * t)
    vy = MOVING_A * MOVING_OMEGA * math.cos(MOVING_OMEGA * t)
    yaw = math.atan2(vy, MOVING_V)
    return x, y, 0.0, yaw, 0.0, 0.0


def moving_gt_twist(t):
    """Return body-frame twist (vx, vy, vz, wx, wy, wz) at time t [s]."""
    vx_w = MOVING_V
    vy_w = MOVING_A * MOVING_OMEGA * math.cos(MOVING_OMEGA * t)
    _, _, _, yaw, _, _ = moving_gt_pose(t)
    # Rotate world velocities to body frame
    vx = vx_w * math.cos(yaw) + vy_w * math.sin(yaw)
    vy = -vx_w * math.sin(yaw) + vy_w * math.cos(yaw)
    # Angular velocity dYaw/dt
    denom = MOVING_V ** 2 + (MOVING_A * MOVING_OMEGA *
                             math.cos(MOVING_OMEGA * t)) ** 2
    wz = (
        -MOVING_V * MOVING_A * MOVING_OMEGA ** 2 * math.sin(MOVING_OMEGA * t)
        / max(denom, 1e-9)
    )
    return vx, vy, 0.0, 0.0, 0.0, wz


def moving_gt_world_accel(t):
    """Return world-frame translational acceleration at time t [s]."""
    ay_w = -MOVING_A * MOVING_OMEGA ** 2 * math.sin(MOVING_OMEGA * t)
    return 0.0, ay_w


# ---------------------------------------------------------------------------
# Geodetic helpers (flat-Earth approximation, good for <100 m)
# ---------------------------------------------------------------------------

_M_PER_DEG_LAT = 111320.0


def enu_to_latlon(east_m, north_m, alt_m,
                  lat0_deg=GT_LAT0_DEG, lon0_deg=GT_LON0_DEG, alt0_m=GT_ALT0_M):
    """ENU [m] -> (lat_deg, lon_deg, alt_m)."""
    m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(lat0_deg))
    lat = lat0_deg + north_m / _M_PER_DEG_LAT
    lon = lon0_deg + east_m / max(m_per_deg_lon, 1e-9)
    return lat, lon, alt0_m + alt_m


def latlon_to_enu(lat_deg, lon_deg, alt,
                  lat0_deg=GT_LAT0_DEG, lon0_deg=GT_LON0_DEG, alt0_m=GT_ALT0_M):
    """(lat_deg, lon_deg, alt_m) -> ENU [m]."""
    m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(lat0_deg))
    north = (lat_deg - lat0_deg) * _M_PER_DEG_LAT
    east = (lon_deg - lon0_deg) * m_per_deg_lon
    return east, north, alt - alt0_m


def _round_trip_check():
    """Smoke-check flat-Earth round-trip at module import time."""
    e0, n0 = 30.0, 50.0
    lat, lon, alt = enu_to_latlon(e0, n0, 0.0)
    e1, n1, _ = latlon_to_enu(lat, lon, alt)
    assert abs(e0 - e1) < 0.01 and abs(n0 - n1) < 0.01, (
        f"flat-Earth round-trip error: ({e0},{n0}) -> ({e1},{n1})")


_round_trip_check()


# ---------------------------------------------------------------------------
# Latest-pose store (thread-safe via GIL for scalar writes)
# ---------------------------------------------------------------------------

class PoseLatest:
    """Stores the most recent (x, y, yaw) from a ROS subscription callback."""

    def __init__(self):
        self._data = None
        self._updated_at = None

    def update(self, x, y, yaw):
        self._data = (x, y, yaw)
        self._updated_at = time.monotonic()

    def latest(self):
        return self._data, self._updated_at


# ---------------------------------------------------------------------------
# Convergence assertion
# ---------------------------------------------------------------------------

def wait_for_convergence(
    node,
    gt_provider,
    est_provider,
    *,
    max_pos_err_m,
    max_heading_err_deg,
    settle_seconds=2.0,
    timeout_seconds=60.0,
    warm_up_seconds=0.0,
    max_sample_age_seconds=0.5,
):
    """
    Spin *node* until the estimated pose converges to the GT pose or timeout.

    Convergence requires the error to stay below threshold continuously for
    *settle_seconds*.  *warm_up_seconds* is a spin-only period before checking.
    """
    import rclpy

    start = time.monotonic()
    deadline = start + warm_up_seconds + timeout_seconds
    warm_up_end = start + warm_up_seconds
    settled_since = None
    pos_err = float("inf")
    yaw_err = float("inf")

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

        now = time.monotonic()
        if now < warm_up_end:
            continue

        gt, gt_t = gt_provider.latest()
        est, est_t = est_provider.latest()
        if (
            gt is None or est is None or
            gt_t is None or est_t is None or
            now - gt_t > max_sample_age_seconds or
            now - est_t > max_sample_age_seconds
        ):
            settled_since = None
            continue

        pos_err = math.hypot(gt[0] - est[0], gt[1] - est[1])
        yaw_err = wrap_pi(gt[2] - est[2])
        passed = (
            pos_err < max_pos_err_m and
            abs(yaw_err) < math.radians(max_heading_err_deg)
        )

        if passed:
            if settled_since is None:
                settled_since = now
            if now - settled_since >= settle_seconds:
                logging.info(
                    "Converged after %.1f s: pos_err=%.3f m, yaw_err=%.3f deg",
                    settle_seconds,
                    pos_err,
                    math.degrees(yaw_err),
                )
                return
        else:
            settled_since = None

    raise AssertionError(
        f"did not converge within {timeout_seconds:.0f} s: "
        f"pos_err={pos_err:.2f} m, yaw_err={math.degrees(yaw_err):.2f} deg"
    )
