/*
 * Arm kinematics and move planning for the two-way mode. No Arduino dependencies.
 * Keep in step with hil_toolkit/waypoint.py and D_OFFSETS in arm-sim-end.py.
 *
 * Joint angles are the sim's, in degrees: 0 at the initial pose.
 *   J1 base (Z) | J2 Arm A (X) | J3 Arm B (X) | J4 wrist pitch (X) | J5 wrist roll (Y)
 * A pose is the wrist (J4) position in mm, plus A = J2+J3+J4 (pitch) and B = J5 (roll).
 *
 * J2 and J3 turn about parallel axes in a plane that only J1 rotates, so the position
 * has a closed-form solution: J1 from the direction, J2/J3 as a two-link arm.
 */
#pragma once
#include <math.h>

namespace kin {

const int JOINT_COUNT = 5;

// d1: J1 -> J2, d2: J2 -> J3, d3: J3 -> J4, all in the arm's plane (y, z), in mm.
const double D1_Y = 5.0, D1_Z = 35.5;
const double D2_Y = -47.5, D2_Z = 194.5;
const double D3_Z = -140.0;

const double DEG = 180.0 / M_PI;
const double LIMIT_TOLERANCE_DEG = 0.05;   // rounding in the frame, not a real overshoot

const int PATH_SAMPLES = 32;               // checks per straight-line move
const double PATH_MAX_SAMPLE_STEP_DEG = 30.0;  // more than this between samples is a branch flip
const double PROFILE_PEAK = 1.875;         // peak speed of the profile / its average speed
const double MIN_DURATION_S = 0.1;

struct Pose {
  double x, y, z, a, b;
};

inline double wrap180(double deg) {
  deg = fmod(deg + 180.0, 360.0);
  if (deg < 0.0) deg += 360.0;
  return deg - 180.0;
}

inline double clampTo(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline void fk(const double q[JOINT_COUNT], Pose &p) {
  const double t2 = q[1] / DEG, t23 = (q[1] + q[2]) / DEG;
  const double s2 = sin(t2), c2 = cos(t2), s23 = sin(t23), c23 = cos(t23);
  const double r = D1_Y + (D2_Y * c2 - D2_Z * s2) - D3_Z * s23;
  const double z = D1_Z + (D2_Y * s2 + D2_Z * c2) + D3_Z * c23;
  const double t1 = q[0] / DEG;
  p.x = -r * sin(t1);
  p.y = r * cos(t1);
  p.z = z;
  p.a = q[1] + q[2] + q[3];
  p.b = q[4];
}

// Brings an angle into [lo, hi] allowing for whole turns and the rounding tolerance.
inline bool fitLimit(double &deg, double lo, double hi) {
  const double offsets[3] = {0.0, -360.0, 360.0};
  for (int i = 0; i < 3; i++) {
    const double v = deg + offsets[i];
    if (v >= lo - LIMIT_TOLERANCE_DEG && v <= hi + LIMIT_TOLERANCE_DEG) {
      deg = clampTo(v, lo, hi);
      return true;
    }
  }
  return false;
}

// The valid solution closest to `seed`, or false if the pose is out of reach.
inline bool ik(const Pose &p, const double seed[JOINT_COUNT],
               const double lo[JOINT_COUNT], const double hi[JOINT_COUNT],
               double out[JOINT_COUNT]) {
  const double L1 = hypot(D2_Y, D2_Z), L2 = fabs(D3_Z);
  const double alpha10 = atan2(D2_Z, D2_Y);          // link 1 direction at the initial pose
  const double gamma0 = atan2(D3_Z, 0.0) - alpha10;  // link 2 relative to link 1, same pose

  const double rho = hypot(p.x, p.y);
  double t1s[2], rs[2];
  int branches = 2;
  if (rho < 1e-6) {          // on the base axis, where J1 is free: stay where it is
    t1s[0] = seed[0];
    rs[0] = 0.0;
    branches = 1;
  } else {
    t1s[0] = atan2(p.x, -p.y) * DEG;   // the arm's working side: r < 0, towards -Y
    rs[0] = -rho;
    t1s[1] = atan2(-p.x, p.y) * DEG;
    rs[1] = rho;
  }

  bool found = false;
  double bestScore = 0.0;
  for (int i = 0; i < branches; i++) {
    double q1 = t1s[i];
    if (!fitLimit(q1, lo[0], hi[0])) continue;

    const double dy = rs[i] - D1_Y, dz = p.z - D1_Z;
    const double cosG = (dy * dy + dz * dz - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (fabs(cosG) > 1.0 + 1e-9) continue;
    const double g = acos(clampTo(cosG, -1.0, 1.0));

    for (int e = 0; e < 2; e++) {
      const double gamma = e == 0 ? g : -g;
      double q3 = wrap180((gamma - gamma0) * DEG);
      if (!fitLimit(q3, lo[2], hi[2])) continue;

      const double alpha1 = atan2(dz, dy) - atan2(L2 * sin(gamma), L1 + L2 * cos(gamma));
      double q2 = wrap180((alpha1 - alpha10) * DEG);
      if (!fitLimit(q2, lo[1], hi[1])) continue;

      double q4 = p.a - q2 - q3;
      if (!fitLimit(q4, lo[3], hi[3])) continue;
      double q5 = p.b;
      if (!fitLimit(q5, lo[4], hi[4])) continue;

      // Squared: J2+J3+J4 is fixed by the pitch, so plain differences often tie exactly
      // and rounding would pick the elbow.
      const double d[JOINT_COUNT] = {q1 - seed[0], q2 - seed[1], q3 - seed[2], q4 - seed[3], q5 - seed[4]};
      double score = 0.0;
      for (int j = 0; j < JOINT_COUNT; j++) score += d[j] * d[j];
      if (!found || score < bestScore) {
        found = true;
        bestScore = score;
        out[0] = q1; out[1] = q2; out[2] = q3; out[3] = q4; out[4] = q5;
      }
    }
  }
  return found;
}

// Minimum-jerk: speed and acceleration are both zero at each end of the move.
inline double profile(double tau) {
  if (tau <= 0.0) return 0.0;
  if (tau >= 1.0) return 1.0;
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}

inline Pose lerpPose(const Pose &a, const Pose &b, double s) {
  Pose p;
  p.x = a.x + (b.x - a.x) * s;
  p.y = a.y + (b.y - a.y) * s;
  p.z = a.z + (b.z - a.z) * s;
  p.a = a.a + (b.a - a.a) * s;
  p.b = a.b + (b.b - a.b) * s;
  return p;
}

enum PlanResult { PLAN_OK = 0, PLAN_REACH, PLAN_PATH };

struct Plan {
  bool linear;
  double duration;      // seconds; 0 = nothing to do
  double start[JOINT_COUNT], end[JOINT_COUNT];
  Pose startPose, endPose;
};

// linear = straight wrist line (G1), otherwise every joint runs straight to its end (G0).
inline PlanResult planMove(const double start[JOINT_COUNT], const Pose &target, bool linear,
                           double speedMmS, const double lo[JOINT_COUNT], const double hi[JOINT_COUNT],
                           const double ratio[JOINT_COUNT], double maxServoSpeed, Plan &plan) {
  plan.linear = linear;
  plan.endPose = target;
  fk(start, plan.startPose);
  for (int j = 0; j < JOINT_COUNT; j++) plan.start[j] = start[j];
  if (!ik(target, start, lo, hi, plan.end)) return PLAN_REACH;

  double peak = 0.0;   // largest servo-degrees across the whole move, at a constant rate
  if (linear) {
    double prev[JOINT_COUNT], q[JOINT_COUNT];
    for (int j = 0; j < JOINT_COUNT; j++) prev[j] = start[j];
    for (int k = 1; k <= PATH_SAMPLES; k++) {
      if (!ik(lerpPose(plan.startPose, target, (double)k / PATH_SAMPLES), prev, lo, hi, q)) return PLAN_PATH;
      for (int j = 0; j < JOINT_COUNT; j++) {
        const double step = fabs(q[j] - prev[j]);
        if (step > PATH_MAX_SAMPLE_STEP_DEG) return PLAN_PATH;
        const double rate = step * PATH_SAMPLES * fabs(ratio[j]);
        if (rate > peak) peak = rate;
        prev[j] = q[j];
      }
    }
  } else {
    for (int j = 0; j < JOINT_COUNT; j++) {
      const double travel = fabs(plan.end[j] - start[j]) * fabs(ratio[j]);
      if (travel > peak) peak = travel;
    }
  }

  const double length = sqrt((target.x - plan.startPose.x) * (target.x - plan.startPose.x) +
                             (target.y - plan.startPose.y) * (target.y - plan.startPose.y) +
                             (target.z - plan.startPose.z) * (target.z - plan.startPose.z));
  if (peak < 1e-3 && length < 1e-3) {
    plan.duration = 0.0;
    return PLAN_OK;
  }
  double t = PROFILE_PEAK * peak / maxServoSpeed;
  if (speedMmS > 0.0 && length / speedMmS > t) t = length / speedMmS;
  plan.duration = t < MIN_DURATION_S ? MIN_DURATION_S : t;
  return PLAN_OK;
}

// Joint angles `t` seconds into a plan. `q` holds the previous angles on the way in, and
// keeps them if a straight-line point fails (the plan has already checked the path).
inline void planAngles(const Plan &plan, double t, const double lo[JOINT_COUNT],
                       const double hi[JOINT_COUNT], double q[JOINT_COUNT]) {
  if (plan.duration <= 0.0 || t >= plan.duration) {
    for (int j = 0; j < JOINT_COUNT; j++) q[j] = plan.end[j];
    return;
  }
  const double s = profile(t / plan.duration);
  if (!plan.linear) {
    for (int j = 0; j < JOINT_COUNT; j++) q[j] = plan.start[j] + (plan.end[j] - plan.start[j]) * s;
    return;
  }
  double next[JOINT_COUNT];
  if (ik(lerpPose(plan.startPose, plan.endPose, s), q, lo, hi, next)) {
    for (int j = 0; j < JOINT_COUNT; j++) q[j] = next[j];
  }
}

}  // namespace kin
