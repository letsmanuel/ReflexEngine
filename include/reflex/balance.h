#pragma once
/// @file balance.h
/// Balance math: centre of mass, support polygon, capture point (inverted pendulum), weight sharing
/// between legs and asymmetric (limping) gait parameters. All functions are pure and unit tested.
/// Everything operates on the ground plane (XZ); Y is ignored unless stated otherwise.

#include "reflex/export.h"
#include "reflex/math.h"
#include "reflex/types.h"

namespace reflex {

/// Mass-weighted centre of a set of points.
REFLEX_API Vec3 WeightedCentre(const Vec3* points, const float* masses, int count);

/// Instantaneous capture point of a linear inverted pendulum: the ground point where the foot must be
/// placed to bring the centre of mass to rest. xi = x + v * sqrt(h / g).
REFLEX_API Vec3 CapturePoint(const Vec3& com, const Vec3& comVelocity, float comHeight, float gravity = kGravity);

/// Convex support polygon on the ground plane (at most 16 vertices).
struct SupportPolygon {
    static constexpr int kMaxPoints = 16;
    Vec3 points[kMaxPoints];
    int count = 0;

    /// Build the convex hull of the supplied ground contact points.
    REFLEX_API static SupportPolygon FromPoints(const Vec3* pts, int n);
    /// Signed distance of p to the polygon boundary on the XZ plane: negative inside, positive outside.
    /// Degenerate polygons (point / segment) have no inside.
    REFLEX_API float SignedDistance(const Vec3& p) const;
    bool Contains(const Vec3& p) const { return count >= 3 && SignedDistance(p) <= 0.0f; }
    /// Closest point of the polygon (including its interior) to p.
    REFLEX_API Vec3 ClosestPoint(const Vec3& p) const;
    REFLEX_API Vec3 Centroid() const;
};

/// Fraction of body weight each leg should carry. Proportional to support capacity, shifted by
/// `bias` (-1..1, positive = lean on the left leg).
struct LegLoads {
    float left = 0.5f, right = 0.5f;
    /// 0 = legs can carry the body together, > 0 = combined capacity is too low (the body will sag).
    float deficit = 0.0f;
};
REFLEX_API LegLoads ComputeLegLoads(float capacityLeft, float capacityRight, float bias = 0.0f);

/// Foot placement for a recovery step. Returns the ground target for the swing foot: the capture point,
/// pushed outwards by `margin`, pulled back to within `maxReach` of the stance foot.
REFLEX_API Vec3 ComputeStepTarget(const Vec3& capturePoint, const Vec3& stanceFoot, const Vec3& comVelocityDir,
                                  float maxReach, float margin);

/// Asymmetric gait parameters derived from per-leg support capacity.
struct GaitParams {
    float stepLengthScale[2] = {1, 1};   ///< per leg (SideIndex): swing step length multiplier
    float swingTimeScale[2] = {1, 1};    ///< per leg: swing duration multiplier (shorter = quicker step)
    float stanceTimeScale[2] = {1, 1};   ///< per leg: how long the leg may stay loaded
    bool drag[2] = {false, false};       ///< leg too weak to swing normally: drag / hop instead
    float speedScale = 1.0f;             ///< overall walking speed multiplier
};
REFLEX_API GaitParams ComputeGait(float capacityLeft, float capacityRight);

} // namespace reflex
