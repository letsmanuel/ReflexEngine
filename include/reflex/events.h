#pragma once
/// @file events.h
/// Perception inputs: confirmed hits and near misses.

#include "reflex/math.h"
#include "reflex/types.h"

namespace reflex {

/// A projectile or impact that struck the character.
struct HitEvent {
    BodyPart bodyPart = BodyPart::Chest;
    Vec3 point;               ///< world-space impact point
    Vec3 direction{0, 0, 1};  ///< world-space direction the projectile was travelling (unit)
    float impulse = 0.0f;     ///< N*s delivered to the body part. If 0, mass * speed is used.
    float speed = 0.0f;       ///< projectile speed in m/s (optional)
    float mass = 0.0f;        ///< projectile mass in kg (optional)
    float damage = 0.0f;      ///< damage in health points (a limb has ~100, see InjuryConfig)
};

/// A projectile that passed close to the character without hitting it.
struct NearMissEvent {
    Vec3 closestPoint;        ///< world-space point on the projectile path closest to the body
    Vec3 direction{0, 0, 1};  ///< world-space travel direction of the projectile (unit)
    float distance = 0.5f;    ///< closest approach to the body surface in metres
    BodyPart nearestPart = BodyPart::Chest; ///< body part the projectile passed closest to
};

} // namespace reflex
