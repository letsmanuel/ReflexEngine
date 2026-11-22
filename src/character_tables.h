#pragma once
// Internal: per-joint limits, drive strengths and mass distribution of the humanoid ragdoll.

#include "reflex/physics_backend.h"
#include "reflex/types.h"

namespace reflex {
namespace tables {

struct JointSpec {
    JointType type;
    float swingA, swingB; ///< ball: swing half angles; hinge unused
    float minA, maxA;     ///< ball: twist limits; hinge: angle limits
};

inline JointSpec SpecFor(BodyPart p) {
    switch (p) {
        case BodyPart::Spine: return {JointType::Ball, 0.35f, 0.35f, -0.35f, 0.35f};
        case BodyPart::Chest: return {JointType::Ball, 0.35f, 0.35f, -0.35f, 0.35f};
        case BodyPart::Neck: return {JointType::Ball, 0.5f, 0.5f, -0.6f, 0.6f};
        case BodyPart::Head: return {JointType::Ball, 0.6f, 0.6f, -0.9f, 0.9f};
        case BodyPart::LeftUpperArm:
        case BodyPart::RightUpperArm: return {JointType::Ball, 1.9f, 1.9f, -1.2f, 1.2f};
        case BodyPart::LeftLowerArm:
        case BodyPart::RightLowerArm: return {JointType::Hinge, 0, 0, -2.6f, 0.05f};
        case BodyPart::LeftHand:
        case BodyPart::RightHand: return {JointType::Ball, 0.5f, 0.5f, -0.2f, 0.2f};
        case BodyPart::LeftUpperLeg:
        case BodyPart::RightUpperLeg: return {JointType::Ball, 1.4f, 1.4f, -0.6f, 0.6f};
        case BodyPart::LeftLowerLeg:
        case BodyPart::RightLowerLeg: return {JointType::Hinge, 0, 0, 0.0f, 2.5f};
        case BodyPart::LeftFoot:
        case BodyPart::RightFoot: return {JointType::Ball, 0.55f, 0.55f, -0.35f, 0.35f};
        default: return {JointType::Ball, 0.5f, 0.5f, -0.5f, 0.5f};
    }
}

struct DriveBase {
    float k, d, maxT;
};

inline DriveBase DriveFor(BodyPart p) {
    switch (p) {
        case BodyPart::Spine: return {900, 90, 300};
        case BodyPart::Chest: return {700, 70, 240};
        case BodyPart::Neck: return {80, 8, 40};
        case BodyPart::Head: return {60, 6, 30};
        case BodyPart::LeftUpperArm:
        case BodyPart::RightUpperArm: return {160, 16, 90};
        case BodyPart::LeftLowerArm:
        case BodyPart::RightLowerArm: return {90, 9, 60};
        case BodyPart::LeftHand:
        case BodyPart::RightHand: return {45, 5, 16};
        case BodyPart::LeftUpperLeg:
        case BodyPart::RightUpperLeg: return {1000, 100, 420};
        case BodyPart::LeftLowerLeg:
        case BodyPart::RightLowerLeg: return {800, 80, 380};
        case BodyPart::LeftFoot:
        case BodyPart::RightFoot: return {200, 20, 120};
        default: return {0, 0, 0};
    }
}

/// Passive joint friction (N*m): keeps unpowered limbs, and especially the hands, from flopping about.
inline float FrictionFor(BodyPart p) {
    switch (p) {
        case BodyPart::LeftHand:
        case BodyPart::RightHand: return 1.6f;
        case BodyPart::Neck:
        case BodyPart::Head: return 1.5f;
        case BodyPart::LeftFoot:
        case BodyPart::RightFoot: return 2.0f;
        case BodyPart::LeftLowerArm:
        case BodyPart::RightLowerArm:
        case BodyPart::LeftUpperArm:
        case BodyPart::RightUpperArm: return 2.5f;
        default: return 4.0f;
    }
}

inline float MassFraction(BodyPart p) {
    switch (p) {
        case BodyPart::Pelvis: return 11.0f;
        case BodyPart::Spine: return 7.0f;
        case BodyPart::Chest: return 18.0f;
        case BodyPart::Neck: return 1.0f;
        case BodyPart::Head: return 5.0f;
        case BodyPart::LeftUpperArm:
        case BodyPart::RightUpperArm: return 2.1f;
        case BodyPart::LeftLowerArm:
        case BodyPart::RightLowerArm: return 1.4f;
        case BodyPart::LeftHand:
        case BodyPart::RightHand: return 0.5f;
        case BodyPart::LeftUpperLeg:
        case BodyPart::RightUpperLeg: return 9.0f;
        case BodyPart::LeftLowerLeg:
        case BodyPart::RightLowerLeg: return 3.5f;
        case BodyPart::LeftFoot:
        case BodyPart::RightFoot: return 1.0f;
        default: return 1.0f;
    }
}


} // namespace tables
} // namespace reflex
