#pragma once
/// @file types.h
/// Humanoid body-part model shared by every module.

#include <array>
#include <cstdint>

#include "reflex/export.h"

namespace reflex {

/// The engine's fixed humanoid body-part set. Game skeletons are mapped onto it (see skeleton.h).
enum class BodyPart : std::uint8_t {
    Pelvis,
    Spine,
    Chest,
    Neck,
    Head,
    LeftUpperArm,
    LeftLowerArm,
    LeftHand,
    RightUpperArm,
    RightLowerArm,
    RightHand,
    LeftUpperLeg,
    LeftLowerLeg,
    LeftFoot,
    RightUpperLeg,
    RightLowerLeg,
    RightFoot,
    Count
};
constexpr int kBodyPartCount = static_cast<int>(BodyPart::Count);

/// Side of the body. Left is +X in the character's local frame.
enum class Side : std::uint8_t { Left, Right };
inline Side Opposite(Side s) { return s == Side::Left ? Side::Right : Side::Left; }
inline int SideIndex(Side s) { return s == Side::Left ? 0 : 1; }
/// +1 for left (+X), -1 for right (-X).
inline float SideSign(Side s) { return s == Side::Left ? 1.0f : -1.0f; }

inline int Index(BodyPart p) { return static_cast<int>(p); }

constexpr bool IsLegPart(BodyPart p) { return p >= BodyPart::LeftUpperLeg; }
constexpr bool IsArmPart(BodyPart p) { return p >= BodyPart::LeftUpperArm && p <= BodyPart::RightHand; }
constexpr bool IsTorsoPart(BodyPart p) { return p <= BodyPart::Chest; }
constexpr bool IsHeadPart(BodyPart p) { return p == BodyPart::Neck || p == BodyPart::Head; }

/// Side a limb part belongs to. Midline parts report Left (check IsLimb first).
constexpr bool IsLimb(BodyPart p) { return p >= BodyPart::LeftUpperArm; }
constexpr Side SideOf(BodyPart p) {
    return (p >= BodyPart::LeftUpperArm && p <= BodyPart::LeftHand) ||
                   (p >= BodyPart::LeftUpperLeg && p <= BodyPart::LeftFoot)
               ? Side::Left
               : Side::Right;
}

constexpr BodyPart UpperLeg(Side s) { return s == Side::Left ? BodyPart::LeftUpperLeg : BodyPart::RightUpperLeg; }
constexpr BodyPart LowerLeg(Side s) { return s == Side::Left ? BodyPart::LeftLowerLeg : BodyPart::RightLowerLeg; }
constexpr BodyPart Foot(Side s) { return s == Side::Left ? BodyPart::LeftFoot : BodyPart::RightFoot; }
constexpr BodyPart UpperArm(Side s) { return s == Side::Left ? BodyPart::LeftUpperArm : BodyPart::RightUpperArm; }
constexpr BodyPart LowerArm(Side s) { return s == Side::Left ? BodyPart::LeftLowerArm : BodyPart::RightLowerArm; }
constexpr BodyPart Hand(Side s) { return s == Side::Left ? BodyPart::LeftHand : BodyPart::RightHand; }

/// Parent of a part in the ragdoll hierarchy (Pelvis is its own parent = root).
constexpr BodyPart ParentOf(BodyPart p) {
    switch (p) {
        case BodyPart::Pelvis: return BodyPart::Pelvis;
        case BodyPart::Spine: return BodyPart::Pelvis;
        case BodyPart::Chest: return BodyPart::Spine;
        case BodyPart::Neck: return BodyPart::Chest;
        case BodyPart::Head: return BodyPart::Neck;
        case BodyPart::LeftUpperArm: case BodyPart::RightUpperArm: return BodyPart::Chest;
        case BodyPart::LeftLowerArm: return BodyPart::LeftUpperArm;
        case BodyPart::RightLowerArm: return BodyPart::RightUpperArm;
        case BodyPart::LeftHand: return BodyPart::LeftLowerArm;
        case BodyPart::RightHand: return BodyPart::RightLowerArm;
        case BodyPart::LeftUpperLeg: case BodyPart::RightUpperLeg: return BodyPart::Pelvis;
        case BodyPart::LeftLowerLeg: return BodyPart::LeftUpperLeg;
        case BodyPart::RightLowerLeg: return BodyPart::RightUpperLeg;
        case BodyPart::LeftFoot: return BodyPart::LeftLowerLeg;
        case BodyPart::RightFoot: return BodyPart::RightLowerLeg;
        default: return BodyPart::Pelvis;
    }
}

REFLEX_API const char* BodyPartName(BodyPart p);

/// A value per body part.
template <typename T>
struct PerPart {
    std::array<T, kBodyPartCount> v{};
    T& operator[](BodyPart p) { return v[static_cast<size_t>(p)]; }
    const T& operator[](BodyPart p) const { return v[static_cast<size_t>(p)]; }
    void Fill(const T& value) { v.fill(value); }
};

} // namespace reflex
