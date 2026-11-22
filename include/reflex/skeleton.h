#pragma once
/// @file skeleton.h
/// Skeleton / rig description, bone -> body-part mapping, poses and the humanoid ragdoll layout.

#include <string>
#include <vector>

#include "reflex/export.h"
#include "reflex/math.h"
#include "reflex/types.h"

namespace reflex {

/// One bone of the game's skeleton. `bindLocal` is the bind (rest) pose relative to the parent bone;
/// for a root bone (parent < 0) it is expressed in character/model space.
struct BoneDesc {
    std::string name;
    int parent = -1;
    Transform bindLocal;
};

struct SkeletonDesc {
    std::vector<BoneDesc> bones;  ///< parents must come before children
    int FindBone(const std::string& name) const {
        for (size_t i = 0; i < bones.size(); ++i)
            if (bones[i].name == name) return static_cast<int>(i);
        return -1;
    }
};

/// Mapping from the engine's humanoid parts to bone indices of the game skeleton (-1 = unmapped).
/// A mapped bone's origin is the joint at which that part connects to its parent part.
struct BodyMapping {
    int bone[kBodyPartCount];
    BodyMapping() { for (int& b : bone) b = -1; }
    int& operator[](BodyPart p) { return bone[static_cast<int>(p)]; }
    int operator[](BodyPart p) const { return bone[static_cast<int>(p)]; }
    /// Map a part by bone name; returns false if the bone does not exist.
    bool Set(const SkeletonDesc& skel, BodyPart p, const std::string& boneName) {
        int i = skel.FindBone(boneName);
        if (i < 0) return false;
        (*this)[p] = i;
        return true;
    }
    bool IsComplete() const {
        for (int b : bone) if (b < 0) return false;
        return true;
    }
};

/// A pose: one local transform per bone of the skeleton (parent-relative; model/world space for root bones).
struct Pose {
    std::vector<Transform> local;
};

/// Convert a local pose into world-space bone transforms.
REFLEX_API void PoseToWorld(const SkeletonDesc& skel, const Pose& pose, std::vector<Transform>& outWorld);

/// Physical proportions of the humanoid ragdoll.
struct HumanoidParams {
    float height = 1.75f;     ///< standing height in metres
    float mass = 75.0f;       ///< total mass in kg
    float strength = 1.0f;    ///< scales all joint torque limits
    float friction = 1.0f;
};

/// A ready-made humanoid: 17-bone skeleton + mapping + bind pose, used by tests and the demo when the game
/// has no rig of its own.
struct HumanoidRig {
    SkeletonDesc skeleton;
    BodyMapping mapping;
};
REFLEX_API HumanoidRig MakeDefaultHumanoidRig(float height = 1.75f);

} // namespace reflex
