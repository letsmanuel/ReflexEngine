#include "reflex/skeleton.h"

namespace reflex {

void PoseToWorld(const SkeletonDesc& skel, const Pose& pose, std::vector<Transform>& out) {
    out.resize(skel.bones.size());
    for (size_t i = 0; i < skel.bones.size(); ++i) {
        const Transform& l = i < pose.local.size() ? pose.local[i] : skel.bones[i].bindLocal;
        int p = skel.bones[i].parent;
        if (p < 0) {
            out[i] = l;
        } else {
            const Transform& pw = out[static_cast<size_t>(p)];
            out[i].position = pw.position + pw.rotation.Rotate(l.position);
            out[i].rotation = (pw.rotation * l.rotation).Normalized();
        }
    }
}

HumanoidRig MakeDefaultHumanoidRig(float height) {
    const float s = height / 1.75f;
    struct Def {
        BodyPart part;
        const char* name;
        Vec3 pos; ///< bind world position of the bone origin (metres at 1.75 m height)
    };
    const Def defs[] = {
        {BodyPart::Pelvis, "pelvis", {0, 0.97f, 0}},
        {BodyPart::Spine, "spine", {0, 1.05f, 0}},
        {BodyPart::Chest, "chest", {0, 1.17f, 0}},
        {BodyPart::Neck, "neck", {0, 1.45f, 0}},
        {BodyPart::Head, "head", {0, 1.52f, 0}},
        {BodyPart::LeftUpperArm, "upperarm_l", {0.21f, 1.38f, 0}},
        {BodyPart::LeftLowerArm, "lowerarm_l", {0.21f, 1.09f, 0}},
        {BodyPart::LeftHand, "hand_l", {0.21f, 0.82f, 0}},
        {BodyPart::RightUpperArm, "upperarm_r", {-0.21f, 1.38f, 0}},
        {BodyPart::RightLowerArm, "lowerarm_r", {-0.21f, 1.09f, 0}},
        {BodyPart::RightHand, "hand_r", {-0.21f, 0.82f, 0}},
        {BodyPart::LeftUpperLeg, "thigh_l", {0.09f, 0.93f, 0}},
        {BodyPart::LeftLowerLeg, "calf_l", {0.09f, 0.49f, 0}},
        {BodyPart::LeftFoot, "foot_l", {0.09f, 0.09f, 0}},
        {BodyPart::RightUpperLeg, "thigh_r", {-0.09f, 0.93f, 0}},
        {BodyPart::RightLowerLeg, "calf_r", {-0.09f, 0.49f, 0}},
        {BodyPart::RightFoot, "foot_r", {-0.09f, 0.09f, 0}},
    };

    HumanoidRig rig;
    Vec3 world[kBodyPartCount];
    for (const Def& d : defs) {
        BoneDesc b;
        b.name = d.name;
        BodyPart par = ParentOf(d.part);
        world[Index(d.part)] = d.pos * s;
        if (d.part == BodyPart::Pelvis) {
            b.parent = -1;
            b.bindLocal.position = world[Index(d.part)];
        } else {
            b.parent = rig.mapping[par];
            b.bindLocal.position = world[Index(d.part)] - world[Index(par)];
        }
        rig.mapping[d.part] = static_cast<int>(rig.skeleton.bones.size());
        rig.skeleton.bones.push_back(std::move(b));
    }
    return rig;
}

} // namespace reflex
