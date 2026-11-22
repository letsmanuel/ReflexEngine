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

std::array<Vec3, kBodyPartCount> DefaultHumanoidJointPositions(float height) {
    const float s = height / 1.75f;
    std::array<Vec3, kBodyPartCount> p;
    p[Index(BodyPart::Pelvis)] = {0, 0.97f, 0};
    p[Index(BodyPart::Spine)] = {0, 1.05f, 0};
    p[Index(BodyPart::Chest)] = {0, 1.17f, 0};
    p[Index(BodyPart::Neck)] = {0, 1.45f, 0};
    p[Index(BodyPart::Head)] = {0, 1.52f, 0};
    p[Index(BodyPart::LeftUpperArm)] = {0.21f, 1.38f, 0};
    p[Index(BodyPart::LeftLowerArm)] = {0.21f, 1.09f, 0};
    p[Index(BodyPart::LeftHand)] = {0.21f, 0.82f, 0};
    p[Index(BodyPart::RightUpperArm)] = {-0.21f, 1.38f, 0};
    p[Index(BodyPart::RightLowerArm)] = {-0.21f, 1.09f, 0};
    p[Index(BodyPart::RightHand)] = {-0.21f, 0.82f, 0};
    p[Index(BodyPart::LeftUpperLeg)] = {0.09f, 0.93f, 0};
    p[Index(BodyPart::LeftLowerLeg)] = {0.09f, 0.49f, 0};
    p[Index(BodyPart::LeftFoot)] = {0.09f, 0.09f, 0};
    p[Index(BodyPart::RightUpperLeg)] = {-0.09f, 0.93f, 0};
    p[Index(BodyPart::RightLowerLeg)] = {-0.09f, 0.49f, 0};
    p[Index(BodyPart::RightFoot)] = {-0.09f, 0.09f, 0};
    for (Vec3& v : p) v = v * s;
    return p;
}

HumanoidRig MakeHumanoidRig(const std::array<Vec3, kBodyPartCount>& pos) {
    static const char* names[kBodyPartCount] = {"pelvis", "spine", "chest", "neck", "head", "upperarm_l", "lowerarm_l",
                                                "hand_l", "upperarm_r", "lowerarm_r", "hand_r", "thigh_l", "calf_l",
                                                "foot_l", "thigh_r", "calf_r", "foot_r"};
    HumanoidRig rig;
    // BodyPart order lists parents before children, so a single pass suffices.
    for (int i = 0; i < kBodyPartCount; ++i) {
        BodyPart part = static_cast<BodyPart>(i);
        BoneDesc b;
        b.name = names[i];
        if (part == BodyPart::Pelvis) {
            b.parent = -1;
            b.bindLocal.position = pos[static_cast<size_t>(i)];
        } else {
            BodyPart par = ParentOf(part);
            b.parent = rig.mapping[par];
            b.bindLocal.position = pos[static_cast<size_t>(i)] - pos[static_cast<size_t>(Index(par))];
        }
        rig.mapping[part] = static_cast<int>(rig.skeleton.bones.size());
        rig.skeleton.bones.push_back(std::move(b));
    }
    return rig;
}

HumanoidRig MakeDefaultHumanoidRig(float height) { return MakeHumanoidRig(DefaultHumanoidJointPositions(height)); }

} // namespace reflex
