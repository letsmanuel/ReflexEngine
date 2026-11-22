#include "reflex/character.h"

#include <algorithm>
#include <cmath>

#include "character_impl.h"
#include "character_tables.h"

namespace reflex {

CharacterDesc CharacterDesc::DefaultHumanoid(float height, float mass) {
    CharacterDesc d;
    HumanoidRig rig = MakeDefaultHumanoidRig(height);
    d.skeleton = std::move(rig.skeleton);
    d.mapping = rig.mapping;
    d.params.height = height;
    d.params.mass = mass;
    return d;
}

// ---------------------------------------------------------------------------------------------------------
// Public handle API (thin wrappers)
// ---------------------------------------------------------------------------------------------------------
bool Character::IsValid() const { return impl_ && impl_->backend != nullptr; }
void Character::SetAnimationPose(const Pose& pose) { impl_->SetAnimationPose(pose); }
void Character::ApplyHit(const HitEvent& hit) { impl_->ApplyHit(hit); }
void Character::ReportNearMiss(const NearMissEvent& miss) { impl_->ReportNearMiss(miss); }
void Character::SetGlobalAuthority(float v, float t) { impl_->authority.SetGlobal(v, t); }
void Character::SetAuthority(BodyPart p, float v, float t) { impl_->authority.SetPart(p, v, t); }
void Character::SetAnimationWeight(float v, float t) { impl_->authority.SetAnimationWeight(v, t); }
void Character::ReleaseToAnimation(float t) { impl_->authority.ReleaseToAnimation(t); }
void Character::TakeProceduralControl(float t) { impl_->authority.TakeProceduralControl(t); }
float Character::GetGlobalAuthority() const { return impl_->authority.Global(); }
float Character::GetAuthority(BodyPart p) const { return impl_->authority.Part(p); }
float Character::GetAnimationWeight() const { return impl_->authority.AnimationWeight(); }
void Character::GetOutputPose(Pose& out) const { impl_->GetOutputPose(out); }
Transform Character::GetBodyPartTransform(BodyPart p) const {
    return impl_->backend->GetBodyState(impl_->parts[Index(p)].body).transform;
}
const CharacterDebugInfo& Character::GetDebugInfo() const { return impl_->debug; }
ShapeDesc Character::GetPartShape(BodyPart p) const { return impl_->parts[Index(p)].shape; }
void Character::Reset() { impl_->Reset(); }
void Character::Teleport(const Transform& t) { impl_->Teleport(t); }
bool Character::RayCastParts(const Vec3& o, const Vec3& d, float maxD, BodyPart& part, Vec3& pt, float& dist) const {
    return impl_->RayCastParts(o, d, maxD, part, pt, dist);
}
float Character::ClosestApproach(const Vec3& o, const Vec3& d, float maxD, BodyPart& part, Vec3& pt) const {
    return impl_->ClosestApproach(o, d, maxD, part, pt);
}
InjuryModel& Character::Injury() { return impl_->injury; }
const InjuryModel& Character::Injury() const { return impl_->injury; }
ThreatModel& Character::Threat() { return impl_->threat; }
DecisionLayer& Character::Decisions() { return impl_->decisions; }
AuthorityBlender& Character::Authority() { return impl_->authority; }
const CharacterDesc& Character::Desc() const { return impl_->desc; }

// ---------------------------------------------------------------------------------------------------------
// Construction of the ragdoll
// ---------------------------------------------------------------------------------------------------------
namespace {

Quat QuatFromBasis(const Vec3& x, const Vec3& y, const Vec3& z) {
    // Rotation matrix with columns x, y, z -> quaternion.
    float m00 = x.x, m01 = y.x, m02 = z.x;
    float m10 = x.y, m11 = y.y, m12 = z.y;
    float m20 = x.z, m21 = y.z, m22 = z.z;
    float tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0) {
        float s = std::sqrt(tr + 1.0f) * 2;
        q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        float s = std::sqrt(1.0f + m00 - m11 - m22) * 2;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    } else if (m11 > m22) {
        float s = std::sqrt(1.0f + m11 - m00 - m22) * 2;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
    } else {
        float s = std::sqrt(1.0f + m22 - m00 - m11) * 2;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
    }
    return q.Normalized();
}

float SegmentSegmentDistance(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, float& s, float& t) {
    Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    float a = Dot(d1, d1), e = Dot(d2, d2), f = Dot(d2, r);
    if (a <= 1e-9f && e <= 1e-9f) { s = t = 0; return Length(r); }
    if (a <= 1e-9f) {
        s = 0;
        t = Saturate(f / e);
    } else {
        float c = Dot(d1, r);
        if (e <= 1e-9f) {
            t = 0;
            s = Saturate(-c / a);
        } else {
            float b = Dot(d1, d2);
            float denom = a * e - b * b;
            s = denom > 1e-9f ? Saturate((b * f - c * e) / denom) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0) { t = 0; s = Saturate(-c / a); }
            else if (t > 1) { t = 1; s = Saturate((b - c) / a); }
        }
    }
    Vec3 c1 = p1 + d1 * s, c2 = p2 + d2 * t;
    return Length(c1 - c2);
}

} // namespace

void Character::Impl::Build(IPhysicsBackend* be, std::uint32_t charId, const CharacterDesc& d) {
    backend = be;
    id = charId;
    desc = d;
    injury = InjuryModel(d.injury);
    spawn = d.spawn;

    // --- Bind pose joint positions in model space ---
    std::vector<Transform> bindWorld;
    Pose bindPose;
    for (const BoneDesc& b : desc.skeleton.bones) bindPose.local.push_back(b.bindLocal);
    PoseToWorld(desc.skeleton, bindPose, bindWorld);
    for (int i = 0; i < kBodyPartCount; ++i) {
        int bi = desc.mapping.bone[i];
        parts[i].jointBind = bindWorld[static_cast<size_t>(bi)].position;
        parts[i].boneBindRot = bindWorld[static_cast<size_t>(bi)].rotation;
    }
    auto J = [&](BodyPart p) -> const Vec3& { return parts[Index(p)].jointBind; };

    // Scale from the measured ankle-to-head distance (1.43 m for the reference proportions).
    scale = std::max(0.1f, (J(BodyPart::Head).y - J(BodyPart::LeftFoot).y) / 1.43f);
    const float s = scale;

    // --- Segment geometry ---
    auto capsuleBetween = [&](BodyPart p, const Vec3& a, const Vec3& b, float radius) {
        PartData& pd = parts[Index(p)];
        pd.centerBind = (a + b) * 0.5f;
        float len = Length(a - b);
        pd.length = len;
        pd.shape.type = ShapeType::Capsule;
        pd.shape.radius = radius;
        pd.shape.halfHeight = std::max(0.005f, len * 0.5f - radius);
    };
    auto boxAt = [&](BodyPart p, const Vec3& center, const Vec3& half) {
        PartData& pd = parts[Index(p)];
        pd.centerBind = center;
        pd.shape.type = ShapeType::Box;
        pd.shape.halfExtents = half;
        pd.length = half.y * 2;
    };
    auto boxBetween = [&](BodyPart p, const Vec3& a, const Vec3& b, float hx, float hz) {
        boxAt(p, (a + b) * 0.5f, {hx, Length(a - b) * 0.5f, hz});
    };

    boxAt(BodyPart::Pelvis, J(BodyPart::Pelvis), {0.15f * s, 0.08f * s, 0.10f * s});
    boxBetween(BodyPart::Spine, J(BodyPart::Spine), J(BodyPart::Chest), 0.13f * s, 0.09f * s);
    boxBetween(BodyPart::Chest, J(BodyPart::Chest), J(BodyPart::Neck), 0.17f * s, 0.10f * s);
    capsuleBetween(BodyPart::Neck, J(BodyPart::Neck), J(BodyPart::Head), 0.045f * s);
    {
        PartData& h = parts[Index(BodyPart::Head)];
        h.shape.type = ShapeType::Sphere;
        h.shape.radius = 0.11f * s;
        h.centerBind = J(BodyPart::Head) + Vec3{0, 0.11f * s, 0};
        h.length = 0.22f * s;
    }
    for (Side sd : {Side::Left, Side::Right}) {
        capsuleBetween(UpperArm(sd), J(UpperArm(sd)), J(LowerArm(sd)), 0.045f * s);
        capsuleBetween(LowerArm(sd), J(LowerArm(sd)), J(Hand(sd)), 0.04f * s);
        boxAt(Hand(sd), J(Hand(sd)) + Vec3{0, -0.05f * s, 0}, {0.04f * s, 0.05f * s, 0.02f * s});
        capsuleBetween(UpperLeg(sd), J(UpperLeg(sd)), J(LowerLeg(sd)), 0.07f * s);
        capsuleBetween(LowerLeg(sd), J(LowerLeg(sd)), J(Foot(sd)), 0.055f * s);
        boxAt(Foot(sd), J(Foot(sd)) + Vec3{0, -0.05f * s, 0.05f * s}, {0.05f * s, 0.04f * s, 0.13f * s});
    }

    // --- Bodies ---
    totalMass = 0;
    float fracSum = 0;
    for (int i = 0; i < kBodyPartCount; ++i) fracSum += tables::MassFraction(static_cast<BodyPart>(i));
    const Quat sr = spawn.rotation;
    for (int i = 0; i < kBodyPartCount; ++i) {
        BodyPart p = static_cast<BodyPart>(i);
        PartData& pd = parts[i];
        pd.mass = desc.params.mass * tables::MassFraction(p) / fracSum;
        totalMass += pd.mass;
        BodyDesc bd;
        bd.shape = pd.shape;
        bd.mass = pd.mass;
        bd.transform.position = spawn.TransformPoint(pd.centerBind);
        bd.transform.rotation = sr;
        bd.friction = (p == BodyPart::LeftFoot || p == BodyPart::RightFoot) ? 1.2f * desc.params.friction : 0.8f;
        bd.angularDamping = 0.15f;
        bd.linearDamping = 0.03f;
        bd.collisionGroup = id;
        bd.subGroup = static_cast<std::uint32_t>(i);
        bd.userData = (static_cast<std::uint64_t>(id) << 8) | static_cast<std::uint64_t>(i + 1);
        pd.body = backend->CreateBody(bd);
        // joint anchor expressed in the body's own frame (bodies are axis-aligned with the spawn rotation at bind)
        pd.jointInChild = pd.jointBind - pd.centerBind;
        pd.jointInParent = (p == BodyPart::Pelvis) ? Vec3{} : pd.jointBind - parts[Index(ParentOf(p))].centerBind;
    }
    // --- Joints ---
    for (int i = 1; i < kBodyPartCount; ++i) {
        BodyPart p = static_cast<BodyPart>(i);
        PartData& pd = parts[i];
        tables::JointSpec spec = tables::SpecFor(p);
        JointDesc jd;
        jd.type = spec.type;
        jd.parent = parts[Index(ParentOf(p))].body;
        jd.child = pd.body;
        jd.anchor = spawn.TransformPoint(pd.jointBind);
        bool vertical = IsTorsoPart(p) || IsHeadPart(p);
        if (spec.type == JointType::Hinge) {
            jd.axis = sr.Rotate({1, 0, 0});
            jd.normalAxis = sr.Rotate({0, 0, 1});
            // Hinge zero angle reference: normal axis must be perpendicular to the hinge axis.
        } else {
            jd.axis = sr.Rotate(vertical ? Vec3{0, 1, 0} : Vec3{0, -1, 0});
            jd.normalAxis = sr.Rotate({1, 0, 0});
        }
        jd.minAngle = spec.minA;
        jd.maxAngle = spec.maxA;
        jd.swingNormal = spec.swingA;
        jd.swingPlane = spec.swingB;
        pd.joint = backend->CreateJoint(jd);
    }
    // --- Self collision filtering ---
    for (int i = 1; i < kBodyPartCount; ++i) {
        BodyPart p = static_cast<BodyPart>(i);
        backend->SetCollisionPairEnabled(parts[i].body, parts[Index(ParentOf(p))].body, false);
    }
    backend->SetCollisionPairEnabled(parts[Index(BodyPart::Pelvis)].body, parts[Index(BodyPart::Chest)].body, false);
    backend->SetCollisionPairEnabled(parts[Index(BodyPart::Spine)].body, parts[Index(BodyPart::Neck)].body, false);
    backend->SetCollisionPairEnabled(parts[Index(BodyPart::Chest)].body, parts[Index(BodyPart::Head)].body, false);

    // --- Leg geometry for IK ---
    for (Side sd : {Side::Left, Side::Right}) {
        int i = SideIndex(sd);
        thighLen[i] = Length(J(UpperLeg(sd)) - J(LowerLeg(sd)));
        shinLen[i] = Length(J(LowerLeg(sd)) - J(Foot(sd)));
        hipInPelvis[i] = J(UpperLeg(sd)) - parts[Index(BodyPart::Pelvis)].centerBind;
    }
    ankleHeight = 0.09f * s;
    standPelvisHeight = J(BodyPart::Pelvis).y; // height of the pelvis centre above the ground

    ResetState();
}

void Character::Impl::ResetState() {
    injury.Reset();
    threat.Reset();
    authority.Reset();
    decisions.Reset();
    hasAnim = false;
    time = 0;
    baseYaw = 0;
    {
        Vec3 f = Horizontal(spawn.rotation.Rotate({0, 0, 1}));
        baseYaw = std::atan2(f.x, f.z);
    }
    curYaw = baseYaw;
    for (int i = 0; i < 2; ++i) {
        leg[i] = Leg{};
        Side sd = i == 0 ? Side::Left : Side::Right;
        Vec3 ankle = spawn.TransformPoint(parts[Index(Foot(sd))].jointBind);
        leg[i].planted = Vec3{ankle.x, 0, ankle.z};
        leg[i].target = leg[i].planted;
        leg[i].start = leg[i].planted;
        leg[i].contact = true;
        leg[i].groundY = spawn.position.y;
    }
    lastSwing = -1;
    stepCooldown = 0;
    debug = CharacterDebugInfo{};
    prevTargets = ReactionTargets{};
    effLean = 0;
    smoothedMoveSpeed = 0;
    smoothedMoveDir = Vec3{};
    for (auto& v : smoothedOffset.v) v = Vec3{};
}

void Character::Impl::Reset() {
    for (int i = 0; i < kBodyPartCount; ++i) {
        BodyState st;
        st.transform.position = spawn.TransformPoint(parts[i].centerBind);
        st.transform.rotation = spawn.rotation;
        backend->SetBodyState(parts[i].body, st);
    }
    ResetState();
}

void Character::Impl::Teleport(const Transform& t) {
    spawn = t;
    Reset();
}

// ---------------------------------------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------------------------------------
void Character::Impl::ApplyHit(const HitEvent& hit) {
    injury.ApplyDamage(hit.bodyPart, hit.damage);
    threat.RegisterHit(hit);
    float impulse = hit.impulse > 0.0f ? hit.impulse : hit.mass * hit.speed;
    if (impulse > 0.0f) {
        PartData& pd = parts[Index(hit.bodyPart)];
        Vec3 imp = Normalize(hit.direction, {0, 0, 1}) * (impulse * desc.hitImpulseScale);
        float limit = pd.mass * desc.maxHitVelocityChange;
        imp = ClampLength(imp, limit);
        backend->AddImpulse(pd.body, imp, hit.point);
    }
}

void Character::Impl::ReportNearMiss(const NearMissEvent& miss) {
    BodyState pel = backend->GetBodyState(parts[Index(BodyPart::Pelvis)].body);
    BodyState head = backend->GetBodyState(parts[Index(BodyPart::Head)].body);
    threat.RegisterNearMiss(miss, Transform{pel.transform.position, pel.transform.rotation}, head.transform.position);
}

void Character::Impl::SetAnimationPose(const Pose& pose) {
    animPose = pose;
    PoseToWorld(desc.skeleton, animPose, animWorld);
    hasAnim = true;
}

// ---------------------------------------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------------------------------------
bool Character::Impl::RayCastParts(const Vec3& origin, const Vec3& dir, float maxDist, BodyPart& part, Vec3& point,
                                   float& dist) const {
    RayHit h;
    if (!backend->RayCast(origin, dir, maxDist, false, h)) return false;
    if ((h.userData >> 8) != id || (h.userData & 0xFF) == 0) return false;
    part = static_cast<BodyPart>((h.userData & 0xFF) - 1);
    point = h.point;
    dist = h.distance;
    return true;
}

float Character::Impl::ClosestApproach(const Vec3& origin, const Vec3& dir, float maxDist, BodyPart& outPart,
                                       Vec3& outPoint) const {
    Vec3 d = Normalize(dir, {0, 0, 1});
    Vec3 a = origin, b = origin + d * maxDist;
    float best = 1e9f;
    for (int i = 0; i < kBodyPartCount; ++i) {
        const PartData& pd = parts[i];
        BodyState st = backend->GetBodyState(pd.body);
        Vec3 axis = st.transform.rotation.Rotate({0, 1, 0});
        float half = 0.0f, radius = pd.shape.radius;
        if (pd.shape.type == ShapeType::Capsule) half = pd.shape.halfHeight;
        else if (pd.shape.type == ShapeType::Box) {
            half = pd.shape.halfExtents.y;
            radius = std::max(pd.shape.halfExtents.x, pd.shape.halfExtents.z) * 0.8f;
        }
        Vec3 p0 = st.transform.position - axis * half, p1 = st.transform.position + axis * half;
        float s, t;
        float dd = SegmentSegmentDistance(a, b, p0, p1, s, t) - radius;
        if (dd < best) {
            best = dd;
            outPart = static_cast<BodyPart>(i);
            outPoint = a + (b - a) * s;
        }
    }
    return std::max(0.0f, best);
}

// ---------------------------------------------------------------------------------------------------------
// Output pose
// ---------------------------------------------------------------------------------------------------------
void Character::Impl::GetOutputPose(Pose& out) const {
    const auto& bones = desc.skeleton.bones;
    std::vector<Transform> world(bones.size());
    std::vector<bool> done(bones.size(), false);
    for (int i = 0; i < kBodyPartCount; ++i) {
        int bi = desc.mapping.bone[i];
        BodyState st = backend->GetBodyState(parts[i].body);
        Transform w;
        w.position = st.transform.TransformPoint(parts[i].jointInChild);
        w.rotation = (st.transform.rotation * parts[i].boneBindRot).Normalized();
        world[static_cast<size_t>(bi)] = w;
        done[static_cast<size_t>(bi)] = true;
    }
    out.local.resize(bones.size());
    for (size_t i = 0; i < bones.size(); ++i) {
        int p = bones[i].parent;
        if (!done[i]) {
            if (p < 0) {
                world[i] = spawn;
            } else {
                const Transform& pw = world[static_cast<size_t>(p)];
                world[i].position = pw.position + pw.rotation.Rotate(bones[i].bindLocal.position);
                world[i].rotation = (pw.rotation * bones[i].bindLocal.rotation).Normalized();
            }
        }
        if (p < 0) {
            out.local[i] = world[i];
        } else {
            const Transform& pw = world[static_cast<size_t>(p)];
            out.local[i].position = pw.rotation.Conjugate().Rotate(world[i].position - pw.position);
            out.local[i].rotation = (pw.rotation.Conjugate() * world[i].rotation).Normalized();
        }
    }
}

} // namespace reflex
