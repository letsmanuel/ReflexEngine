// The control loop of a character: perceive -> assess -> decide -> act.

#include <algorithm>
#include <cmath>

#include "character_impl.h"
#include "character_tables.h"

namespace reflex {
namespace {

constexpr float kWalkSpeed = 1.2f; // m/s at moveSpeed = 1

inline float ExpBlend(float dt, float rate) { return 1.0f - std::exp(-dt * rate); }

Quat YawQuat(float yaw) { return Quat::FromAxisAngle({0, 1, 0}, yaw); }

Quat BasisQuat(const Vec3& x, const Vec3& y, const Vec3& z) {
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

struct LegIK {
    Quat thigh;        ///< desired world rotation of the upper leg body
    float kneeFlex = 0;///< desired knee flexion (rad, >= 0 bends the shin backwards)
};

/// Two-bone IK in the leg plane. Returns the desired thigh orientation and knee flexion so that the ankle
/// reaches `ankleTarget` from `hip`. `fwd` and `left` are the horizontal heading axes.
LegIK SolveLeg(const Vec3& hip, const Vec3& ankleTarget, float a, float b, const Vec3& fwd, const Vec3& left) {
    Vec3 v = ankleTarget - hip;
    float dist = Length(v);
    float maxReach = (a + b) * 0.998f;
    float d = Clamp(dist, std::abs(a - b) + 0.02f, maxReach);
    Vec3 dirHA = dist > 1e-5f ? v / dist : Vec3{0, -1, 0};
    float cosBeta = Clamp((a * a + d * d - b * b) / (2 * a * d), -1.0f, 1.0f);
    float beta = std::acos(cosBeta);
    float cosGamma = Clamp((a * a + b * b - d * d) / (2 * a * b), -1.0f, 1.0f);
    float knee = kPi - std::acos(cosGamma);
    Vec3 thighDir = Quat::FromAxisAngle(left, -beta).Rotate(dirHA);
    Vec3 y = -thighDir;
    Vec3 z = fwd - y * Dot(fwd, y);
    z = Normalize(z, Cross(left, y));
    Vec3 x = Cross(y, z);
    LegIK out;
    out.thigh = BasisQuat(x, y, z);
    out.kneeFlex = knee;
    return out;
}

} // namespace

void Character::Impl::Tick(float dt) {
    time += dt;
    injury.Update(dt);
    threat.Update(dt);
    authority.Update(dt);

    const float s = scale;
    const Vec3 up{0, 1, 0};

    // ---- 1. Perceive: read the physical state ------------------------------------------------------------
    BodyState st[kBodyPartCount];
    for (int i = 0; i < kBodyPartCount; ++i) st[i] = backend->GetBodyState(parts[i].body);
    const BodyState& pel = st[Index(BodyPart::Pelvis)];
    const BodyState& chest = st[Index(BodyPart::Chest)];

    Vec3 fwdP = Horizontal(pel.transform.rotation.Rotate({0, 0, 1}));
    if (LengthSq(fwdP) < 1e-4f) fwdP = Horizontal(chest.transform.rotation.Rotate({0, 0, 1}));
    fwdP = Normalize(fwdP, {0, 0, 1});
    const Quat headingActual = FromTo({0, 0, 1}, fwdP);

    // Feet: ground height and contact
    Vec3 corners[8];
    int cornerCount = 0;
    bool anyContact = false;
    for (Side sd : {Side::Left, Side::Right}) {
        const int i = SideIndex(sd);
        const PartData& pd = parts[Index(Foot(sd))];
        const BodyState& fs = st[Index(Foot(sd))];
        Leg& L = leg[i];
        L.ankle = fs.transform.TransformPoint(pd.jointInChild);
        RayHit gh;
        if (backend->RayCast(L.ankle + Vec3{0, 0.6f, 0}, {0, -1, 0}, 3.0f, true, gh)) L.groundY = gh.point.y;
        float minY = 1e9f;
        Vec3 c4[4];
        int k = 0;
        for (float sx : {-1.0f, 1.0f})
            for (float sz : {-1.0f, 1.0f}) {
                Vec3 local{sx * pd.shape.halfExtents.x, -pd.shape.halfExtents.y, sz * pd.shape.halfExtents.z};
                Vec3 w = fs.transform.TransformPoint(local);
                c4[k++] = w;
                minY = std::min(minY, w.y);
            }
        L.contact = (minY - L.groundY) < 0.04f;
        if (L.contact) anyContact = true;
        if (L.contact && !L.swinging)
            for (int q = 0; q < 4 && cornerCount < 8; ++q) corners[cornerCount++] = c4[q];
    }
    const float groundRef = 0.5f * (leg[0].groundY + leg[1].groundY);

    // Centre of mass
    Vec3 com, comVel;
    for (int i = 0; i < kBodyPartCount; ++i) {
        com += st[i].transform.position * parts[i].mass;
        comVel += st[i].linearVelocity * parts[i].mass;
    }
    com = com / totalMass;
    comVel = comVel / totalMass;
    const float comHeight = com.y - groundRef;
    const Vec3 cp = CapturePoint(com, comVel, std::max(comHeight, 0.25f));
    const SupportPolygon poly = SupportPolygon::FromPoints(corners, cornerCount);
    const float margin = poly.count > 0 ? poly.SignedDistance(cp) : 1.0f;

    // Body tilt: how far the whole body is leaning over (not just a hunched chest): the larger of the pelvis
    // tilt and the angle of the line from the support point to the centre of mass.
    Vec3 pelvisUp = pel.transform.rotation.Rotate(up);
    Vec3 supportPt;
    {
        int n = 0;
        for (int i = 0; i < 2; ++i) {
            supportPt += Vec3{leg[i].ankle.x, 0, leg[i].ankle.z};
            ++n;
        }
        supportPt = supportPt / static_cast<float>(n);
    }
    Vec3 comOffset = Horizontal(com) - supportPt;
    float comTilt = std::atan2(Length(comOffset), std::max(comHeight, 0.05f));
    float pelvisTilt = std::acos(Clamp(pelvisUp.y, -1.0f, 1.0f));
    float tilt = std::max(comTilt, pelvisTilt);
    Vec3 tiltDir = Normalize(pelvisTilt > comTilt ? Horizontal(pelvisUp) : comOffset, fwdP);

    // ---- 2. Assess + decide -----------------------------------------------------------------------------
    BodyStateSummary sum;
    sum.com = com;
    sum.comVelocity = comVel;
    sum.capturePoint = cp;
    sum.comHeight = comHeight;
    sum.standingComHeight = standPelvisHeight * 0.98f;
    sum.supportMargin = margin;
    sum.footContact[0] = leg[0].contact;
    sum.footContact[1] = leg[1].contact;
    sum.tilt = tilt;
    sum.tiltDirection = tiltDir;
    sum.speed = Length(Horizontal(comVel));
    sum.heading = headingActual;
    sum.position = pel.transform.position;

    Transform frame{pel.transform.position, headingActual};
    ThreatPicture picture = threat.Evaluate(frame, st[Index(BodyPart::Head)].transform.position);
    BehaviorContext ctx{injury, picture, sum, time};
    const Decision& decision = decisions.Decide(ctx, dt);
    const ReactionTargets& T = decision.targets;

    const float control = injury.ControlLevel();
    const float animW = hasAnim ? authority.AnimationWeight() : 0.0f;
    const float capL = injury.SupportCapacity(Side::Left), capR = injury.SupportCapacity(Side::Right);
    const GaitParams gait = ComputeGait(capL, capR);

    // ---- 3. Smooth the targets so behaviours blend in and out instead of snapping ---------------------------
    const float kb = ExpBlend(dt, 9.0f);
    sCrouch += (T.crouch - sCrouch) * kb;
    sStance += (T.stanceWidth - sStance) * kb;
    sTurn += (T.turnYaw - sTurn) * ExpBlend(dt, 6.0f);
    Vec3 leanTarget = T.lean > 0.0f ? Normalize(T.leanDirection, {0, 0, 1}) * T.lean : Vec3{};
    sLean += (leanTarget - sLean) * ExpBlend(dt, 7.0f);
    Vec3 moveVelTarget = T.moveDirection * (T.moveSpeed * T.speedLimit * kWalkSpeed * s);
    sMoveVel += (moveVelTarget - sMoveVel) * ExpBlend(dt, 3.0f);
    for (int i = 0; i < kBodyPartCount; ++i) {
        BodyPart p = static_cast<BodyPart>(i);
        smoothedOffset[p] += (T.jointOffset[p] - smoothedOffset[p]) * kb;
    }
    float desiredYaw = baseYaw + sTurn;
    curYaw += (desiredYaw - curYaw) * ExpBlend(dt, 8.0f);
    const Quat yawQ = YawQuat(curYaw);
    const Vec3 fwd = yawQ.Rotate({0, 0, 1});
    const Vec3 left = yawQ.Rotate({1, 0, 0});

    // ---- 4. Stepping state machine --------------------------------------------------------------------------
    stepCooldown = std::max(0.0f, stepCooldown - dt);
    bool anySwing = false;
    for (int i = 0; i < 2; ++i) {
        Leg& L = leg[i];
        if (L.swinging) {
            anySwing = true;
            L.t += dt;
            if (L.t >= L.duration) {
                L.swinging = false;
                L.planted = L.target;
                L.stanceTimer = 0;
                stepCooldown = 0.10f;
                lastSwing = i;
            }
        } else {
            L.stanceTimer += dt;
            // Re-plant if the foot has slid away from where it was supposed to stay.
            Vec3 drift = Horizontal(L.ankle) - Horizontal(L.planted);
            if (L.contact && Length(drift) > 0.18f * s) L.planted = Vec3{L.ankle.x, 0, L.ankle.z};
        }
    }
    const bool canStep = anyContact && control > 0.12f && stepCooldown <= 0.0f && !anySwing &&
                         leg[0].contact && leg[1].contact && T.collapse < 0.5f;
    const float stepThreshold = T.wantStep ? -0.045f : -0.015f;
    if (canStep && margin > stepThreshold && animW < 0.99f) {
        // Choose the swing leg: the step target for each candidate, penalising leaning on a weak stance leg.
        int best = -1;
        float bestCost = 1e9f;
        Vec3 bestTarget;
        for (int sw = 0; sw < 2; ++sw) {
            int stance = 1 - sw;
            float capStance = stance == 0 ? capL : capR;
            float reach = (thighLen[sw] + shinLen[sw]) * 0.62f * gait.stepLengthScale[sw];
            Vec3 velDir = Horizontal(comVel);
            Vec3 tgt = ComputeStepTarget(cp, leg[stance].planted, velDir, reach, 0.05f * s);
            // keep the feet from crossing over
            Vec3 loc = headingActual.Conjugate().Rotate(tgt - leg[stance].planted);
            float minSep = (0.10f * s) + 0.5f * sStance;
            if (sw == 0) loc.x = std::max(loc.x, minSep);
            else loc.x = std::min(loc.x, -minSep);
            tgt = leg[stance].planted + headingActual.Rotate(loc);
            // How far from the capture point would the foot land? (the step should catch the fall)
            float cost = Length(Horizontal(tgt) - Horizontal(cp));
            cost += 0.5f * (1.0f - capStance) * (1.0f - capStance); // avoid standing on a weak leg
            cost += (sw == lastSwing) ? 0.05f : 0.0f;               // alternate legs
            if (cost < bestCost) {
                bestCost = cost;
                best = sw;
                bestTarget = tgt;
            }
        }
        if (best >= 0) {
            Leg& L = leg[best];
            L.swinging = true;
            L.t = 0.0f;
            L.start = Vec3{L.ankle.x, 0, L.ankle.z};
            L.target = Vec3{bestTarget.x, 0, bestTarget.z};
            L.duration = std::max(0.16f, 0.30f * gait.swingTimeScale[best]);
            L.lift = (gait.drag[best] ? 0.015f : 0.075f) * s;
            anySwing = true;
        }
    }

    // ---- 5. Weight distribution -----------------------------------------------------------------------------
    LegLoads loads = ComputeLegLoads(capL, capR, T.legLoadBias);
    float w[2] = {loads.left, loads.right};
    for (int i = 0; i < 2; ++i)
        if (leg[i].swinging || !leg[i].contact) w[i] = 0.0f;
    float wsum = w[0] + w[1];
    if (wsum > 1e-4f) {
        w[0] /= wsum;
        w[1] /= wsum;
    }
    const bool supported = wsum > 1e-4f;
    const float assist = Saturate(control * T.assistScale * authority.Global()) * (supported ? 1.0f : 0.0f);
    const bool lowAssist = assist < 0.02f;

    for (int i = 0; i < 2; ++i) injury.ReportLegLoad(i == 0 ? Side::Left : Side::Right, w[i], dt);

    // ---- 6. Act: balance assist forces ------------------------------------------------------------------------
    const float W = totalMass * kGravity;
    const BodyHandle pelvisBody = parts[Index(BodyPart::Pelvis)].body;
    float legStrengthLoss = 1.0f - 0.8f * loads.deficit;
    if (!lowAssist) {
        // Height: a virtual leg spring between pelvis and the supporting feet (net force on the body is zero).
        float hTarget = groundRef + standPelvisHeight - (0.02f + 0.30f * sCrouch) * s;
        float Fup = W * 0.90f + 2600.0f * (hTarget - pel.transform.position.y) * (totalMass / 75.0f) -
                    420.0f * (totalMass / 75.0f) * pel.linearVelocity.y;
        Fup = Clamp(Fup, 0.0f, 1.8f * W) * assist * legStrengthLoss;
        backend->AddForce(pelvisBody, {0, Fup, 0});
        for (int i = 0; i < 2; ++i)
            if (w[i] > 0) backend->AddForce(parts[Index(Foot(i == 0 ? Side::Left : Side::Right))].body, {0, -Fup * w[i], 0});

        // Horizontal: accelerate the COM towards the commanded velocity, or hold it over the support point.
        Vec3 support;
        for (int i = 0; i < 2; ++i) support += Vec3{leg[i].ankle.x, 0, leg[i].ankle.z} * w[i];
        support += fwd * (0.05f * s);
        support += fwd * (-0.05f * s * sCrouch);
        Vec3 comH = Horizontal(com);
        Vec3 velH = Horizontal(comVel);
        Vec3 accel;
        if (Length(sMoveVel) > 0.05f * s) {
            accel = (sMoveVel - velH) * 3.5f;
            // keep the COM laterally centred on the support while walking
        } else {
            accel = (Vec3{support.x, 0, support.z} - comH) * 22.0f - velH * 6.5f;
        }
        accel = ClampLength(accel, 0.45f * kGravity);
        Vec3 Fh = accel * (totalMass * assist);
        backend->AddForce(pelvisBody, Fh);
        for (int i = 0; i < 2; ++i)
            if (w[i] > 0) backend->AddForce(parts[Index(Foot(i == 0 ? Side::Left : Side::Right))].body, -Fh * w[i]);

        // Upright / heading torque on the pelvis.
        Quat qd = yawQ;
        if (animW > 0.0f && !animWorld.empty()) {
            const Quat animPel = (animWorld[static_cast<size_t>(desc.mapping[BodyPart::Pelvis])].rotation *
                                  parts[Index(BodyPart::Pelvis)].boneBindRot.Conjugate()).Normalized();
            qd = Slerp(yawQ, animPel, animW);
        }
        Vec3 err = (qd * pel.transform.rotation.Conjugate()).ToRotationVector();
        float mScale = totalMass / 75.0f;
        Vec3 tau = err * (900.0f * mScale) - pel.angularVelocity * (110.0f * mScale);
        tau = ClampLength(tau, 700.0f * mScale) * assist;
        backend->AddTorque(pelvisBody, tau);
    }

    // ---- 7. Act: joint targets ---------------------------------------------------------------------------------
    PerPart<Quat> delta;
    for (auto& q : delta.v) q = Quat::Identity();

    // Lean (limited by how close the capture point is to the edge of the support polygon).
    float leanLimit = Saturate((-margin - 0.02f) / 0.06f);
    Vec3 leanVec = sLean * leanLimit;
    Quat leanQ = Quat::Identity();
    if (Length(leanVec) > 1e-4f) leanQ = Quat::FromAxisAngle(Cross(up, Normalize(leanVec)), Length(leanVec));
    effLean = Length(leanVec);
    const Quat chestDes = (leanQ * yawQ).Normalized();

    const Quat qPel = pel.transform.rotation;
    Quat relSpine = (qPel.Conjugate() * chestDes).Normalized();
    Quat half = Slerp(Quat::Identity(), relSpine, 0.5f);
    delta[BodyPart::Spine] = Quat::FromRotationVector(smoothedOffset[BodyPart::Spine]) * half;
    delta[BodyPart::Chest] = Quat::FromRotationVector(smoothedOffset[BodyPart::Chest]) * half;

    {
        const Quat qChest = chest.transform.rotation;
        Quat headDes = Slerp(yawQ, chestDes, 0.4f);
        Quat relNeck = (qChest.Conjugate() * headDes).Normalized();
        delta[BodyPart::Neck] = Quat::FromRotationVector(smoothedOffset[BodyPart::Neck]) * Slerp(Quat::Identity(), relNeck, 0.5f);
        const Quat qNeck = st[Index(BodyPart::Neck)].transform.rotation;
        Quat relHead = (qNeck.Conjugate() * headDes).Normalized();
        delta[BodyPart::Head] = Quat::FromRotationVector(smoothedOffset[BodyPart::Head]) * Slerp(Quat::Identity(), relHead, 0.7f);
    }

    for (Side sd : {Side::Left, Side::Right}) {
        const float sg = SideSign(sd);
        delta[UpperArm(sd)] = Quat::FromRotationVector(smoothedOffset[UpperArm(sd)] + Vec3{0, 0, sg * 0.12f});
        delta[LowerArm(sd)] = Quat::FromRotationVector(smoothedOffset[LowerArm(sd)] + Vec3{-0.25f, 0, 0});
        delta[Hand(sd)] = Quat::FromRotationVector(smoothedOffset[Hand(sd)]);
    }

    // Legs by inverse kinematics.
    {
        for (Side sd : {Side::Left, Side::Right}) {
            const int i = SideIndex(sd);
            Leg& L = leg[i];
            Vec3 px = L.planted;
            float lift = 0.0f;
            if (L.swinging) {
                float u = Saturate(L.t / L.duration);
                float e = SmoothStep(u);
                px = Lerp(L.start, L.target, e);
                lift = std::sin(kPi * u) * L.lift;
            }
            Vec3 ankleTarget{px.x, L.groundY + ankleHeight + lift, px.z};
            Vec3 hip = pel.transform.TransformPoint(hipInPelvis[i]);
            LegIK ik = SolveLeg(hip, ankleTarget, thighLen[i], shinLen[i], fwd, left);
            delta[UpperLeg(sd)] = (qPel.Conjugate() * ik.thigh).Normalized();
            delta[LowerLeg(sd)] = Quat::FromAxisAngle({1, 0, 0}, ik.kneeFlex);
            // keep the foot flat on the ground in heading direction
            const Quat qShin = st[Index(LowerLeg(sd))].transform.rotation;
            Quat footDes = yawQ;
            delta[Foot(sd)] = (qShin.Conjugate() * footDes).Normalized();
        }
    }

    // Blend with the game's animation pose, if any.
    if (animW > 0.0f && !animWorld.empty()) {
        PerPart<Quat> desiredWorld;
        for (int i = 0; i < kBodyPartCount; ++i) {
            BodyPart p = static_cast<BodyPart>(i);
            size_t bi = static_cast<size_t>(desc.mapping[p]);
            desiredWorld[p] = (animWorld[bi].rotation * parts[i].boneBindRot.Conjugate()).Normalized();
        }
        for (int i = 1; i < kBodyPartCount; ++i) {
            BodyPart p = static_cast<BodyPart>(i);
            Quat animRel = (desiredWorld[ParentOf(p)].Conjugate() * desiredWorld[p]).Normalized();
            delta[p] = Slerp(delta[p], animRel, animW);
        }
    }

    // Drives: stiffness, damping and torque limit scale with the effective authority of each joint.
    const float strength = desc.params.strength * (totalMass / 75.0f);
    for (int i = 1; i < kBodyPartCount; ++i) {
        BodyPart p = static_cast<BodyPart>(i);
        // Core muscles keep most of their strength after a wound, limbs lose it in proportion to capacity.
        float cap = injury.Capacity(p);
        if (IsTorsoPart(p)) cap = 0.55f + 0.45f * cap;
        float a = Saturate(authority.Effective(p, control, cap) * T.authorityScale[p]);
        tables::DriveBase base = tables::DriveFor(p);
        JointDrive d;
        d.target = delta[p];
        if (a < 0.02f) {
            d.stiffness = 0.0f;
        } else {
            d.stiffness = base.k * a * strength;
            d.damping = base.d * a * strength;
            d.maxTorque = base.maxT * a * strength;
        }
        backend->SetJointDrive(parts[i].joint, d);
        debug.authority[p] = a;
    }

    // ---- 8. Debug snapshot ----------------------------------------------------------------------------------------
    for (int i = 0; i < kBodyPartCount; ++i) debug.parts.v[static_cast<size_t>(i)] = injury.Part(static_cast<BodyPart>(i));
    debug.supportCapacity[0] = capL;
    debug.supportCapacity[1] = capR;
    debug.legLoad[0] = w[0];
    debug.legLoad[1] = w[1];
    debug.armCapacity[0] = injury.ArmCapacity(Side::Left);
    debug.armCapacity[1] = injury.ArmCapacity(Side::Right);
    debug.shock = injury.Shock();
    debug.consciousness = injury.Consciousness();
    debug.control = control;
    debug.vitality = injury.Vitality();
    debug.fatigue = injury.Fatigue();
    debug.com = com;
    debug.comVelocity = comVel;
    debug.capturePoint = cp;
    debug.supportPolygon = poly;
    for (int i = 0; i < 2; ++i) {
        debug.footContact[i] = leg[i].contact;
        debug.stepping[i] = leg[i].swinging;
        debug.footTarget[i] = leg[i].swinging ? leg[i].target : leg[i].planted;
    }
    debug.tilt = tilt;
    debug.supportMargin = margin;
    debug.fallen = comHeight < 0.45f * standPelvisHeight || tilt > 1.25f;
    debug.activeBehaviors = decision.active;
    debug.reason = decision.reason;
    debug.targets = T;
    prevTargets = T;
}

void Character::Impl::Destroy() {
    if (!backend) return;
    for (int i = 1; i < kBodyPartCount; ++i)
        if (parts[i].joint != kInvalidHandle) backend->DestroyJoint(parts[i].joint);
    for (int i = 0; i < kBodyPartCount; ++i)
        if (parts[i].body != kInvalidHandle) backend->DestroyBody(parts[i].body);
    backend = nullptr;
}

} // namespace reflex
