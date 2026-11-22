#include "reflex/behavior.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace reflex {
namespace {

constexpr float kNoLimpAbove = 0.75f; ///< support capacity above which a leg is considered fine

Vec3 ToWorld(const BehaviorContext& c, const Vec3& local) { return c.body.heading.Rotate(local); }

/// Offset helpers. Rotation vectors are expressed in the parent frame of the joint.
/// Rest pose: arms and legs hang straight down. Raising an arm forward is a negative X rotation, abducting
/// an arm outwards is Z rotation by SideSign, bending an elbow is a negative X rotation, a forward nod
/// or hunch is a positive X rotation.
void RaiseArms(ReactionTargets& t, float forward, float out, float elbow, float w) {
    for (Side s : {Side::Left, Side::Right}) {
        t.jointOffset[UpperArm(s)] += Vec3{-forward * w, 0, SideSign(s) * out * w};
        t.jointOffset[LowerArm(s)] += Vec3{-elbow * w, 0, 0};
    }
}

float LegCap(const BehaviorContext& c, Side s) { return c.injury.SupportCapacity(s); }

// ---------------------------------------------------------------------------------------------------------
class Brace final : public Behavior {
public:
    const char* Name() const override { return "Brace"; }
    Channel GetChannel() const override { return Channel::Recovery; }
    float Score(const BehaviorContext& c) const override {
        float t = c.injury.TimeSinceLastHit();
        if (t > 2.5f) return 0.0f;
        float dmg = Saturate(c.injury.LastHitDamage() / 40.0f);
        return Saturate(0.15f + 0.6f * dmg) * (1.0f - t / 2.5f);
    }
    void Apply(const BehaviorContext&, float w, ReactionTargets& o, std::string& clause) const override {
        o.crouch = std::max(o.crouch, 0.25f * w);
        o.stanceWidth += 0.08f * w;
        o.speedLimit = std::min(o.speedLimit, 1.0f - 0.5f * w);
        clause += "brace: widen stance and lower the centre of mass";
    }
};

// ---------------------------------------------------------------------------------------------------------
class StumbleRecover final : public Behavior {
public:
    const char* Name() const override { return "StumbleRecover"; }
    Channel GetChannel() const override { return Channel::Recovery; }
    float Score(const BehaviorContext& c) const override {
        float s = Saturate((c.body.supportMargin - 0.02f) / 0.15f);
        s = std::max(s, Saturate((c.body.tilt - 0.18f) / 0.4f) * 0.8f);
        return s * Saturate(c.injury.ControlLevel() * 2.0f);
    }
    void Apply(const BehaviorContext&, float w, ReactionTargets& o, std::string& clause) const override {
        o.wantStep = true;
        o.crouch = std::max(o.crouch, 0.15f * w);
        o.stanceWidth += 0.10f * w;
        RaiseArms(o, 0.2f, 0.9f, 0.3f, w); // arms out to balance
        clause += "stumble recovery: step to catch the fall, arms out, lower the centre of mass";
    }
};

// ---------------------------------------------------------------------------------------------------------
class LimpAway final : public Behavior {
public:
    const char* Name() const override { return "LimpAway"; }
    Channel GetChannel() const override { return Channel::Locomotion; }
    float Score(const BehaviorContext& c) const override {
        Side weak = c.injury.WeakerLeg();
        float cap = LegCap(c, weak);
        if (cap >= kNoLimpAbove) return 0.0f;
        float weakness = 1.0f - cap / kNoLimpAbove;
        float urgency = c.threat.active ? 0.5f + 0.5f * c.threat.recency : 0.25f;
        return Saturate(weakness * urgency * 1.1f);
    }
    void Apply(const BehaviorContext& c, float w, ReactionTargets& o, std::string& clause) const override {
        Side weak = c.injury.WeakerLeg();
        Side strong = Opposite(weak);
        float cap = LegCap(c, weak);
        GaitParams gait = ComputeGait(LegCap(c, Side::Left), LegCap(c, Side::Right));

        o.limping = true;
        o.limpSide = weak;
        o.limpAmount = std::max(o.limpAmount, 1.0f - cap);
        o.legLoadBias += SideSign(strong) * (1.0f - cap) * w;
        o.crouch = std::max(o.crouch, (0.2f + 0.25f * (1.0f - cap)) * w);
        o.stanceWidth += 0.04f * w;

        char buf[64];
        std::snprintf(buf, sizeof buf, "shift weight to %s, limp", strong == Side::Left ? "left" : "right");
        clause += buf;

        if (c.threat.active) {
            Vec3 dir;
            if (c.threat.hasCover && c.threat.coverDistance > 0.5f) {
                Vec3 toCover = Normalize(Horizontal(c.threat.coverPoint - c.body.position), c.threat.fireDirection);
                Vec3 lateral = Cross({0, 1, 0}, c.threat.fireDirection);
                if (Dot(lateral, toCover) < 0) lateral = -lateral;
                dir = Normalize(toCover * 0.75f + lateral * 0.25f);
                clause += ", move to cover";
            } else {
                // Leave the line of fire: sidestep with the healthy leg's push-off plus a bit of retreat.
                Vec3 lateral = Cross({0, 1, 0}, c.threat.fireDirection);
                Vec3 strongSide = ToWorld(c, {SideSign(strong), 0, 0});
                if (Dot(lateral, strongSide) < 0) lateral = -lateral;
                dir = Normalize(lateral * 0.7f + c.threat.fireDirection * 0.5f);
                clause += ", move away from fire";
            }
            o.moveDirection = dir;
            o.moveSpeed = std::max(o.moveSpeed, gait.speedScale * w);
            o.crouch = std::max(o.crouch, 0.3f * w);
            clause += ", crouch for cover";
        }
    }
};

// ---------------------------------------------------------------------------------------------------------
class SeekCover final : public Behavior {
public:
    const char* Name() const override { return "SeekCover"; }
    Channel GetChannel() const override { return Channel::Locomotion; }
    float Score(const BehaviorContext& c) const override {
        if (!c.threat.active) return 0.0f;
        float base = 0.30f + 0.35f * c.threat.recency * c.threat.confidence;
        if (!c.threat.hasCover) base *= 0.75f;
        return Saturate(base * Saturate(c.injury.ControlLevel() * 1.5f));
    }
    void Apply(const BehaviorContext& c, float w, ReactionTargets& o, std::string& clause) const override {
        if (c.threat.hasCover && c.threat.coverDistance > 0.7f) {
            o.moveDirection = Normalize(Horizontal(c.threat.coverPoint - c.body.position), c.threat.fireDirection);
            o.moveSpeed = std::max(o.moveSpeed, 0.85f * w);
            o.crouch = std::max(o.crouch, 0.35f * w);
            clause += "run to cover, stay low";
        } else if (c.threat.hasCover) {
            o.crouch = std::max(o.crouch, 0.7f * w);
            clause += "behind cover, duck low";
        } else {
            Vec3 lateral = Cross({0, 1, 0}, c.threat.fireDirection);
            if (Dot(lateral, ToWorld(c, {1, 0, 0})) < 0) lateral = -lateral;
            o.moveDirection = Normalize(lateral * 0.6f + c.threat.fireDirection * 0.4f);
            o.moveSpeed = std::max(o.moveSpeed, 0.6f * w);
            o.crouch = std::max(o.crouch, 0.45f * w);
            clause += "no cover: leave the line of fire, duck low";
        }
    }
};

// ---------------------------------------------------------------------------------------------------------
class TurnAndLeanAway final : public Behavior {
public:
    const char* Name() const override { return "TurnAndLeanAway"; }
    Channel GetChannel() const override { return Channel::Posture; }
    float Score(const BehaviorContext& c) const override {
        float s = 0.0f;
        if (c.threat.nearMissActive)
            s = 0.25f + 0.65f * c.threat.nearMissProximity * c.threat.nearMissRecency;
        Side weakArm = c.injury.WeakerArm();
        float armCap = c.injury.ArmCapacity(weakArm);
        if (armCap < 0.7f && c.threat.active)
            s = std::max(s, 0.6f * (1.0f - armCap / 0.7f) * c.threat.recency);
        return Saturate(s * Saturate(c.injury.ControlLevel() * 2.0f));
    }
    void Apply(const BehaviorContext& c, float w, ReactionTargets& o, std::string& clause) const override {
        if (c.threat.nearMissActive &&
            0.25f + 0.65f * c.threat.nearMissProximity * c.threat.nearMissRecency >= 0.25f) {
            Side side = c.threat.nearMissSide;
            // Lean away from the side the bullet passed, and turn the face away from it.
            Vec3 localLean{-SideSign(side), 0.0f, -0.25f};
            o.leanDirection = Normalize(ToWorld(c, localLean));
            o.lean = std::max(o.lean, 0.30f * w);
            o.turnYaw += -SideSign(side) * 0.6f * w;
            o.jointOffset[BodyPart::Head] += Vec3{0, -SideSign(side) * 0.35f * w, 0};
            clause += side == Side::Left ? "turn away to the right, lean away from the left"
                                         : "turn away to the left, lean away from the right";
        } else {
            o.leanDirection = Normalize(c.threat.fireDirection);
            o.lean = std::max(o.lean, 0.2f * w);
            clause += "lean away from the threat";
        }
    }
};

// ---------------------------------------------------------------------------------------------------------
class ProtectHead final : public Behavior {
public:
    const char* Name() const override { return "ProtectHead"; }
    Channel GetChannel() const override { return Channel::UpperBody; }
    float Score(const BehaviorContext& c) const override {
        float s = 0.0f;
        if (c.threat.nearMissActive && c.threat.nearMissHead)
            s = 0.2f + 0.7f * c.threat.nearMissProximity * c.threat.nearMissRecency;
        float t = c.injury.TimeSinceLastHit();
        if (c.injury.LastHitPart() == BodyPart::Head && t < 3.0f) s = std::max(s, 0.8f * (1.0f - t / 3.0f));
        return Saturate(s * Saturate(c.injury.ControlLevel() * 2.0f));
    }
    void Apply(const BehaviorContext&, float w, ReactionTargets& o, std::string& clause) const override {
        RaiseArms(o, 1.5f, 0.45f, 2.1f, w);
        o.jointOffset[BodyPart::Head] += Vec3{0.35f * w, 0, 0};
        o.jointOffset[BodyPart::Neck] += Vec3{0.15f * w, 0, 0};
        clause += "raise arms to protect the head, tuck chin";
    }
};

// ---------------------------------------------------------------------------------------------------------
class ProtectWound final : public Behavior {
public:
    const char* Name() const override { return "ProtectWound"; }
    Channel GetChannel() const override { return Channel::UpperBody; }
    float Score(const BehaviorContext& c) const override {
        float pain = 0.0f;
        c.injury.WorstTorsoWound(&pain);
        float s = Saturate(pain * 1.3f);
        Side weak = c.injury.WeakerArm();
        float armCap = c.injury.ArmCapacity(weak);
        if (armCap < 0.8f) s = std::max(s, Saturate((1.0f - armCap) * 1.0f));
        return Saturate(s * Saturate(c.injury.ControlLevel() * 2.0f));
    }
    void Apply(const BehaviorContext& c, float w, ReactionTargets& o, std::string& clause) const override {
        float pain = 0.0f;
        c.injury.WorstTorsoWound(&pain);
        Side weak = c.injury.WeakerArm();
        float armCap = c.injury.ArmCapacity(weak);
        float armNeed = armCap < 0.8f ? (1.0f - armCap) : 0.0f;
        if (Saturate(pain * 1.3f) >= armNeed) {
            // Torso wound: hunch over it and cover it with both arms.
            o.jointOffset[BodyPart::Spine] += Vec3{0.17f * w, 0, 0};
            o.jointOffset[BodyPart::Chest] += Vec3{0.15f * w, 0, 0};
            RaiseArms(o, 0.7f, -0.15f, 1.7f, w);
            o.crouch = std::max(o.crouch, 0.2f * w);
            o.speedLimit = std::min(o.speedLimit, 1.0f - 0.4f * w);
            clause += "hunch over the wound, protect it with the arms, slow down";
        } else {
            // Arm wound: the injured arm is pulled across the body and cradled, the shoulder is turned away
            // from the shooter, the other arm stays out for balance.
            o.jointOffset[UpperArm(weak)] += Vec3{-0.55f * w, 0, SideSign(weak) * -0.45f * w};
            o.jointOffset[LowerArm(weak)] += Vec3{-1.9f * w, 0, 0};
            Side other = Opposite(weak);
            o.jointOffset[UpperArm(other)] += Vec3{-0.15f * w, 0, SideSign(other) * 0.7f * w};
            o.jointOffset[LowerArm(other)] += Vec3{-0.3f * w, 0, 0};
            o.authorityScale[UpperArm(weak)] *= 0.7f;
            o.authorityScale[LowerArm(weak)] *= 0.7f;
            o.jointOffset[BodyPart::Spine] += Vec3{0.10f * w, 0, 0};
            if (c.threat.active) o.turnYaw += SideSign(weak) * 0.45f * w;
            clause += weak == Side::Left ? "cradle the left arm, use the right arm for balance"
                                         : "cradle the right arm, use the left arm for balance";
        }
    }
};

// ---------------------------------------------------------------------------------------------------------
class BreakFall final : public Behavior {
public:
    const char* Name() const override { return "BreakFall"; }
    Channel GetChannel() const override { return Channel::Fall; }
    float Score(const BehaviorContext& c) const override {
        if (c.injury.ControlLevel() < 0.3f || c.injury.IsIncapacitated()) return 0.0f;
        if (c.body.supportMargin < 0.1f) return 0.0f; // still recoverable: keep stepping instead
        return Saturate((c.body.tilt - 0.7f) / 0.5f) * 0.9f;
    }
    void Apply(const BehaviorContext& c, float w, ReactionTargets& o, std::string& clause) const override {
        o.breakFall = true;
        o.assistScale *= 1.0f - 0.5f * w;
        Vec3 local = c.body.heading.Conjugate().Rotate(c.body.tiltDirection);
        float fwd = std::max(0.0f, local.z), side = std::abs(local.x);
        for (Side s : {Side::Left, Side::Right}) {
            // Reach towards the fall direction.
            float out = side * 1.1f * (local.x * SideSign(s) > 0 ? 1.0f : 0.4f);
            o.jointOffset[UpperArm(s)] += Vec3{-1.3f * fwd * w, 0, SideSign(s) * out * w};
            o.jointOffset[LowerArm(s)] += Vec3{-0.4f * fwd * w, 0, 0};
        }
        o.jointOffset[BodyPart::Head] += Vec3{0.4f * w, 0, 0};
        o.crouch = std::max(o.crouch, 0.5f * w);
        clause += "falling: reach out to break the fall, tuck the head";
    }
};

// ---------------------------------------------------------------------------------------------------------
class Collapse final : public Behavior {
public:
    const char* Name() const override { return "Collapse"; }
    Channel GetChannel() const override { return Channel::Fall; }
    float Score(const BehaviorContext& c) const override {
        if (c.injury.IsDead()) return 1.0f;
        float control = c.injury.ControlLevel();
        if (control >= 0.4f) return 0.0f;
        return 0.6f + 0.4f * (1.0f - control / 0.4f);
    }
    void Apply(const BehaviorContext& c, float w, ReactionTargets& o, std::string& clause) const override {
        float control = c.injury.ControlLevel();
        float progress = c.injury.IsDead() ? 1.0f : Saturate(1.0f - control / 0.4f);
        o.collapse = std::max(o.collapse, progress);
        o.crouch = std::max(o.crouch, progress);
        o.assistScale *= 1.0f - progress;
        float protect = Saturate((control - 0.1f) / 0.3f);
        if (protect > 0.0f) {
            RaiseArms(o, 1.2f, 0.4f, 1.9f, protect * w);
            o.jointOffset[BodyPart::Head] += Vec3{0.3f * protect * w, 0, 0};
            clause += "losing control: knees buckle, protect the head";
        } else {
            clause += "unconscious: go limp";
        }
    }
};

// ---------------------------------------------------------------------------------------------------------
/// Shared ground behaviour: weak muscles, curled-up protective pose, crawl or struggle. `sink` (0..1) is how far
/// the descent to the ground has progressed.
void GroundPose(const BehaviorContext& c, float sink, ReactionTargets& o, std::string& clause, const char* cause) {
    const float ground = c.body.lying ? 1.0f : Saturate((sink - 0.55f) / 0.45f);
    o.assistScale *= (1.0f - sink) * (1.0f - sink);
    o.crouch = std::max(o.crouch, sink);
    o.groundPose = std::max(o.groundPose, ground);
    o.collapse = std::max(o.collapse, 0.5f * sink);
    // Curl up around the vital area, arms covering head and chest.
    o.jointOffset[BodyPart::Spine] += Vec3{0.30f * sink, 0, 0};
    o.jointOffset[BodyPart::Chest] += Vec3{0.30f * sink, 0, 0};
    o.jointOffset[BodyPart::Head] += Vec3{0.30f * sink, 0, 0};
    RaiseArms(o, 1.1f, -0.1f, 2.0f, sink);
    // Weakened muscles: enough to hold a protective pose, never enough to stand again.
    for (int i = 1; i < kBodyPartCount; ++i) {
        BodyPart p = static_cast<BodyPart>(i);
        float f = IsLegPart(p) ? 0.5f : IsArmPart(p) ? 0.55f : 0.6f;
        if (c.body.lying && c.body.supine && (p == BodyPart::Spine || p == BodyPart::Chest)) f = 0.22f;
        o.authorityScale[p] *= Lerp(1.0f, f, sink);
    }
    clause += cause;
    if (c.body.lying && c.body.supine) {
        o.struggle = true;
        clause += " on the back: struggles to get up but cannot, keeps falling back down";
    } else if (c.body.lying) {
        o.crawl = true;
        clause += ": curled up on the ground protecting the head and body, crawling";
    } else {
        clause += ": legs give out, slowly sinking to the ground, curling up to protect the body";
    }
}

class Incapacitated final : public Behavior {
public:
    const char* Name() const override { return "Incapacitated"; }
    Channel GetChannel() const override { return Channel::Fall; }
    float Score(const BehaviorContext& c) const override { return c.injury.IsIncapacitated() ? 0.7f : 0.0f; }
    void Apply(const BehaviorContext& c, float, ReactionTargets& o, std::string& clause) const override {
        // Sinking: the assist fades over ~2 s so the body slowly goes down instead of dropping.
        GroundPose(c, Saturate(c.injury.IncapacitatedTime() / 2.0f), o, clause, "incapacitated");
    }
};

/// Knocked off its feet (thrown, shoved, fell over) and now lying down: it stays down protecting itself.
class KnockedDown final : public Behavior {
public:
    const char* Name() const override { return "KnockedDown"; }
    Channel GetChannel() const override { return Channel::Fall; }
    float Score(const BehaviorContext& c) const override {
        if (c.injury.IsDead() || c.injury.IsIncapacitated()) return 0.0f;
        return (c.body.lying && c.body.lyingTime > 0.4f) || c.body.downHold > 0.0f ? 0.65f : 0.0f;
    }
    void Apply(const BehaviorContext& c, float, ReactionTargets& o, std::string& clause) const override {
        // Fully down while lying; once up, the descent state fades out over the last second (a gradual get-up).
        float sink = c.body.lying ? 1.0f : Saturate(c.body.downHold);
        GroundPose(c, sink, o, clause, "knocked down");
    }
};

} // namespace

std::string DescribeSituation(const BehaviorContext& c) {
    char buf[160];
    const InjuryModel& inj = c.injury;
    if (inj.IsDead() || inj.ControlLevel() < 0.4f) {
        std::snprintf(buf, sizeof buf, "Critical damage (control %.2f)", inj.ControlLevel());
        return buf;
    }
    if (inj.IsIncapacitated()) return "Incapacitated by a torso wound";
    Side weak = inj.WeakerLeg();
    float cap = inj.SupportCapacity(weak);
    if (cap < kNoLimpAbove) {
        std::snprintf(buf, sizeof buf, "%s leg support %.2f", weak == Side::Left ? "Left" : "Right", cap);
        return buf;
    }
    if (c.threat.nearMissActive && c.threat.nearMissRecency > 0.3f) {
        std::snprintf(buf, sizeof buf, "Bullet passed close to the %s side of the %s (proximity %.2f)",
                      c.threat.nearMissSide == Side::Left ? "left" : "right",
                      c.threat.nearMissHead ? "head" : "body", c.threat.nearMissProximity);
        return buf;
    }
    Side wa = inj.WeakerArm();
    float acap = inj.ArmCapacity(wa);
    if (acap < 0.8f) {
        std::snprintf(buf, sizeof buf, "%s arm capacity %.2f", wa == Side::Left ? "Left" : "Right", acap);
        return buf;
    }
    float pain = 0.0f;
    BodyPart wound = inj.WorstTorsoWound(&pain);
    if (pain > 0.15f) {
        std::snprintf(buf, sizeof buf, "%s wound (pain %.2f)", BodyPartName(wound), pain);
        return buf;
    }
    if (c.body.supportMargin > 0.02f) return "Capture point outside the support polygon";
    if (c.threat.active) return "Under threat";
    return "No threat";
}

DecisionLayer::DecisionLayer() {
    AddBehavior(std::make_unique<Brace>());
    AddBehavior(std::make_unique<StumbleRecover>());
    AddBehavior(std::make_unique<LimpAway>());
    AddBehavior(std::make_unique<SeekCover>());
    AddBehavior(std::make_unique<TurnAndLeanAway>());
    AddBehavior(std::make_unique<ProtectHead>());
    AddBehavior(std::make_unique<ProtectWound>());
    AddBehavior(std::make_unique<BreakFall>());
    AddBehavior(std::make_unique<Collapse>());
    AddBehavior(std::make_unique<Incapacitated>());
    AddBehavior(std::make_unique<KnockedDown>());
}

DecisionLayer::~DecisionLayer() = default;

void DecisionLayer::AddBehavior(std::unique_ptr<Behavior> b) {
    Entry e;
    e.behavior = std::move(b);
    behaviors_.push_back(std::move(e));
}

bool DecisionLayer::RemoveBehavior(const std::string& name) {
    for (auto it = behaviors_.begin(); it != behaviors_.end(); ++it) {
        if (name == it->behavior->Name()) {
            behaviors_.erase(it);
            return true;
        }
    }
    return false;
}

bool DecisionLayer::SetEnabled(const std::string& name, bool enabled) {
    for (auto& e : behaviors_) {
        if (name == e.behavior->Name()) {
            e.enabled = enabled;
            return true;
        }
    }
    return false;
}

void DecisionLayer::Reset() {
    decision_ = Decision{};
    lastWinners_.clear();
    reasonTimer_ = 1e9f;
    lastKey_.clear();
}

const Decision& DecisionLayer::Decide(const BehaviorContext& ctx, float dt) {
    decision_.targets = ReactionTargets{};
    decision_.active.clear();

    constexpr int kChannels = static_cast<int>(Channel::Count);
    const Behavior* winner[kChannels] = {};
    float winnerScore[kChannels] = {};
    for (auto& e : behaviors_) {
        if (!e.enabled) continue;
        float s = Saturate(e.behavior->Score(ctx));
        float biased = s;
        if (std::find(lastWinners_.begin(), lastWinners_.end(), e.behavior.get()) != lastWinners_.end()) biased += hysteresis;
        int ch = static_cast<int>(e.behavior->GetChannel());
        if (s >= minScore && biased > winnerScore[ch]) {
            winner[ch] = e.behavior.get();
            winnerScore[ch] = biased;
        }
    }
    // A dominant fall behaviour (collapse) overrides the controlled channels.
    const int fall = static_cast<int>(Channel::Fall);
    bool fallDominates = winner[fall] && winnerScore[fall] >= 0.6f;

    std::string clauses;
    std::string key;
    lastWinners_.clear();
    // Apply in a fixed channel order so the reason string reads: fall, locomotion, recovery, posture, upper body.
    static const Channel order[] = {Channel::Fall, Channel::Locomotion, Channel::Recovery, Channel::Posture,
                                    Channel::UpperBody};
    for (Channel chn : order) {
        int ch = static_cast<int>(chn);
        if (!winner[ch]) continue;
        if (fallDominates && chn != Channel::Fall && chn != Channel::UpperBody) continue;
        float w = Saturate(winnerScore[ch]);
        std::string clause;
        winner[ch]->Apply(ctx, w, decision_.targets, clause);
        if (!clauses.empty()) clauses += ", ";
        clauses += clause;
        decision_.active.push_back({winner[ch]->Name(), chn, w});
        lastWinners_.push_back(winner[ch]);
        key += winner[ch]->Name();
        key += ';';
    }

    reasonTimer_ += dt;
    if (key != lastKey_ || reasonTimer_ >= reasonRefreshSeconds) {
        reasonTimer_ = 0.0f;
        lastKey_ = key;
        std::string situation = DescribeSituation(ctx);
        decision_.reason = clauses.empty() ? situation + " -> idle: stand and observe" : situation + " -> " + clauses;
    }
    return decision_;
}

} // namespace reflex
