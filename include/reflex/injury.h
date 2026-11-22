#pragma once
/// @file injury.h
/// Injury and body-state model: per-part health, pain and functional capacity, plus derived whole-body
/// state (shock, consciousness, overall control level).
///
/// Units: damage is measured in health points. A limb has `InjuryConfig::maxHealth` of 100 by default,
/// so a damage value of 35 is a serious but survivable limb wound.

#include "reflex/math.h"
#include "reflex/types.h"

namespace reflex {

struct InjuryConfig {
    InjuryConfig();
    PerPart<float> maxHealth;      ///< health points per body part
    PerPart<float> vitalWeight;    ///< how much a damage point to this part drains whole-body vitality
    float vitalityPerDamage = 1.0f / 150.0f; ///< vitality lost per weighted damage point
    float painPerDamage = 0.02f;   ///< pain added per damage point (pain is 0..1)
    float painDecay = 0.12f;       ///< pain decays towards its floor by this much per second
    float painFloorFraction = 0.25f; ///< persistent pain = this * (1 - health fraction)
    float shockPerDamage = 0.012f; ///< whole-body shock per damage point (scaled by vital weight)
    float shockDecay = 0.10f;      ///< shock recovery per second
    float bleedPerDamage = 0.010f; ///< HP/s of bleeding added to a part per damage point
    float bleedMinDamage = 15.0f;  ///< wounds below this do not bleed
    float bleedClotRate = 0.03f;   ///< bleeding rate decays by this fraction per second
    float consciousnessFallRate = 0.5f; ///< per second
    float consciousnessRiseRate = 0.15f;
    float legFatiguePerSecond = 0.02f;  ///< fatigue growth per second at full load on an injured leg
};

struct PartState {
    float health = 1.0f;   ///< 0..1 fraction of max health remaining
    float pain = 0.0f;     ///< 0..1
    float capacity = 1.0f; ///< 0..1 functional capacity (torque / load / mobility left)
    float bleed = 0.0f;    ///< HP/s currently being lost
};

class REFLEX_API InjuryModel {
public:
    explicit InjuryModel(const InjuryConfig& cfg = InjuryConfig());

    void Reset();
    /// Apply damage (health points) to a body part.
    void ApplyDamage(BodyPart part, float damage);
    /// Advance the model: pain/shock decay, bleeding, consciousness dynamics.
    void Update(float dt);
    /// Report how heavily a leg was loaded this tick (0..1) so injured legs tire out.
    void ReportLegLoad(Side side, float load, float dt);

    const PartState& Part(BodyPart p) const { return parts_[p]; }
    float Capacity(BodyPart p) const { return parts_[p].capacity; }
    /// How much body weight a leg can still bear (0..1). Drops sharply when the leg is shot.
    float SupportCapacity(Side side) const;
    /// How usable an arm still is (0..1).
    float ArmCapacity(Side side) const;
    float HeadCapacity() const;
    float Shock() const { return shock_; }
    float Fatigue() const { return 0.5f * (legFatigue_[0] + legFatigue_[1]); }
    float Vitality() const { return vitality_; }
    float Consciousness() const { return consciousness_; }
    /// Overall control level 0..1 (the global authority ceiling imposed by injuries).
    float ControlLevel() const;
    bool IsDead() const { return vitality_ <= 0.0f; }
    /// True when the character is about to lose consciousness or is dying.
    bool IsCritical() const { return vitality_ < 0.3f || consciousness_ < 0.45f; }

    /// Part that received the most recent damage (valid when TimeSinceLastHit() is finite).
    BodyPart LastHitPart() const { return lastHitPart_; }
    float TimeSinceLastHit() const { return timeSinceHit_; }
    float LastHitDamage() const { return lastDamage_; }
    /// Torso part with the highest pain (the wound the character wants to protect), or Chest.
    BodyPart WorstTorsoWound(float* outPain = nullptr) const;
    /// Arm/leg side with the lowest capacity.
    Side WeakerLeg() const { return SupportCapacity(Side::Left) < SupportCapacity(Side::Right) ? Side::Left : Side::Right; }
    Side WeakerArm() const { return ArmCapacity(Side::Left) < ArmCapacity(Side::Right) ? Side::Left : Side::Right; }

    const InjuryConfig& Config() const { return cfg_; }

private:
    void RecomputeCapacity(BodyPart p);

    InjuryConfig cfg_;
    PerPart<PartState> parts_;
    float shock_ = 0, vitality_ = 1, consciousness_ = 1;
    float legFatigue_[2] = {0, 0};
    BodyPart lastHitPart_ = BodyPart::Chest;
    float timeSinceHit_ = 1e9f;
    float lastDamage_ = 0;
};

} // namespace reflex
