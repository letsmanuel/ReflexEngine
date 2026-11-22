#include "reflex/injury.h"

#include <algorithm>
#include <cmath>

namespace reflex {

InjuryConfig::InjuryConfig() {
    maxHealth.Fill(100.0f);
    vitalWeight.Fill(0.4f);
    maxHealth[BodyPart::Pelvis] = 140.0f;
    maxHealth[BodyPart::Spine] = 140.0f;
    maxHealth[BodyPart::Chest] = 160.0f;
    maxHealth[BodyPart::Neck] = 70.0f;
    maxHealth[BodyPart::Head] = 80.0f;
    maxHealth[BodyPart::LeftHand] = maxHealth[BodyPart::RightHand] = 60.0f;
    maxHealth[BodyPart::LeftFoot] = maxHealth[BodyPart::RightFoot] = 60.0f;
    vitalWeight[BodyPart::Pelvis] = 1.0f;
    vitalWeight[BodyPart::Spine] = 1.3f;
    vitalWeight[BodyPart::Chest] = 1.6f;
    vitalWeight[BodyPart::Neck] = 2.0f;
    vitalWeight[BodyPart::Head] = 2.2f;
    vitalWeight[BodyPart::LeftUpperLeg] = vitalWeight[BodyPart::RightUpperLeg] = 0.6f; // femoral artery
}

InjuryModel::InjuryModel(const InjuryConfig& cfg) : cfg_(cfg) { Reset(); }

void InjuryModel::Reset() {
    for (auto& p : parts_.v) p = PartState{};
    shock_ = 0;
    vitality_ = 1;
    consciousness_ = 1;
    legFatigue_[0] = legFatigue_[1] = 0;
    lastHitPart_ = BodyPart::Chest;
    timeSinceHit_ = 1e9f;
    lastDamage_ = 0;
}

void InjuryModel::RecomputeCapacity(BodyPart p) {
    PartState& s = parts_[p];
    float base = std::pow(Saturate(s.health), 1.5f);
    s.capacity = Saturate(base * (1.0f - 0.7f * Saturate(s.pain)));
}

void InjuryModel::ApplyDamage(BodyPart part, float damage) {
    if (damage <= 0.0f) return;
    PartState& s = parts_[part];
    const float maxHp = cfg_.maxHealth[part];
    s.health = Saturate(s.health - damage / maxHp);
    s.pain = Saturate(s.pain + damage * cfg_.painPerDamage);
    if (damage >= cfg_.bleedMinDamage) s.bleed += damage * cfg_.bleedPerDamage;

    const float w = cfg_.vitalWeight[part];
    vitality_ -= damage * w * cfg_.vitalityPerDamage;
    shock_ = Saturate(shock_ + damage * cfg_.shockPerDamage * (0.5f + 0.5f * w));

    // Neighbouring parts feel part of the blow (pain only).
    BodyPart parent = ParentOf(part);
    if (parent != part) parts_[parent].pain = Saturate(parts_[parent].pain + damage * cfg_.painPerDamage * 0.15f);

    lastHitPart_ = part;
    lastDamage_ = damage;
    timeSinceHit_ = 0.0f;
    for (int i = 0; i < kBodyPartCount; ++i) RecomputeCapacity(static_cast<BodyPart>(i));
}

void InjuryModel::ReportLegLoad(Side side, float load, float dt) {
    int i = SideIndex(side);
    float injury = 1.0f - Saturate(parts_[UpperLeg(side)].health * parts_[LowerLeg(side)].health);
    legFatigue_[i] = Saturate(legFatigue_[i] + load * injury * cfg_.legFatiguePerSecond * dt - 0.004f * dt);
}

void InjuryModel::Update(float dt) {
    timeSinceHit_ += dt;
    for (int i = 0; i < kBodyPartCount; ++i) {
        BodyPart bp = static_cast<BodyPart>(i);
        PartState& s = parts_[bp];
        float floorPain = cfg_.painFloorFraction * (1.0f - s.health);
        if (s.pain > floorPain) s.pain = std::max(floorPain, s.pain - cfg_.painDecay * dt);
        if (s.bleed > 0.0f) {
            float loss = s.bleed * dt;
            s.health = Saturate(s.health - loss / cfg_.maxHealth[bp]);
            vitality_ -= loss * cfg_.vitalWeight[bp] * cfg_.vitalityPerDamage * 0.5f;
            s.bleed = std::max(0.0f, s.bleed * (1.0f - cfg_.bleedClotRate * dt) - 0.002f * dt);
        }
        RecomputeCapacity(bp);
    }
    shock_ = std::max(0.0f, shock_ - cfg_.shockDecay * dt);

    // Consciousness drifts towards a target set by vitality and shock.
    float target = vitality_ > 0.3f ? 1.0f : Saturate(vitality_ / 0.3f);
    if (vitality_ <= 0.0f) target = 0.0f;
    target *= 1.0f - 0.9f * Saturate((shock_ - 0.55f) * 2.2f);
    float rate = target < consciousness_ ? cfg_.consciousnessFallRate : cfg_.consciousnessRiseRate;
    if (vitality_ <= 0.0f) rate = cfg_.consciousnessFallRate * 1.5f;
    float delta = target - consciousness_;
    consciousness_ += Clamp(delta, -rate * dt, rate * dt);
    consciousness_ = Saturate(consciousness_);
}

float InjuryModel::SupportCapacity(Side side) const {
    const float up = parts_[UpperLeg(side)].capacity;
    const float lo = parts_[LowerLeg(side)].capacity;
    const float ft = parts_[Foot(side)].capacity;
    float c = std::min(up, lo) * (0.85f + 0.15f * ft);
    // A destroyed pelvis weakens both legs.
    c *= 0.5f + 0.5f * parts_[BodyPart::Pelvis].capacity;
    c *= 1.0f - 0.35f * legFatigue_[SideIndex(side)];
    return Saturate(c);
}

float InjuryModel::ArmCapacity(Side side) const {
    float c = std::min(parts_[UpperArm(side)].capacity, parts_[LowerArm(side)].capacity);
    c *= 0.8f + 0.2f * parts_[Hand(side)].capacity;
    return Saturate(c);
}

float InjuryModel::HeadCapacity() const {
    return std::min(parts_[BodyPart::Head].capacity, parts_[BodyPart::Neck].capacity);
}

float InjuryModel::ControlLevel() const { return Saturate(consciousness_ * (1.0f - 0.6f * shock_)); }

BodyPart InjuryModel::WorstTorsoWound(float* outPain) const {
    BodyPart best = BodyPart::Chest;
    float bestPain = 0.0f;
    for (BodyPart p : {BodyPart::Pelvis, BodyPart::Spine, BodyPart::Chest}) {
        if (parts_[p].pain > bestPain) {
            bestPain = parts_[p].pain;
            best = p;
        }
    }
    if (outPain) *outPain = bestPain;
    return best;
}

} // namespace reflex
