#pragma once
/// @file authority.h
/// Control-authority system. Authority (0..1) says how strongly the engine drives a joint
/// versus letting physics run passively. Three sources are blended:
///   (a) the game's authored animation,
///   (b) the engine's procedural reaction,
///   (c) a pure passive ragdoll (authority 0).
///
/// effective authority(part) = global * perPart * injuryControl * injuryCapacity(part)

#include "reflex/export.h"
#include "reflex/math.h"
#include "reflex/types.h"

namespace reflex {

/// A float that eases towards a target over a blend time (smooth-step).
class BlendedValue {
public:
    BlendedValue() = default;
    explicit BlendedValue(float v) : cur_(v), from_(v), to_(v) {}
    void SetTarget(float target, float blendTime) {
        to_ = target;
        from_ = cur_;
        elapsed_ = 0.0f;
        duration_ = blendTime;
        if (blendTime <= 0.0f) cur_ = target;
    }
    void Update(float dt) {
        if (cur_ == to_) return;
        elapsed_ += dt;
        float t = duration_ > 0.0f ? elapsed_ / duration_ : 1.0f;
        cur_ = t >= 1.0f ? to_ : Lerp(from_, to_, SmoothStep(t));
    }
    float Value() const { return cur_; }
    float Target() const { return to_; }
    bool Settled() const { return cur_ == to_; }

private:
    float cur_ = 1.0f, from_ = 1.0f, to_ = 1.0f, elapsed_ = 0.0f, duration_ = 0.0f;
};

class REFLEX_API AuthorityBlender {
public:
    AuthorityBlender();
    void Reset();

    /// Set the authority of every joint (0 = fully passive ragdoll, 1 = fully driven).
    void SetGlobal(float value, float blendTime = 0.0f);
    /// Set the authority of one body part's joint (the joint that connects it to its parent).
    void SetPart(BodyPart part, float value, float blendTime = 0.0f);
    /// Blend between the engine's procedural targets (0) and the game's animation pose (1).
    void SetAnimationWeight(float value, float blendTime = 0.0f);
    /// Smoothly hand control back to the game's animation. Equivalent to SetAnimationWeight(1, t).
    void ReleaseToAnimation(float blendTime) { SetAnimationWeight(1.0f, blendTime); }
    /// Take control of the pose procedurally again.
    void TakeProceduralControl(float blendTime) { SetAnimationWeight(0.0f, blendTime); }

    void Update(float dt);

    float Global() const { return global_.Value(); }
    float Part(BodyPart p) const { return part_[static_cast<size_t>(p)].Value(); }
    float AnimationWeight() const { return anim_.Value(); }

    /// Authority actually applied to a joint after combining the game's settings with the injury model.
    float Effective(BodyPart p, float injuryControl, float injuryCapacity) const {
        return Saturate(Global() * Part(p) * Saturate(injuryControl) * Saturate(injuryCapacity));
    }

private:
    BlendedValue global_;
    BlendedValue part_[kBodyPartCount];
    BlendedValue anim_{0.0f};
};

} // namespace reflex
