#include "reflex/authority.h"

namespace reflex {

AuthorityBlender::AuthorityBlender() { Reset(); }

void AuthorityBlender::Reset() {
    global_ = BlendedValue(1.0f);
    for (auto& p : part_) p = BlendedValue(1.0f);
    anim_ = BlendedValue(0.0f);
}

void AuthorityBlender::SetGlobal(float value, float blendTime) { global_.SetTarget(Saturate(value), blendTime); }

void AuthorityBlender::SetPart(BodyPart part, float value, float blendTime) {
    part_[static_cast<size_t>(part)].SetTarget(Saturate(value), blendTime);
}

void AuthorityBlender::SetAnimationWeight(float value, float blendTime) {
    anim_.SetTarget(Saturate(value), blendTime);
}

void AuthorityBlender::Update(float dt) {
    global_.Update(dt);
    for (auto& p : part_) p.Update(dt);
    anim_.Update(dt);
}

} // namespace reflex
