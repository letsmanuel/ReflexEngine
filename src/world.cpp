#include "reflex/world.h"

#include <algorithm>

#include "character_impl.h"

namespace reflex {

World::World(IPhysicsBackend& backend) : backend_(&backend) {}

World::World(std::unique_ptr<IPhysicsBackend> backend) : owned_(std::move(backend)), backend_(owned_.get()) {}

World::~World() {
    for (auto& c : characters_) c->Destroy();
    characters_.clear();
}

Character World::CreateCharacter(const CharacterDesc& desc) {
    auto impl = std::make_shared<Character::Impl>();
    impl->Build(backend_, nextId_++, desc);
    characters_.push_back(impl);
    return Character(impl);
}

void World::DestroyCharacter(Character& c) {
    auto it = std::find(characters_.begin(), characters_.end(), c.impl_);
    if (it != characters_.end()) {
        (*it)->Destroy();
        characters_.erase(it);
    }
    c.impl_.reset();
}

int World::CharacterCount() const { return static_cast<int>(characters_.size()); }

Character World::GetCharacter(int index) const {
    if (index < 0 || index >= CharacterCount()) return Character();
    return Character(characters_[static_cast<size_t>(index)]);
}

void World::Step(float dt) {
    accumulator_ = std::min(accumulator_ + dt, 0.25f);
    while (accumulator_ >= fixedStep_) {
        for (auto& c : characters_) c->Tick(fixedStep_);
        backend_->Step(fixedStep_);
        accumulator_ -= fixedStep_;
        time_ += fixedStep_;
    }
}

BulletResult World::TraceBullet(const Vec3& origin, const Vec3& direction, float maxDistance,
                                const BulletParams& params) {
    BulletResult result;
    const Vec3 dir = Normalize(direction, {0, 0, 1});
    RayHit hit;
    const bool anyHit = backend_->RayCast(origin, dir, maxDistance, false, hit);
    const float reach = anyHit ? hit.distance : maxDistance;

    std::shared_ptr<Character::Impl> struck;
    if (anyHit) {
        result.point = hit.point;
        result.distance = hit.distance;
        result.type = BulletResult::Type::HitScenery;
        std::uint32_t cid = static_cast<std::uint32_t>(hit.userData >> 8);
        int partIdx = static_cast<int>(hit.userData & 0xFF) - 1;
        if (cid != 0 && partIdx >= 0 && partIdx < kBodyPartCount) {
            for (auto& c : characters_) {
                if (c->id == cid) struck = c;
            }
        }
        if (struck) {
            BodyPart part = static_cast<BodyPart>(partIdx);
            float mult = IsHeadPart(part) ? params.headMultiplier
                         : IsLimb(part)   ? params.limbMultiplier
                                          : params.torsoMultiplier;
            HitEvent ev;
            ev.bodyPart = part;
            ev.point = hit.point;
            ev.direction = dir;
            ev.speed = params.speed;
            ev.mass = params.mass;
            ev.impulse = params.mass * params.speed;
            ev.damage = params.damage * mult;
            struck->ApplyHit(ev);
            result.type = BulletResult::Type::HitCharacter;
            result.part = part;
            result.character = Character(struck);
        }
    }

    // Near misses for everyone else the bullet passed.
    float bestMiss = 1e9f;
    for (auto& c : characters_) {
        if (c == struck) continue;
        BodyPart part;
        Vec3 pt;
        float d = c->ClosestApproach(origin, dir, reach, part, pt);
        if (d <= params.nearMissRadius) {
            NearMissEvent ev;
            ev.closestPoint = pt;
            ev.direction = dir;
            ev.distance = d;
            ev.nearestPart = part;
            c->ReportNearMiss(ev);
            if (result.type != BulletResult::Type::HitCharacter && d < bestMiss) {
                bestMiss = d;
                result.type = BulletResult::Type::NearMiss;
                result.point = pt;
                result.part = part;
                result.distance = d;
                result.character = Character(c);
            }
        }
    }
    return result;
}

} // namespace reflex
