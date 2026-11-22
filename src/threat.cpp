#include "reflex/threat.h"

#include <cmath>

namespace reflex {

void ThreatModel::Reset() {
    memory_ = Memory{};
    miss_ = NearMiss{};
}

void ThreatModel::RegisterHit(const HitEvent& hit) {
    Vec3 d = Horizontal(hit.direction);
    if (LengthSq(d) < 1e-6f) return; // vertical shot: no usable direction information
    memory_.valid = true;
    memory_.fireDirection = Normalize(d);
    memory_.age = 0.0f;
    memory_.confidence = 1.0f;
}

void ThreatModel::RegisterNearMiss(const NearMissEvent& m) {
    miss_.valid = true;
    miss_.closestPoint = m.closestPoint;
    miss_.direction = Normalize(m.direction, {0, 0, 1});
    miss_.distance = m.distance;
    miss_.part = m.nearestPart;
    miss_.age = 0.0f;
    Vec3 d = Horizontal(m.direction);
    if (LengthSq(d) > 1e-6f) {
        // A near miss is weaker evidence than a hit, so never overwrite fresher, stronger knowledge
        // unless the new information is more recent.
        if (!memory_.valid || memory_.confidence < 0.99f || memory_.age > 1.0f) {
            memory_.valid = true;
            memory_.fireDirection = Normalize(d);
            memory_.age = 0.0f;
            memory_.confidence = 0.6f;
        }
    }
}

void ThreatModel::Update(float dt) {
    memory_.age += dt;
    miss_.age += dt;
}

ThreatPicture ThreatModel::Evaluate(const Transform& frame, const Vec3& headPosition) const {
    ThreatPicture t;
    t.secondsSinceThreat = std::min(memory_.age, miss_.age);

    // Heading: project the frame's forward axis on the ground.
    Vec3 fwd = Horizontal(frame.rotation.Rotate({0, 0, 1}));
    fwd = Normalize(fwd, {0, 0, 1});
    Quat heading = FromTo({0, 0, 1}, fwd);
    Quat inv = heading.Conjugate();

    if (memory_.valid) {
        float recency = std::exp(-memory_.age / RecencyDecaySeconds);
        if (recency > 0.04f) {
            t.active = true;
            t.recency = recency;
            t.confidence = memory_.confidence;
            t.fireDirection = memory_.fireDirection;
            t.towardSource = -memory_.fireDirection;
            Vec3 local = inv.Rotate(t.towardSource);
            t.exposedSide = local.x;
            t.frontExposure = local.z;

            // Cover: callback first, then registered points that shield from this direction.
            Vec3 coverPos;
            if (query_ && query_(frame.position, t.towardSource, coverPos)) {
                t.hasCover = true;
                t.coverPoint = coverPos;
            } else {
                float best = -1e9f;
                for (const CoverPoint& c : cover_) {
                    Vec3 to = Horizontal(c.position - frame.position);
                    float dist = Length(to);
                    if (dist > MaxCoverDistance) continue;
                    float shield = Dot(Horizontal(c.normal), t.towardSource); // 1 = shields perfectly
                    if (shield < 0.3f) continue;
                    float score = shield * 4.0f - dist;
                    if (score > best) {
                        best = score;
                        t.hasCover = true;
                        t.coverPoint = c.position;
                    }
                }
            }
            if (t.hasCover) t.coverDistance = Length(Horizontal(t.coverPoint - frame.position));
        }
    }

    if (miss_.valid) {
        float rec = std::exp(-miss_.age / 3.0f);
        if (rec > 0.05f) {
            t.nearMissActive = true;
            t.nearMissRecency = rec;
            t.nearMissPart = miss_.part;
            t.nearMissProximity = Saturate(1.0f - miss_.distance / 0.8f);
            Vec3 local = inv.Rotate(miss_.closestPoint - headPosition);
            t.nearMissSide = local.x >= 0.0f ? Side::Left : Side::Right;
            t.nearMissHead = IsHeadPart(miss_.part);
            t.active = true;
            t.recency = std::max(t.recency, rec);
        }
    }
    return t;
}

} // namespace reflex
