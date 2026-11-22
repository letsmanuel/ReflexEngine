#pragma once
/// @file threat.h
/// Perception and threat model. Turns hit and near-miss events into a "threat picture": the direction
/// of fire, how recent it was, which side of the body is exposed and where cover is.

#include <functional>
#include <vector>

#include "reflex/events.h"
#include "reflex/export.h"
#include "reflex/math.h"
#include "reflex/types.h"

namespace reflex {

/// A place that protects against fire coming from `normal` (a horizontal unit vector pointing from the
/// cover object towards the side it shields from, i.e. towards the shooter).
struct CoverPoint {
    Vec3 position;  ///< where to stand (behind the cover object)
    Vec3 normal{0, 0, 1};
};

/// Optional user-supplied cover query: given the character position and the horizontal direction towards
/// the shooter, return true and a cover position (ground point) if cover is available.
using CoverQuery = std::function<bool(const Vec3& characterPosition, const Vec3& towardShooter, Vec3& outCover)>;

struct ThreatPicture {
    bool active = false;
    Vec3 fireDirection{0, 0, 1};  ///< horizontal unit direction the projectiles travel
    Vec3 towardSource{0, 0, -1};  ///< horizontal unit direction from the character towards the shooter
    float recency = 0.0f;         ///< 1 = just happened, decays to 0 over several seconds
    float confidence = 0.0f;      ///< 1 for hits, lower for near misses only
    float exposedSide = 0.0f;     ///< local x of towardSource: + the shooter is on the left, - on the right
    float frontExposure = 0.0f;   ///< local z of towardSource: + the shooter is in front, - behind

    bool hasCover = false;
    Vec3 coverPoint;
    float coverDistance = 0.0f;

    bool nearMissActive = false;
    Side nearMissSide = Side::Left;  ///< side of the body the bullet passed
    float nearMissProximity = 0.0f;  ///< 1 = grazed, 0 = far away
    float nearMissRecency = 0.0f;
    BodyPart nearMissPart = BodyPart::Head;
    bool nearMissHead = false;       ///< passed close to the head / neck
    float secondsSinceThreat = 1e9f;
};

class REFLEX_API ThreatModel {
public:
    ThreatModel() = default;

    void Reset();
    void RegisterHit(const HitEvent& hit);
    void RegisterNearMiss(const NearMissEvent& miss);
    /// Same, but latches which side of the body the bullet passed at this moment (relative to the character's
    /// frame and head position), so the answer does not change as the character moves afterwards.
    void RegisterNearMiss(const NearMissEvent& miss, const Transform& frame, const Vec3& headPosition);
    void Update(float dt);

    void AddCoverPoint(const CoverPoint& c) { cover_.push_back(c); }
    void ClearCoverPoints() { cover_.clear(); }
    void SetCoverQuery(CoverQuery q) { query_ = std::move(q); }

    /// Build the picture relative to a character frame: `frame.position` is the character's position,
    /// `frame.rotation` its heading (yaw only matters). Local +Z is forward, +X is left.
    ThreatPicture Evaluate(const Transform& frame, const Vec3& headPosition) const;

    float RecencyDecaySeconds = 6.0f;
    float MaxCoverDistance = 12.0f;

private:
    struct Memory {
        bool valid = false;
        Vec3 fireDirection{0, 0, 1};
        float age = 1e9f;
        float confidence = 0.0f;
    };
    struct NearMiss {
        bool valid = false;
        Vec3 closestPoint;
        Vec3 direction{0, 0, 1};
        float distance = 1.0f;
        BodyPart part = BodyPart::Head;
        float age = 1e9f;
        bool latched = false;
        Side side = Side::Left;
    };
    Memory memory_;
    NearMiss miss_;
    std::vector<CoverPoint> cover_;
    CoverQuery query_;
};

} // namespace reflex
