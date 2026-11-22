#pragma once
/// @file behavior.h
/// Behaviour / decision layer. A utility-based selector scores every behaviour each tick, picks the best
/// one per channel (locomotion, recovery, upper body, posture, fall) and blends their outputs into one set of
/// ReactionTargets. Behaviours never touch physics: they output targets (joint offsets, stance, step goals,
/// movement direction). The decision always carries a human-readable reason string.

#include <memory>
#include <string>
#include <vector>

#include "reflex/balance.h"
#include "reflex/export.h"
#include "reflex/injury.h"
#include "reflex/math.h"
#include "reflex/threat.h"
#include "reflex/types.h"

namespace reflex {

/// Summary of the physical state of the character, computed by the Character each tick.
struct BodyStateSummary {
    Vec3 com;                    ///< centre of mass (world)
    Vec3 comVelocity;
    Vec3 capturePoint;           ///< ground projection of the capture point
    float comHeight = 0.95f;     ///< COM height above the ground
    float standingComHeight = 0.95f;
    float supportMargin = -0.1f; ///< signed distance of capture point to support polygon (<0 inside)
    bool footContact[2] = {true, true};
    float tilt = 0.0f;           ///< torso angle from vertical (rad)
    Vec3 tiltDirection{0, 0, 1}; ///< horizontal direction the torso leans towards (world)
    float speed = 0.0f;          ///< horizontal COM speed
    Quat heading;                ///< yaw-only orientation of the character (local +Z forward, +X left)
    Vec3 position;               ///< pelvis position
    bool lying = false;          ///< body is down on the ground
    bool supine = false;         ///< lying on the back
    float lyingTime = 0.0f;      ///< seconds spent lying
    float downHold = 0.0f;       ///< seconds a knocked-down character is still kept down (counts down once up)
};

struct BehaviorContext {
    const InjuryModel& injury;
    const ThreatPicture& threat;
    const BodyStateSummary& body;
    float time = 0.0f;
};

/// Combined output of all active behaviours. Everything is a target, nothing is a force.
struct ReactionTargets {
    float crouch = 0.0f;           ///< 0..1 lowering of the centre of mass
    float stanceWidth = 0.0f;      ///< extra distance between the feet (m)
    Vec3 moveDirection;            ///< world horizontal unit vector (or zero)
    float moveSpeed = 0.0f;        ///< 0..1 fraction of walking speed
    float speedLimit = 1.0f;       ///< multiplier applied to moveSpeed (< 1 slows the character down)
    Vec3 leanDirection;            ///< world horizontal unit vector the upper body leans towards
    float lean = 0.0f;             ///< lean angle (rad)
    float turnYaw = 0.0f;          ///< desired heading change (rad, + = turn left)
    float legLoadBias = 0.0f;      ///< -1..1, positive = put more weight on the left leg
    bool wantStep = false;         ///< balance controller should take a recovery step
    bool limping = false;
    Side limpSide = Side::Right;   ///< the injured leg
    float limpAmount = 0.0f;       ///< 0..1
    PerPart<Vec3> jointOffset;     ///< rotation vector (rad) offset from the stance pose, parent frame
    PerPart<float> authorityScale; ///< multiplier on each joint's authority (default 1)
    float assistScale = 1.0f;      ///< multiplier on the balance assist forces
    float collapse = 0.0f;         ///< 0..1 how far the collapse has progressed
    bool breakFall = false;
    float groundPose = 0.0f;       ///< 0..1 blend from standing IK to the on-the-ground pose
    bool struggle = false;         ///< lying on the back, trying (and failing) to get up
    bool crawl = false;            ///< curled up / crawling protectively
    float struggleStrength = 1.0f;

    ReactionTargets() { authorityScale.Fill(1.0f); }
};

enum class Channel : std::uint8_t { Locomotion, Recovery, UpperBody, Posture, Fall, Count };

class REFLEX_API Behavior {
public:
    virtual ~Behavior() = default;
    virtual const char* Name() const = 0;
    virtual Channel GetChannel() const = 0;
    /// Utility score 0..1. Anything below `DecisionLayer::minScore` is ignored.
    virtual float Score(const BehaviorContext& ctx) const = 0;
    /// Write targets for this behaviour scaled by `weight` (0..1) and append a short clause to `clause`
    /// describing the action in words (for the reason string).
    virtual void Apply(const BehaviorContext& ctx, float weight, ReactionTargets& out, std::string& clause) const = 0;
};

struct ActiveBehavior {
    const char* name = "";
    Channel channel = Channel::Locomotion;
    float score = 0.0f;
};

struct Decision {
    ReactionTargets targets;
    std::vector<ActiveBehavior> active;
    std::string reason;
};

class REFLEX_API DecisionLayer {
public:
    /// Creates the layer with the built-in behaviours: Brace, StumbleRecover, LimpAway, SeekCover,
    /// TurnAndLeanAway, ProtectHead, ProtectWound, BreakFall, Collapse, Incapacitated.
    DecisionLayer();
    ~DecisionLayer();

    void AddBehavior(std::unique_ptr<Behavior> b);
    bool RemoveBehavior(const std::string& name);
    bool SetEnabled(const std::string& name, bool enabled);
    int BehaviorCount() const { return static_cast<int>(behaviors_.size()); }

    /// Run perceive->assess->decide. The returned reference stays valid until the next call.
    const Decision& Decide(const BehaviorContext& ctx, float dt);
    const Decision& Last() const { return decision_; }
    void Reset();

    float minScore = 0.08f;          ///< behaviours scoring below this stay off
    float hysteresis = 0.10f;        ///< bonus given to last tick's winner to prevent flicker
    float reasonRefreshSeconds = 0.25f;

private:
    struct Entry {
        std::unique_ptr<Behavior> behavior;
        bool enabled = true;
    };
    std::vector<Entry> behaviors_;
    Decision decision_;
    std::vector<const Behavior*> lastWinners_;
    float reasonTimer_ = 1e9f;
    std::string lastKey_;
};

/// Short description of what the character is reacting to (first half of the reason string).
REFLEX_API std::string DescribeSituation(const BehaviorContext& ctx);

} // namespace reflex
