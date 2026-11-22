#include "reflex/backend/jolt_backend.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <atomic>
#include <unordered_map>
#include <vector>

namespace reflex {
namespace {

namespace Layers {
constexpr JPH::ObjectLayer NON_MOVING = 0;
constexpr JPH::ObjectLayer MOVING = 1;
} // namespace Layers

namespace BPLayers {
constexpr JPH::BroadPhaseLayer NON_MOVING(0);
constexpr JPH::BroadPhaseLayer MOVING(1);
constexpr JPH::uint NUM = 2;
} // namespace BPLayers

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        if (a == Layers::NON_MOVING) return b == Layers::MOVING;
        return true;
    }
};

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BPLayers::NUM; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer l) const override {
        return l == Layers::NON_MOVING ? BPLayers::NON_MOVING : BPLayers::MOVING;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer l) const override {
        return l == BPLayers::NON_MOVING ? "NON_MOVING" : "MOVING";
    }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer l, JPH::BroadPhaseLayer bp) const override {
        if (l == Layers::NON_MOVING) return bp == BPLayers::MOVING;
        return true;
    }
};

inline JPH::Vec3 ToJ(const Vec3& v) { return JPH::Vec3(v.x, v.y, v.z); }
inline JPH::RVec3 ToJR(const Vec3& v) { return JPH::RVec3(v.x, v.y, v.z); }
inline JPH::Quat ToJ(const Quat& q) { return JPH::Quat(q.x, q.y, q.z, q.w); }
inline Vec3 FromJ(JPH::Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
inline Quat FromJ(JPH::QuatArg q) { return {q.GetX(), q.GetY(), q.GetZ(), q.GetW()}; }

std::atomic<int> gJoltUsers{0};

void JoltAcquire() {
    if (gJoltUsers.fetch_add(1) == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}
void JoltRelease() {
    if (gJoltUsers.fetch_sub(1) == 1) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

struct JointRecord {
    JPH::Ref<JPH::TwoBodyConstraint> constraint;
    JointType type = JointType::Ball;
    BodyHandle parent = kInvalidHandle;
    Quat restRelative; ///< parent^-1 * child at creation
    Vec3 axisInParent; ///< hinge axis expressed in the parent body frame
    float minAngle = 0, maxAngle = 0;
};

class JoltBackend final : public IPhysicsBackend {
public:
    explicit JoltBackend(const JoltBackendOptions& o) : opt_(o) {
        JoltAcquire();
        temp_ = std::make_unique<JPH::TempAllocatorImpl>(16 * 1024 * 1024);
        jobs_ = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
        system_.Init(o.maxBodies, 0, o.maxBodyPairs, o.maxContactConstraints, bpInterface_, objVsBp_, pairFilter_);
        JPH::PhysicsSettings ps = system_.GetPhysicsSettings();
        ps.mNumVelocitySteps = static_cast<JPH::uint>(o.velocitySteps);
        ps.mNumPositionSteps = static_cast<JPH::uint>(o.positionSteps);
        ps.mAllowSleeping = false;
        system_.SetPhysicsSettings(ps);
        system_.SetGravity(JPH::Vec3(0, -kGravity, 0));
    }

    ~JoltBackend() override {
        for (auto& j : joints_)
            if (j.constraint) system_.RemoveConstraint(j.constraint);
        joints_.clear();
        JPH::BodyInterface& bi = system_.GetBodyInterface();
        for (auto id : bodies_) {
            if (!id.IsInvalid()) {
                bi.RemoveBody(id);
                bi.DestroyBody(id);
            }
        }
        bodies_.clear();
        groups_.clear();
        jobs_.reset();
        temp_.reset();
        JoltRelease();
    }

    void SetGravity(const Vec3& g) override { system_.SetGravity(ToJ(g)); }

    BodyHandle CreateBody(const BodyDesc& d) override {
        JPH::Ref<JPH::Shape> shape;
        switch (d.shape.type) {
            case ShapeType::Box: {
                const Vec3& he = d.shape.halfExtents;
                float cr = std::min(0.02f, std::min(he.x, std::min(he.y, he.z)) * 0.5f);
                shape = new JPH::BoxShape(ToJ(he), cr);
                break;
            }
            case ShapeType::Sphere: shape = new JPH::SphereShape(d.shape.radius); break;
            case ShapeType::Capsule: shape = new JPH::CapsuleShape(d.shape.halfHeight, d.shape.radius); break;
        }
        JPH::BodyCreationSettings s(shape, ToJR(d.transform.position), ToJ(d.transform.rotation).Normalized(),
                                    d.isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic,
                                    d.isStatic ? Layers::NON_MOVING : Layers::MOVING);
        s.mFriction = d.friction;
        s.mRestitution = d.restitution;
        s.mLinearDamping = d.linearDamping;
        s.mAngularDamping = d.angularDamping;
        s.mAllowSleeping = false;
        s.mUserData = d.userData;
        s.mMaxAngularVelocity = 40.0f;
        if (!d.isStatic) {
            s.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            s.mMassPropertiesOverride.mMass = d.mass;
        }
        if (d.collisionGroup != 0) {
            auto& table = groups_[d.collisionGroup];
            if (!table) table = new JPH::GroupFilterTable(32);
            s.mCollisionGroup.SetGroupFilter(table);
            s.mCollisionGroup.SetGroupID(d.collisionGroup);
            s.mCollisionGroup.SetSubGroupID(d.subGroup);
        }
        JPH::BodyID id = system_.GetBodyInterface().CreateAndAddBody(s, JPH::EActivation::Activate);
        return StoreBody(id);
    }

    void DestroyBody(BodyHandle h) override {
        if (h >= bodies_.size() || bodies_[h].IsInvalid()) return;
        JPH::BodyInterface& bi = system_.GetBodyInterface();
        bi.RemoveBody(bodies_[h]);
        bi.DestroyBody(bodies_[h]);
        bodies_[h] = JPH::BodyID();
        freeBodies_.push_back(h);
    }

    JointHandle CreateJoint(const JointDesc& d) override {
        JPH::BodyInterface& bi = system_.GetBodyInterface();
        JPH::BodyID pa = bodies_.at(d.parent), ch = bodies_.at(d.child);
        JointRecord rec;
        rec.type = d.type;
        rec.parent = d.parent;
        Quat qp = FromJ(bi.GetRotation(pa)), qc = FromJ(bi.GetRotation(ch));
        rec.restRelative = qp.Conjugate() * qc;
        rec.axisInParent = qp.Conjugate().Rotate(Normalize(d.axis));
        rec.minAngle = d.minAngle;
        rec.maxAngle = d.maxAngle;

        {
            JPH::BodyID ids[2] = {pa, ch};
            JPH::BodyLockMultiWrite lock(system_.GetBodyLockInterface(), ids, 2);
            JPH::Body* b1 = lock.GetBody(0);
            JPH::Body* b2 = lock.GetBody(1);
            if (d.type == JointType::Hinge) {
                JPH::HingeConstraintSettings s;
                s.mSpace = JPH::EConstraintSpace::WorldSpace;
                s.mPoint1 = s.mPoint2 = ToJR(d.anchor);
                s.mHingeAxis1 = s.mHingeAxis2 = ToJ(Normalize(d.axis));
                s.mNormalAxis1 = s.mNormalAxis2 = ToJ(Normalize(d.normalAxis));
                s.mLimitsMin = d.minAngle;
                s.mLimitsMax = d.maxAngle;
                rec.constraint = s.Create(*b1, *b2);
            } else {
                JPH::SwingTwistConstraintSettings s;
                s.mSpace = JPH::EConstraintSpace::WorldSpace;
                s.mPosition1 = s.mPosition2 = ToJR(d.anchor);
                s.mTwistAxis1 = s.mTwistAxis2 = ToJ(Normalize(d.axis));
                s.mPlaneAxis1 = s.mPlaneAxis2 = ToJ(Normalize(d.normalAxis));
                s.mNormalHalfConeAngle = d.swingNormal;
                s.mPlaneHalfConeAngle = d.swingPlane;
                s.mTwistMinAngle = d.minAngle;
                s.mTwistMaxAngle = d.maxAngle;
                s.mMaxFrictionTorque = 0.0f;
                rec.constraint = s.Create(*b1, *b2);
            }
        }
        system_.AddConstraint(rec.constraint);
        if (!freeJoints_.empty()) {
            JointHandle i = freeJoints_.back();
            freeJoints_.pop_back();
            joints_[i] = std::move(rec);
            return i;
        }
        joints_.push_back(std::move(rec));
        return static_cast<JointHandle>(joints_.size() - 1);
    }

    void DestroyJoint(JointHandle h) override {
        if (h >= joints_.size() || !joints_[h].constraint) return;
        system_.RemoveConstraint(joints_[h].constraint);
        joints_[h].constraint = nullptr;
        freeJoints_.push_back(h);
    }

    void SetCollisionPairEnabled(BodyHandle a, BodyHandle b, bool enabled) override {
        if (enabled) return; // the group filter table only supports disabling pairs
        JPH::BodyID ids[2] = {bodies_.at(a), bodies_.at(b)};
        JPH::BodyLockMultiRead lock(system_.GetBodyLockInterface(), ids, 2);
        const JPH::Body* ba = lock.GetBody(0);
        const JPH::Body* bb = lock.GetBody(1);
        if (!ba || !bb) return;
        const JPH::CollisionGroup& ga = ba->GetCollisionGroup();
        const JPH::CollisionGroup& gb = bb->GetCollisionGroup();
        if (ga.GetGroupID() != gb.GetGroupID() || ga.GetGroupFilter() == nullptr) return;
        auto* table = static_cast<JPH::GroupFilterTable*>(const_cast<JPH::GroupFilter*>(ga.GetGroupFilter()));
        table->DisableCollision(ga.GetSubGroupID(), gb.GetSubGroupID());
    }

    void SetJointDrive(JointHandle h, const JointDrive& drive) override {
        JointRecord& r = joints_.at(h);
        bool on = drive.stiffness > 1e-4f && drive.maxTorque > 1e-4f;
        if (r.type == JointType::Hinge) {
            auto* c = static_cast<JPH::HingeConstraint*>(r.constraint.GetPtr());
            if (!on) {
                c->SetMotorState(JPH::EMotorState::Off);
                return;
            }
            JPH::MotorSettings& m = c->GetMotorSettings();
            m.mSpringSettings = JPH::SpringSettings(JPH::ESpringMode::StiffnessAndDamping, drive.stiffness, drive.damping);
            m.SetTorqueLimit(drive.maxTorque);
            float angle = Dot(drive.target.ToRotationVector(), r.axisInParent);
            c->SetTargetAngle(Clamp(angle, r.minAngle, r.maxAngle));
            c->SetMotorState(JPH::EMotorState::Position);
        } else {
            auto* c = static_cast<JPH::SwingTwistConstraint*>(r.constraint.GetPtr());
            if (!on) {
                c->SetSwingMotorState(JPH::EMotorState::Off);
                c->SetTwistMotorState(JPH::EMotorState::Off);
                return;
            }
            JPH::SpringSettings sp(JPH::ESpringMode::StiffnessAndDamping, drive.stiffness, drive.damping);
            c->GetSwingMotorSettings().mSpringSettings = sp;
            c->GetSwingMotorSettings().SetTorqueLimit(drive.maxTorque);
            c->GetTwistMotorSettings().mSpringSettings = sp;
            c->GetTwistMotorSettings().SetTorqueLimit(drive.maxTorque);
            Quat rel = (drive.target * r.restRelative).Normalized();
            c->SetTargetOrientationBS(ToJ(rel));
            c->SetSwingMotorState(JPH::EMotorState::Position);
            c->SetTwistMotorState(JPH::EMotorState::Position);
        }
    }

    BodyState GetBodyState(BodyHandle h) const override {
        const JPH::BodyInterface& bi = system_.GetBodyInterface();
        JPH::BodyID id = bodies_.at(h);
        BodyState s;
        JPH::RVec3 p;
        JPH::Quat q;
        bi.GetPositionAndRotation(id, p, q);
        s.transform.position = {static_cast<float>(p.GetX()), static_cast<float>(p.GetY()),
                                static_cast<float>(p.GetZ())};
        s.transform.rotation = FromJ(q);
        s.linearVelocity = FromJ(bi.GetLinearVelocity(id));
        s.angularVelocity = FromJ(bi.GetAngularVelocity(id));
        return s;
    }

    void SetBodyState(BodyHandle h, const BodyState& s) override {
        JPH::BodyInterface& bi = system_.GetBodyInterface();
        JPH::BodyID id = bodies_.at(h);
        bi.SetPositionAndRotation(id, ToJR(s.transform.position), ToJ(s.transform.rotation).Normalized(),
                                  JPH::EActivation::Activate);
        bi.SetLinearVelocity(id, ToJ(s.linearVelocity));
        bi.SetAngularVelocity(id, ToJ(s.angularVelocity));
    }

    void AddForce(BodyHandle h, const Vec3& f) override { system_.GetBodyInterface().AddForce(bodies_.at(h), ToJ(f)); }
    void AddTorque(BodyHandle h, const Vec3& t) override { system_.GetBodyInterface().AddTorque(bodies_.at(h), ToJ(t)); }
    void AddImpulse(BodyHandle h, const Vec3& imp, const Vec3& p) override {
        system_.GetBodyInterface().AddImpulse(bodies_.at(h), ToJ(imp), ToJR(p));
    }

    bool RayCast(const Vec3& origin, const Vec3& dir, float maxDist, bool staticOnly, RayHit& out) const override {
        Vec3 n = Normalize(dir, {0, -1, 0});
        JPH::RRayCast ray(ToJR(origin), ToJ(n * maxDist));
        JPH::RayCastResult res;
        bool hit;
        if (staticOnly) {
            JPH::SpecifiedObjectLayerFilter olf(Layers::NON_MOVING);
            JPH::SpecifiedBroadPhaseLayerFilter bpf(BPLayers::NON_MOVING);
            hit = system_.GetNarrowPhaseQuery().CastRay(ray, res, bpf, olf);
        } else {
            hit = system_.GetNarrowPhaseQuery().CastRay(ray, res);
        }
        if (!hit) return false;
        out.distance = res.mFraction * maxDist;
        out.point = origin + n * out.distance;
        out.body = kInvalidHandle;
        out.userData = 0;
        JPH::BodyLockRead lock(system_.GetBodyLockInterface(), res.mBodyID);
        if (lock.Succeeded()) {
            const JPH::Body& b = lock.GetBody();
            out.userData = b.GetUserData();
            out.normal = FromJ(b.GetWorldSpaceSurfaceNormal(res.mSubShapeID2, ToJR(out.point)));
            for (size_t i = 0; i < bodies_.size(); ++i) {
                if (bodies_[i] == res.mBodyID) {
                    out.body = static_cast<BodyHandle>(i);
                    break;
                }
            }
        }
        return true;
    }

    void Step(float dt) override { system_.Update(dt, opt_.collisionSteps, temp_.get(), jobs_.get()); }

private:
    BodyHandle StoreBody(JPH::BodyID id) {
        if (!freeBodies_.empty()) {
            BodyHandle i = freeBodies_.back();
            freeBodies_.pop_back();
            bodies_[i] = id;
            return i;
        }
        bodies_.push_back(id);
        return static_cast<BodyHandle>(bodies_.size() - 1);
    }

    JoltBackendOptions opt_;
    BPLayerInterfaceImpl bpInterface_;
    ObjectVsBroadPhaseLayerFilterImpl objVsBp_;
    ObjectLayerPairFilterImpl pairFilter_;
    std::unique_ptr<JPH::TempAllocatorImpl> temp_;
    std::unique_ptr<JPH::JobSystemSingleThreaded> jobs_;
    JPH::PhysicsSystem system_;
    std::vector<JPH::BodyID> bodies_;
    std::vector<std::uint32_t> freeBodies_;
    std::vector<JointRecord> joints_;
    std::vector<std::uint32_t> freeJoints_;
    std::unordered_map<std::uint32_t, JPH::Ref<JPH::GroupFilterTable>> groups_;
};

} // namespace

std::unique_ptr<IPhysicsBackend> CreateJoltBackend(const JoltBackendOptions& options) {
    return std::make_unique<JoltBackend>(options);
}

} // namespace reflex
