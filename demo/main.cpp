// ReflexEngine demo: shoot a physics-driven character and watch it think.
//
//   Left mouse   fire a bullet at the point under the cursor (hit or near miss)
//   Right mouse  orbit the camera       Mouse wheel   zoom
//   R            reset                  Tab           toggle the debug overlay
//   A            toggle "game animation" (arms-up pose) vs procedural control
//
//   reflex_demo --capture <dir> [width height]   runs a scripted shooting sequence and writes PNG frames

#include <memory>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "reflex/backend/jolt_backend.h"
#include "reflex/world.h"

namespace rx = reflex;

namespace {

Vector3 V(const rx::Vec3& v) { return {v.x, v.y, v.z}; }
rx::Vec3 R(const Vector3& v) { return {v.x, v.y, v.z}; }

// ---- Skinned character model (CesiumMan, CC BY 4.0, (c) 2017 Cesium) -------------------------------------------------
// The mesh is posed from the physics bodies: each mapped bone takes its rotation from the matching body part.
struct Skinned {
    Model model{};
    ModelAnimation anim{};
    std::vector<Vector3> bindPos;   // scaled bind positions (model space)
    std::vector<Quaternion> bindRot;
    bool loaded = false;
};

constexpr int kModelBones = 19;
constexpr float kModelScale = 1.75f / 1.51f; // the model is 1.51 m tall
// CesiumMan bone index -> body part (-1 = unmapped toe bones)
const int kBonePart[kModelBones] = {0, 1, 2, 3, 4, 5, 8, 6, 9, 7, 10, 11, 14, 12, 15, 13, 16, -1, -1};
// swing bones aim at a child bone: bone -> child bone (-1 = rotate with the body part)
const int kBoneChild[kModelBones] = {-1, -1, -1, -1, -1, 7, 8, 9, 10, -2, -2, 13, 14, 15, 16, -1, -1, -1, -1};

bool LoadSkinned(Skinned& sk) {
    std::string path = std::string(GetApplicationDirectory()) + "assets/CesiumMan.glb";
    sk.model = LoadModel(path.c_str());
    if (sk.model.meshCount < 1 || sk.model.boneCount != kModelBones || sk.model.meshes[0].animVertices == nullptr) {
        TraceLog(LOG_WARNING, "Could not load %s, using the primitive figure", path.c_str());
        return false;
    }
    for (int i = 0; i < kModelBones; ++i) {
        sk.model.bindPose[i].translation = Vector3Scale(sk.model.bindPose[i].translation, kModelScale);
        sk.bindPos.push_back(sk.model.bindPose[i].translation);
        sk.bindRot.push_back(sk.model.bindPose[i].rotation);
    }
    Mesh& m = sk.model.meshes[0];
    for (int i = 0; i < m.vertexCount * 3; ++i) m.vertices[i] *= kModelScale;
    sk.anim.boneCount = kModelBones;
    sk.anim.frameCount = 1;
    sk.anim.bones = static_cast<BoneInfo*>(MemAlloc(kModelBones * sizeof(BoneInfo)));
    for (int i = 0; i < kModelBones; ++i) sk.anim.bones[i] = sk.model.bones[i];
    sk.anim.framePoses = static_cast<Transform**>(MemAlloc(sizeof(Transform*)));
    sk.anim.framePoses[0] = static_cast<Transform*>(MemAlloc(kModelBones * sizeof(Transform)));
    sk.loaded = true;
    return true;
}

void FreeSkinned(Skinned& sk) {
    if (!sk.loaded) return;
    UnloadModelAnimation(sk.anim);
    UnloadModel(sk.model);
    sk.loaded = false;
}

Quaternion Q(const rx::Quat& q) { return {q.x, q.y, q.z, q.w}; }

void PoseSkinned(Skinned& sk, const rx::Character& c) {
    if (!sk.loaded) return;
    Quaternion rot[kModelBones];
    Vector3 pos[kModelBones];
    Quaternion delta[kModelBones];
    auto bodyRot = [&](int part) { return c.GetBodyPartTransform(static_cast<rx::BodyPart>(part)).rotation; };
    for (int i = 0; i < kModelBones; ++i) {
        int par = sk.model.bones[i].parent;
        int part = kBonePart[i];
        if (part >= 0) {
            rx::Quat br = bodyRot(part);
            if (kBoneChild[i] == -1) {
                rot[i] = QuaternionMultiply(Q(br), sk.bindRot[i]); // follows the body part
            } else {
                Vector3 from;
                if (kBoneChild[i] >= 0) from = Vector3Normalize(Vector3Subtract(sk.bindPos[kBoneChild[i]], sk.bindPos[i]));
                else from = Vector3Normalize(Vector3Subtract(sk.bindPos[i], sk.bindPos[par])); // hand continues the forearm
                rx::Vec3 d = br.Rotate({0, -1, 0});
                // a touch of outward swing keeps the mesh arms clear of the torso
                if (part == static_cast<int>(rx::BodyPart::LeftUpperArm) || part == static_cast<int>(rx::BodyPart::RightUpperArm))
                    d = rx::Normalize(d + rx::Vec3{(part == static_cast<int>(rx::BodyPart::LeftUpperArm) ? 0.18f : -0.18f), 0, 0});
                Quaternion swing = QuaternionFromVector3ToVector3(from, V(d));
                rot[i] = QuaternionMultiply(swing, sk.bindRot[i]);
            }
        } else if (par >= 0) {
            rot[i] = QuaternionMultiply(delta[par], sk.bindRot[i]);
        } else {
            rot[i] = sk.bindRot[i];
        }
        delta[i] = QuaternionMultiply(rot[i], QuaternionInvert(sk.bindRot[i]));
        if (par < 0) {
            rx::Vec3 pp = c.GetBodyPartTransform(rx::BodyPart::Pelvis).position;
            pos[i] = {pp.x, pp.y, pp.z}; // the model pelvis sits at the physics pelvis
        } else {
            Vector3 off = Vector3Subtract(sk.bindPos[i], sk.bindPos[par]);
            pos[i] = Vector3Add(pos[par], Vector3RotateByQuaternion(off, delta[par]));
        }
        sk.anim.framePoses[0][i].translation = pos[i];
        sk.anim.framePoses[0][i].rotation = rot[i];
        sk.anim.framePoses[0][i].scale = {1, 1, 1};
    }
    UpdateModelAnimation(sk.model, sk.anim, 0);
}

struct StaticBox {
    rx::Vec3 center, half;
    Color color;
};

struct Marker {
    rx::Vec3 pos;
    float life;
    Color color;
};
struct Tracer {
    rx::Vec3 a, b;
    float life;
};

struct Ball {
    rx::Vec3 pos, dir;
    float speed = 0;
    float age = 0;
    std::vector<int> hit; // indices of characters already struck (a ball hits each one once)
};

constexpr float kBallRadius = 0.45f;
constexpr float kBallMass = 60.0f; // kg: a giant ball

struct App {
    int scene = 0; // 0 = shooting range, 1 = force lab
    bool charging = false;
    float chargeTime = 0.0f;
    std::vector<Ball> balls;
    std::vector<rx::Character> crowd; // extra characters standing in a line behind the main one
    bool useModel = false;            // M toggles between the skinned model and the primitive figure
    Skinned mainSkin;
    std::vector<std::unique_ptr<Skinned>> crowdSkin;
    std::unique_ptr<rx::IPhysicsBackend> backend;
    std::unique_ptr<rx::World> world;
    rx::Character character;
    std::vector<StaticBox> boxes;
    std::vector<Marker> markers;
    std::vector<Tracer> tracers;
    float hitFlash[rx::kBodyPartCount] = {};
    bool overlay = true;
    bool useAnimation = false;
    rx::Pose animPose;
    // camera
    float camYaw = 0.55f, camPitch = 0.28f, camDist = 5.2f;
    Vector3 camTarget{0, 1.0f, 0};
    Camera3D camera{};
    int shots = 0;
    std::string lastShot = "-";
};

App* gApp = nullptr;

// Physics rig. With the skinned model the legs/lower torso follow its (shorter-legged) proportions.
rx::CharacterDesc MakeDesc(bool model, float height) {
    if (!model) return rx::CharacterDesc::DefaultHumanoid(height, 75.0f);
    auto p = rx::DefaultHumanoidJointPositions(1.75f);
    auto set = [&](rx::BodyPart part, rx::Vec3 v) { p[static_cast<size_t>(rx::Index(part))] = v; };
    set(rx::BodyPart::Pelvis, {0, 0.787f, 0});
    set(rx::BodyPart::Spine, {0, 0.955f, 0});
    set(rx::BodyPart::Chest, {0, 1.10f, 0});
    for (float sg : {1.0f, -1.0f}) {
        bool left = sg > 0;
        set(left ? rx::BodyPart::LeftUpperLeg : rx::BodyPart::RightUpperLeg, {0.09f * sg, 0.71f, 0});
        set(left ? rx::BodyPart::LeftLowerLeg : rx::BodyPart::RightLowerLeg, {0.09f * sg, 0.41f, 0});
        set(left ? rx::BodyPart::LeftFoot : rx::BodyPart::RightFoot, {0.09f * sg, 0.10f, 0});
    }
    rx::CharacterDesc d;
    rx::HumanoidRig rig = rx::MakeHumanoidRig(p);
    d.skeleton = std::move(rig.skeleton);
    d.mapping = rig.mapping;
    d.params.height = height;
    d.params.mass = 75.0f;
    return d;
}

void AddStaticBox(App& a, rx::Vec3 center, rx::Vec3 half, Color color, bool addCover = true) {
    rx::BodyDesc b;
    b.shape.type = rx::ShapeType::Box;
    b.shape.halfExtents = half;
    b.isStatic = true;
    b.transform.position = center;
    b.friction = 1.0f;
    a.backend->CreateBody(b);
    a.boxes.push_back({center, half, color});
    (void)addCover;
}

void BuildScene(App& a, int scene) {
    a.backend = rx::CreateJoltBackend();
    AddStaticBox(a, {0, -0.5f, 0}, {40, 0.5f, 40}, Color{70, 74, 82, 255});
    a.boxes.back().color = Color{58, 62, 70, 255};
    if (scene == 1) {
        // Force lab: open floor with a brick wall and crates behind the character to be thrown into.
        AddStaticBox(a, {0.0f, 2.0f, -14.0f}, {9.0f, 2.0f, 0.5f}, Color{150, 82, 70, 255});
        AddStaticBox(a, {-4.0f, 0.5f, -10.0f}, {0.5f, 0.5f, 0.5f}, Color{176, 128, 72, 255});
        AddStaticBox(a, {-4.0f, 1.5f, -10.0f}, {0.5f, 0.5f, 0.5f}, Color{150, 108, 62, 255});
        AddStaticBox(a, {4.0f, 0.5f, -9.0f}, {0.5f, 0.5f, 0.5f}, Color{176, 128, 72, 255});
        AddStaticBox(a, {0.0f, 0.15f, -7.0f}, {1.2f, 0.15f, 1.5f}, Color{110, 114, 124, 255});
    } else {
    // Cover: crates and walls around the character
    AddStaticBox(a, {3.0f, 0.5f, -2.5f}, {0.5f, 0.5f, 0.5f}, Color{176, 128, 72, 255});
    AddStaticBox(a, {-3.2f, 0.5f, -1.5f}, {0.5f, 0.5f, 0.5f}, Color{176, 128, 72, 255});
    AddStaticBox(a, {-3.2f, 1.5f, -1.5f}, {0.5f, 0.5f, 0.5f}, Color{150, 108, 62, 255});
    AddStaticBox(a, {0.0f, 1.0f, -5.5f}, {3.0f, 1.0f, 0.2f}, Color{120, 124, 134, 255});
    AddStaticBox(a, {4.5f, 1.0f, 2.0f}, {0.2f, 1.0f, 1.8f}, Color{120, 124, 134, 255});
    }

    a.world = std::make_unique<rx::World>(*a.backend);
    rx::CharacterDesc desc = MakeDesc(a.useModel, 1.75f);
    desc.spawn.position = {0, 0, 0};
    desc.spawn.rotation = rx::Quat::Identity();
    a.character = a.world->CreateCharacter(desc);

    // The character knows about the cover objects: standing behind one on the side away from the shooter.
    std::vector<StaticBox> cover(a.boxes.begin() + 1, a.boxes.end());
    if (scene == 0) a.character.Threat().SetCoverQuery([cover](const rx::Vec3& from, const rx::Vec3& towardShooter,
                                               rx::Vec3& outCover) {
        float best = 1e9f;
        bool found = false;
        for (const StaticBox& b : cover) {
            rx::Vec3 d = rx::Horizontal(towardShooter);
            // half extent of the box along the shooting direction
            float depth = std::abs(d.x) * b.half.x + std::abs(d.z) * b.half.z;
            rx::Vec3 stand = rx::Horizontal(b.center) - d * (depth + 0.55f);
            float dist = rx::Length(stand - rx::Horizontal(from));
            // the cover must be on the shooter's side of the point we stand at
            rx::Vec3 toBox = rx::Horizontal(b.center) - rx::Horizontal(from);
            if (rx::Dot(toBox, d) < -0.2f) continue; // box is behind us relative to the shooter
            if (dist < best && dist < 12.0f) {
                best = dist;
                outCover = stand;
                found = true;
            }
        }
        return found;
    });

    // "Game animation": a hands-up pose, used to demonstrate blending procedural <-> authored animation.
    const rx::CharacterDesc& d = a.character.Desc();
    for (const rx::BoneDesc& b : d.skeleton.bones) a.animPose.local.push_back(b.bindLocal);
    auto set = [&](rx::BodyPart p, rx::Quat q) { a.animPose.local[static_cast<size_t>(d.mapping[p])].rotation = q; };
    set(rx::BodyPart::LeftUpperArm, rx::Quat::FromAxisAngle({1, 0, 0}, -2.6f) * rx::Quat::FromAxisAngle({0, 0, 1}, 0.3f));
    set(rx::BodyPart::RightUpperArm, rx::Quat::FromAxisAngle({1, 0, 0}, -2.6f) * rx::Quat::FromAxisAngle({0, 0, 1}, -0.3f));
}

void LoadScene(App& a, int scene) {
    FreeSkinned(a.mainSkin);
    for (auto& k : a.crowdSkin) FreeSkinned(*k);
    a.crowdSkin.clear();
    a.crowd.clear();
    a.character = rx::Character(); // release the characters before their world goes away
    a.world.reset();
    a.backend.reset();
    a.boxes.clear();
    a.markers.clear();
    a.tracers.clear();
    a.balls.clear();
    a.animPose.local.clear();
    a.charging = false;
    for (float& f : a.hitFlash) f = 0;
    a.scene = scene;
    a.shots = 0;
    a.lastShot = "-";
    a.useAnimation = false;
    BuildScene(a, scene);
    for (int i = 0; i < 120; ++i) a.world->Step(1.0f / 120.0f);
    if (a.useModel && !LoadSkinned(a.mainSkin)) a.useModel = false;
    a.camTarget = {0, 1.0f, 0};
}

// A row of characters one behind the other along the ball path: shoot a ball through them all.
void SpawnCrowd(App& a, int count) {
    for (auto& k : a.crowdSkin) FreeSkinned(*k);
    a.crowdSkin.clear();
    a.crowd.clear();
    for (int i = 1; i <= count; ++i) {
        rx::CharacterDesc d = MakeDesc(a.useModel, 1.75f + (a.useModel ? 0.0f : 0.04f * static_cast<float>((i * 7) % 5) - 0.08f));
        d.spawn.position = {0.12f * static_cast<float>((i * 5) % 3 - 1), 0.0f, -1.5f * static_cast<float>(i)};
        a.crowd.push_back(a.world->CreateCharacter(d));
        if (a.useModel) {
            auto sk = std::make_unique<Skinned>();
            LoadSkinned(*sk);
            a.crowdSkin.push_back(std::move(sk));
        }
    }
    for (int i = 0; i < 120; ++i) a.world->Step(1.0f / 120.0f);
}

void UpdateCamera(App& a) {
    a.camera.target = a.camTarget;
    a.camera.position = {a.camTarget.x + a.camDist * std::cos(a.camPitch) * std::sin(a.camYaw),
                         a.camTarget.y + a.camDist * std::sin(a.camPitch),
                         a.camTarget.z + a.camDist * std::cos(a.camPitch) * std::cos(a.camYaw)};
    a.camera.up = {0, 1, 0};
    a.camera.fovy = 55.0f;
    a.camera.projection = CAMERA_PERSPECTIVE;
}

// ---- Shooting ---------------------------------------------------------------------------------------------------
void FireAt(App& a, const rx::Vec3& shooter, const rx::Vec3& target) {
    rx::Vec3 dir = rx::Normalize(target - shooter);
    rx::BulletParams params; // 35 damage, head x2.5
    rx::BulletResult r = a.world->TraceBullet(shooter, dir, 100.0f, params);
    rx::Vec3 end = shooter + dir * (r.type == rx::BulletResult::Type::Miss ? 60.0f : r.distance);
    if (r.type == rx::BulletResult::Type::NearMiss) end = shooter + dir * 60.0f;
    a.tracers.push_back({shooter, end, 0.12f});
    ++a.shots;
    switch (r.type) {
        case rx::BulletResult::Type::HitCharacter:
            a.markers.push_back({r.point, 1.2f, RED});
            a.hitFlash[rx::Index(r.part)] = 1.0f;
            a.lastShot = std::string("HIT ") + rx::BodyPartName(r.part);
            break;
        case rx::BulletResult::Type::NearMiss:
            a.markers.push_back({r.point, 1.2f, YELLOW});
            a.lastShot = std::string("NEAR MISS (") + rx::BodyPartName(r.part) + ", " +
                         std::to_string(static_cast<int>(r.distance * 100)) + " cm)";
            break;
        case rx::BulletResult::Type::HitScenery:
            a.markers.push_back({r.point, 0.8f, WHITE});
            a.lastShot = "hit scenery";
            break;
        default: a.lastShot = "miss"; break;
    }
}

void ShootFromCamera(App& a, Vector2 mouse) {
    Ray ray = GetMouseRay(mouse, a.camera);
    rx::RayHit hit;
    rx::Vec3 aim = R(ray.position) + R(ray.direction) * 60.0f;
    if (a.backend->RayCast(R(ray.position), R(ray.direction), 100.0f, false, hit)) aim = hit.point;
    // the shooter stands a little right of and below the camera so the tracer is visible
    rx::Vec3 right = R(Vector3CrossProduct(ray.direction, {0, 1, 0}));
    rx::Vec3 shooter = R(ray.position) + rx::Normalize(right) * 0.6f + rx::Vec3{0, -0.35f, 0};
    FireAt(a, shooter, aim);
}

// ---- Force lab: hold to charge, release to launch a giant ball -------------------------------------------------
float ChargeFraction(const App& a) { return rx::Saturate(a.chargeTime / 0.7f); }
float BallSpeed(float charge) { return 6.0f + 36.0f * charge; }

void FireBall(App& a, Vector2 mouse, float charge) {
    Ray ray = GetMouseRay(mouse, a.camera);
    rx::RayHit hit;
    rx::Vec3 aim = R(ray.position) + R(ray.direction) * 30.0f;
    if (a.backend->RayCast(R(ray.position), R(ray.direction), 100.0f, false, hit)) aim = hit.point;
    rx::Vec3 right = R(Vector3CrossProduct(ray.direction, {0, 1, 0}));
    rx::Vec3 shooter = R(ray.position) + rx::Normalize(right) * 0.6f + rx::Vec3{0, -0.4f, 0};
    Ball b;
    b.pos = shooter;
    b.dir = rx::Normalize(aim - shooter);
    b.speed = BallSpeed(charge);
    a.balls.push_back(b);
    ++a.shots;
}

void UpdateBalls(App& a, float dt) {
    std::vector<rx::Character*> everyone;
    everyone.push_back(&a.character);
    for (rx::Character& c : a.crowd) everyone.push_back(&c);
    for (Ball& b : a.balls) {
        float step = b.speed * dt;
        b.age += dt;
        rx::RayHit wall;
        bool hitWall = a.backend->RayCast(b.pos, b.dir, step + kBallRadius, true, wall);
        for (size_t k = 0; k < everyone.size(); ++k) {
            int idx = static_cast<int>(k);
            if (std::find(b.hit.begin(), b.hit.end(), idx) != b.hit.end()) continue;
            rx::BodyPart part;
            rx::Vec3 pt;
            float d = everyone[k]->ClosestApproach(b.pos, b.dir, step, part, pt);
            if (d > kBallRadius) continue;
            // The ball hits: it launches this character along its direction with a little lift and carries on.
            rx::Vec3 push = rx::Normalize(b.dir + rx::Vec3{0, 0.2f, 0});
            float impulse = kBallMass * b.speed;
            everyone[k]->ApplyImpact(push * impulse, pt);
            b.hit.push_back(idx);
            a.markers.push_back({pt, 1.0f, ORANGE});
            a.lastShot = std::string("FORCE ") + std::to_string(static_cast<int>(b.speed)) + " m/s -> launch " +
                         std::to_string(static_cast<int>(impulse / 75.0f)) + " m/s, hit " + std::to_string(b.hit.size());
            b.speed *= 0.93f; // each body takes a little momentum
        }
        if (hitWall || b.speed < 4.0f) {
            if (hitWall) a.markers.push_back({wall.point, 0.6f, WHITE});
            b.age = 99.0f;
        } else {
            b.pos = b.pos + b.dir * step;
        }
    }
    a.balls.erase(std::remove_if(a.balls.begin(), a.balls.end(), [](const Ball& b) { return b.age > 4.0f; }),
                  a.balls.end());
}

// ---- Drawing ----------------------------------------------------------------------------------------------------
Color HealthColor(float health, float flash) {
    float h = rx::Saturate(health);
    Color base = {static_cast<unsigned char>(90 + 150 * (1 - h)), static_cast<unsigned char>(120 + 90 * h),
                  static_cast<unsigned char>(110 - 50 * (1 - h)), 255};
    if (flash > 0) {
        base.r = static_cast<unsigned char>(base.r + (255 - base.r) * flash);
        base.g = static_cast<unsigned char>(base.g * (1 - 0.7f * flash));
        base.b = static_cast<unsigned char>(base.b * (1 - 0.7f * flash));
    }
    return base;
}

void DrawCharacter(const App& a, const rx::Character& character, bool primary) {
    const rx::CharacterDebugInfo& info = character.GetDebugInfo();
    for (int i = 0; i < rx::kBodyPartCount; ++i) {
        rx::BodyPart p = static_cast<rx::BodyPart>(i);
        rx::Transform t = character.GetBodyPartTransform(p);
        rx::ShapeDesc sh = character.GetPartShape(p);
        Color c = HealthColor(info.parts[p].health, primary ? a.hitFlash[i] : 0.0f);
        rx::Vec3 axis = t.rotation.Rotate({0, 1, 0});
        if (sh.type == rx::ShapeType::Capsule) {
            Vector3 p0 = V(t.position - axis * sh.halfHeight), p1 = V(t.position + axis * sh.halfHeight);
            DrawCapsule(p0, p1, sh.radius, 10, 6, c);
        } else if (sh.type == rx::ShapeType::Sphere) {
            DrawSphere(V(t.position), sh.radius, c);
        } else {
            Quaternion q = {t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w};
            Matrix m = QuaternionToMatrix(q);
            float16 f = MatrixToFloatV(m);
            rlPushMatrix();
            rlTranslatef(t.position.x, t.position.y, t.position.z);
            rlMultMatrixf(f.v);
            DrawCube({0, 0, 0}, sh.halfExtents.x * 2, sh.halfExtents.y * 2, sh.halfExtents.z * 2, c);
            DrawCubeWires({0, 0, 0}, sh.halfExtents.x * 2, sh.halfExtents.y * 2, sh.halfExtents.z * 2, Fade(BLACK, 0.5f));
            rlPopMatrix();
        }
    }
    // face direction marker on the head
    rx::Transform ht = character.GetBodyPartTransform(rx::BodyPart::Head);
    rx::Vec3 nose = ht.position + ht.rotation.Rotate({0, 0, 0.13f});
    DrawSphere(V(nose), 0.03f, Color{240, 220, 200, 255});
}

void DrawDebug3D(const App& a) {
    const rx::CharacterDebugInfo& d = a.character.GetDebugInfo();
    // support polygon on the ground
    const rx::SupportPolygon& poly = d.supportPolygon;
    for (int i = 0; i < poly.count; ++i) {
        rx::Vec3 p = poly.points[i], q = poly.points[(i + 1) % poly.count];
        p.y = q.y = 0.012f;
        DrawLine3D(V(p), V(q), Color{80, 220, 120, 255});
    }
    rx::Vec3 comG{d.com.x, 0.02f, d.com.z};
    DrawLine3D(V(d.com), V(comG), Color{255, 200, 60, 200});
    DrawSphere(V(d.com), 0.045f, Color{255, 200, 60, 255});
    DrawSphere(V(comG), 0.03f, Color{255, 200, 60, 255});
    DrawSphere(V(rx::Vec3{d.capturePoint.x, 0.02f, d.capturePoint.z}), 0.04f, Color{80, 180, 255, 255});
    for (int i = 0; i < 2; ++i) {
        if (d.stepping[i]) DrawCylinder(V(rx::Vec3{d.footTarget[i].x, 0.0f, d.footTarget[i].z}), 0.05f, 0.05f, 0.015f, 12, Color{255, 120, 200, 255});
    }
}

void DrawScene(const App& a) {
    for (const StaticBox& b : a.boxes) {
        DrawCube(V(b.center), b.half.x * 2, b.half.y * 2, b.half.z * 2, b.color);
        DrawCubeWires(V(b.center), b.half.x * 2, b.half.y * 2, b.half.z * 2, Fade(BLACK, 0.35f));
    }
    DrawGrid(60, 1.0f);
    if (a.useModel && a.mainSkin.loaded) {
        DrawModel(a.mainSkin.model, {0, 0, 0}, 1.0f, WHITE);
        for (const auto& k : a.crowdSkin) DrawModel(k->model, {0, 0, 0}, 1.0f, WHITE);
    } else {
        DrawCharacter(a, a.character, true);
        for (const rx::Character& c : a.crowd) DrawCharacter(a, c, false);
    }
    for (const Ball& b : a.balls) {
        DrawSphere(V(b.pos), kBallRadius, Color{255, 140, 40, 255});
        DrawSphereWires(V(b.pos), kBallRadius, 12, 12, Fade(BLACK, 0.5f));
    }
    for (const Tracer& t : a.tracers) DrawLine3D(V(t.a), V(t.b), Fade(YELLOW, t.life / 0.12f));
    for (const Marker& m : a.markers) DrawSphere(V(m.pos), 0.05f + 0.04f * m.life, Fade(m.color, rx::Saturate(m.life)));
    if (a.overlay) DrawDebug3D(a);
}

// ---- UI ----------------------------------------------------------------------------------------------------------
const void* gActiveSlider = nullptr;

bool Slider(const void* id, const char* label, int x, int y, int w, float& value, float lo, float hi, bool enabled = true) {
    const int h = 14;
    Rectangle bar{static_cast<float>(x), static_cast<float>(y + 14), static_cast<float>(w), static_cast<float>(h)};
    Vector2 m = GetMousePosition();
    bool changed = false;
    if (enabled) {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(m, bar)) gActiveSlider = id;
        if (gActiveSlider == id) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                float t = rx::Saturate((m.x - bar.x) / bar.width);
                float nv = lo + t * (hi - lo);
                if (nv != value) {
                    value = nv;
                    changed = true;
                }
            } else {
                gActiveSlider = nullptr;
            }
        }
    }
    DrawText(TextFormat("%s %.2f", label, value), x, y, 12, enabled ? RAYWHITE : GRAY);
    DrawRectangleRec(bar, Color{40, 44, 52, 230});
    float t = (value - lo) / (hi - lo);
    DrawRectangle(x, y + 14, static_cast<int>(bar.width * rx::Saturate(t)), h, Color{80, 160, 240, 255});
    DrawRectangleLinesEx(bar, 1, Color{120, 126, 140, 255});
    return changed;
}

void Bar(int x, int y, int w, int h, float v, Color c) {
    DrawRectangle(x, y, w, h, Color{30, 33, 40, 230});
    DrawRectangle(x, y, static_cast<int>(w * rx::Saturate(v)), h, c);
    DrawRectangleLines(x, y, w, h, Color{90, 96, 110, 255});
}

void DrawWrapped(const std::string& text, int x, int y, int maxWidth, int fontSize, Color color) {
    std::string line, word;
    int yy = y;
    auto flush = [&]() {
        DrawText(line.c_str(), x, yy, fontSize, color);
        yy += fontSize + 3;
        line.clear();
    };
    for (size_t i = 0; i <= text.size(); ++i) {
        char ch = i < text.size() ? text[i] : ' ';
        if (ch == ' ') {
            std::string trial = line.empty() ? word : line + " " + word;
            if (MeasureText(trial.c_str(), fontSize) > maxWidth && !line.empty()) {
                flush();
                line = word;
            } else {
                line = trial;
            }
            word.clear();
        } else {
            word += ch;
        }
    }
    if (!line.empty()) flush();
}

bool MouseOverUI(const App& a) {
    if (!a.overlay) return false;
    Vector2 m = GetMousePosition();
    return m.x < 330 || m.x > GetScreenWidth() - 200 || m.y > GetScreenHeight() - 150;
}

void DrawOverlay(App& a, bool interactive) {
    const rx::CharacterDebugInfo& d = a.character.GetDebugInfo();
    const int W = GetScreenWidth(), H = GetScreenHeight();

    // Left: injury state per limb
    DrawRectangle(0, 0, 320, H - 150, Fade(Color{16, 18, 24, 255}, 0.82f));
    DrawText("ReflexEngine", 12, 8, 20, RAYWHITE);
    DrawText(TextFormat("%d FPS   shots: %d   characters: %d   last: %s", GetFPS(), a.shots, 1 + static_cast<int>(a.crowd.size()), a.lastShot.c_str()), 12, 32, 12, LIGHTGRAY);
    int y = 52;
    DrawText(TextFormat("control %.2f   consciousness %.2f", d.control, d.consciousness), 12, y, 12, RAYWHITE);
    Bar(12, y + 15, 296, 8, d.control, Color{90, 200, 120, 255});
    y += 30;
    DrawText(TextFormat("shock %.2f   vitality %.2f   fatigue %.2f", d.shock, d.vitality, d.fatigue), 12, y, 12, RAYWHITE);
    y += 18;
    DrawText(TextFormat("leg support  L %.2f  R %.2f    load  L %.2f  R %.2f", d.supportCapacity[0], d.supportCapacity[1],
                        d.legLoad[0], d.legLoad[1]), 12, y, 12, RAYWHITE);
    y += 22;
    DrawText("part              health    capacity   authority", 12, y, 11, GRAY);
    y += 15;
    for (int i = 0; i < rx::kBodyPartCount; ++i) {
        rx::BodyPart p = static_cast<rx::BodyPart>(i);
        DrawText(rx::BodyPartName(p), 12, y, 11, RAYWHITE);
        Bar(118, y + 1, 56, 8, d.parts[p].health, Color{90, 200, 120, 255});
        Bar(184, y + 1, 56, 8, d.parts[p].capacity, Color{240, 200, 80, 255});
        Bar(250, y + 1, 56, 8, d.authority[p], Color{90, 160, 240, 255});
        y += 14;
    }
    y += 6;
    DrawText(TextFormat("tilt %.2f   margin %.2f   feet %s%s%s", d.tilt, d.supportMargin, d.footContact[0] ? "L" : "-",
                        d.footContact[1] ? "R" : "-", d.fallen ? "   FALLEN" : ""), 12, y, 12, RAYWHITE);
    y += 16;
    DrawText("green=support polygon  yellow=COM  blue=capture point", 12, y, 11, GRAY);

    // Right: authority sliders
    int rx0 = W - 190;
    DrawRectangle(rx0 - 10, 0, 200, H - 150, Fade(Color{16, 18, 24, 255}, 0.82f));
    DrawText("Control authority", rx0, 8, 14, RAYWHITE);
    float g = a.character.GetGlobalAuthority();
    if (Slider(&g, "global", rx0, 30, 170, g, 0, 1, interactive)) a.character.SetGlobalAuthority(g);
    float aw = a.character.GetAnimationWeight();
    if (Slider(&aw, "animation weight", rx0, 62, 170, aw, 0, 1, interactive)) {
        a.character.SetAnimationPose(a.animPose);
        a.character.SetAnimationWeight(aw);
    }
    int sy = 98;
    for (int i = 1; i < rx::kBodyPartCount; ++i) {
        rx::BodyPart p = static_cast<rx::BodyPart>(i);
        float v = a.character.GetAuthority(p);
        if (Slider(reinterpret_cast<const void*>(static_cast<size_t>(0x100 + i)), rx::BodyPartName(p), rx0, sy, 170, v, 0, 1, interactive))
            a.character.SetAuthority(p, v);
        sy += 29;
        if (sy > H - 190) break;
    }

    // Bottom: what the engine is thinking
    DrawRectangle(0, H - 150, W, 150, Fade(Color{16, 18, 24, 255}, 0.88f));
    std::string active;
    for (const rx::ActiveBehavior& b : d.activeBehaviors) {
        if (!active.empty()) active += "  +  ";
        active += b.name;
    }
    DrawText(active.empty() ? "behaviour: (idle)" : TextFormat("behaviour: %s", active.c_str()), 12, H - 142, 16, Color{255, 210, 90, 255});
    DrawWrapped(std::string("reason: ") + d.reason, 12, H - 118, W - 24, 15, RAYWHITE);
    if (a.scene == 1) {
        DrawText("FORCE LAB: hold LMB to charge a giant ball, release to fire   |   C crowd   X clear   R reset   M model   1 shooting range", 12, H - 20, 13, GRAY);
        float ch = ChargeFraction(a);
        int bw = 360, bx = W / 2 - bw / 2, by = H - 175;
        if (a.charging || ch > 0) {
            Bar(bx, by, bw, 16, ch, Color{255, 140, 40, 255});
            DrawText(TextFormat("FORCE  %.0f m/s  (launch ~%.0f m/s)", BallSpeed(ch), kBallMass * BallSpeed(ch) / 75.0f), bx, by - 18, 14, ORANGE);
        }
    } else {
        DrawText("LMB shoot   RMB orbit   wheel zoom   R reset   Tab overlay   A animation   C crowd   M model   2 force lab", 12, H - 20, 13, GRAY);
    }
}

void ResetAll(App& a) {
    a.character.Reset();
    for (rx::Character& c : a.crowd) c.Reset();
    a.balls.clear();
    a.character.SetAnimationWeight(0.0f, 0.0f);
    a.markers.clear();
    a.tracers.clear();
    for (float& f : a.hitFlash) f = 0;
    a.shots = 0;
    a.lastShot = "-";
}

// ---- Scripted capture (used to produce the README GIF) ------------------------------------------------------------
struct ScriptStep {
    float time;
    int kind; // 0 right leg, 1 near miss head left, 2 chest, 3 head, 4 left leg
};

void RunScript(App& a, float t, size_t& next, const std::vector<ScriptStep>& script) {
    while (next < script.size() && t >= script[next].time) {
        const rx::Vec3 shooter{1.2f, 1.35f, 7.0f};
        auto part = [&](rx::BodyPart p) { return a.character.GetBodyPartTransform(p).position; };
        switch (script[next].kind) {
            case 0: FireAt(a, shooter, part(rx::BodyPart::RightLowerLeg)); break;
            case 1: {
                rx::Vec3 h = part(rx::BodyPart::Head);
                FireAt(a, shooter, h + rx::Vec3{0.17f, 0.0f, 0}); // grazes the left side of the head
                break;
            }
            case 2: FireAt(a, shooter, part(rx::BodyPart::Chest)); break;
            case 3: FireAt(a, shooter, part(rx::BodyPart::Head)); break;
            case 4: FireAt(a, shooter, part(rx::BodyPart::LeftLowerLeg)); break;
        }
        ++next;
    }
}

} // namespace


int main(int argc, char** argv) {
    std::string captureDir;
    int width = 1280, height = 720;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--capture") && i + 1 < argc) {
            captureDir = argv[++i];
            if (i + 2 < argc && argv[i + 1][0] != '-') {
                width = std::atoi(argv[++i]);
                height = std::atoi(argv[++i]);
            }
        }
    }
    const bool capture = !captureDir.empty();
    bool bench = false;
    for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], "--bench")) bench = true;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(width, height, "ReflexEngine demo - shoot the character");
    if (!bench) SetTargetFPS(240);
    double benchStart = GetTime();
    int benchFrames = 0;

#ifndef NDEBUG
    TraceLog(LOG_WARNING, "Debug build: physics runs far slower. Build with --config Release for the real frame rate.");
#endif
    App app;
    gApp = &app;
    for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], "--model")) app.useModel = true;
    LoadScene(app, 0);

    std::vector<ScriptStep> script = {{1.0f, 0}, {4.5f, 1}, {7.5f, 2}, {10.5f, 4}, {13.0f, 3}};
    size_t scriptNext = 0;
    float scriptTime = 0.0f;
    int frame = 0;
    if (capture) {
        // raylib writes screenshots into the working directory: refuse to run if we cannot switch to the target,
        // so frames are never dumped somewhere unexpected (e.g. a synced project folder).
        if (!ChangeDirectory(captureDir.c_str())) {
            TraceLog(LOG_ERROR, "Capture directory '%s' is not usable, aborting", captureDir.c_str());
            CloseWindow();
            return 1;
        }
        app.camYaw = 0.65f; // look from the shooter's side so the character faces the camera
        app.camPitch = 0.20f;
        app.camDist = 3.9f;
        app.overlay = true;
    }

    while (!WindowShouldClose()) {
        float dt = (capture || bench) ? 1.0f / 60.0f : GetFrameTime();
        // Input
        if (IsKeyPressed(KEY_R)) {
            ResetAll(app);
            scriptNext = 0;
            scriptTime = 0;
        }
        if (IsKeyPressed(KEY_TAB)) app.overlay = !app.overlay;
        if (!capture && !bench) {
            if (IsKeyPressed(KEY_M)) {
                app.useModel = !app.useModel;
                LoadScene(app, app.scene);
            }
            if (IsKeyPressed(KEY_C)) SpawnCrowd(app, 10);
            if (IsKeyPressed(KEY_X)) app.crowd.clear();
            if (IsKeyPressed(KEY_ONE) && app.scene != 0) LoadScene(app, 0);
            if (IsKeyPressed(KEY_TWO) && app.scene != 1) LoadScene(app, 1);
        }
        if (IsKeyPressed(KEY_A)) {
            app.useAnimation = !app.useAnimation;
            app.character.SetAnimationPose(app.animPose);
            if (app.useAnimation) app.character.ReleaseToAnimation(0.4f);
            else app.character.TakeProceduralControl(0.4f);
        }
        if (!capture && !bench) {
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 md = GetMouseDelta();
                app.camYaw -= md.x * 0.006f;
                app.camPitch = Clamp(app.camPitch + md.y * 0.006f, -0.2f, 1.4f);
            }
            app.camDist = Clamp(app.camDist - GetMouseWheelMove() * 0.4f, 1.8f, 20.0f);
            if (app.scene == 0) {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !MouseOverUI(app) && gActiveSlider == nullptr)
                    ShootFromCamera(app, GetMousePosition());
            } else {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !MouseOverUI(app) && gActiveSlider == nullptr) {
                    app.charging = true;
                    app.chargeTime = 0.0f;
                }
                if (app.charging) {
                    app.chargeTime += dt;
                    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                        FireBall(app, GetMousePosition(), ChargeFraction(app));
                        app.charging = false;
                        app.chargeTime = 0.0f;
                    }
                }
            }
        } else {
            scriptTime += dt;
            RunScript(app, scriptTime, scriptNext, script);
        }
        app.camTarget.y = 1.0f;
        {
            rx::Vec3 p = app.character.GetBodyPartTransform(rx::BodyPart::Pelvis).position;
            app.camTarget.x += (p.x - app.camTarget.x) * 0.1f;
            app.camTarget.z += (p.z - app.camTarget.z) * 0.1f;
        }

        // Simulation
        app.world->Step(dt);
        UpdateBalls(app, dt);
        if (app.useModel) {
            PoseSkinned(app.mainSkin, app.character);
            for (size_t i = 0; i < app.crowd.size() && i < app.crowdSkin.size(); ++i) PoseSkinned(*app.crowdSkin[i], app.crowd[i]);
        }
        for (float& f : app.hitFlash) f = std::max(0.0f, f - dt * 2.5f);
        for (auto& m : app.markers) m.life -= dt;
        for (auto& t : app.tracers) t.life -= dt;
        app.markers.erase(std::remove_if(app.markers.begin(), app.markers.end(), [](const Marker& m) { return m.life <= 0; }), app.markers.end());
        app.tracers.erase(std::remove_if(app.tracers.begin(), app.tracers.end(), [](const Tracer& t) { return t.life <= 0; }), app.tracers.end());

        // Render
        UpdateCamera(app);
        BeginDrawing();
        ClearBackground(Color{150, 190, 230, 255});
        BeginMode3D(app.camera);
        DrawScene(app);
        EndMode3D();
        if (app.overlay) DrawOverlay(app, !capture && !bench);
        else DrawText("Tab: show debug overlay", 12, 10, 14, DARKGRAY);
        EndDrawing();

        if (bench && ++benchFrames == 600) {
            double sec = GetTime() - benchStart;
            std::printf("%d frames in %.2f s = %.1f FPS (%.2f ms/frame)\n", benchFrames, sec, benchFrames / sec, 1000.0 * sec / benchFrames);
            break;
        }
        if (capture) {
            if (frame % 3 == 0) {
                char name[64];
                std::snprintf(name, sizeof name, "frame_%04d.png", frame / 3);
                TakeScreenshot(name);
            }
            ++frame;
            if (scriptTime > 17.0f) break;
        }
    }
    FreeSkinned(app.mainSkin);
    for (auto& k : app.crowdSkin) FreeSkinned(*k);
    CloseWindow();
    return 0;
}
