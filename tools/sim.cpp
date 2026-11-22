// Debug tool: runs a headless scenario and prints a time series.
//   reflex_sim stand|legright|legleft|headleft|torso|armr|fatal|push|walk [seconds]
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include "scenario.h"

using namespace reflex;
using namespace reflex::test;

static void Print(Scenario& s, float t) {
    const CharacterDebugInfo& d = s.Info();
    Vec3 p = s.Pelvis();
    Quat qc = s.c.GetBodyPartTransform(BodyPart::Chest).rotation, qp = s.c.GetBodyPartTransform(BodyPart::Pelvis).rotation;
    float relAng = Length((qp.Conjugate() * qc).ToRotationVector());
    Transform lf = s.c.GetBodyPartTransform(BodyPart::LeftFoot), rf = s.c.GetBodyPartTransform(BodyPart::RightFoot);
    std::printf("t=%5.2f pel=(%6.3f %5.3f %6.3f) tilt=%4.2f margin=%6.3f com=(%6.3f %5.3f %6.3f) feet L(%5.2f %5.2f %5.2f) R(%5.2f %5.2f %5.2f) "
                "chestRel=%4.2f ctl=%4.2f step=%d%d load=%4.2f/%4.2f %s\n",
                t, p.x, p.y, p.z, d.tilt, d.supportMargin, d.com.x, d.com.y, d.com.z, lf.position.x, lf.position.y,
                lf.position.z, rf.position.x, rf.position.y, rf.position.z, relAng, d.control, d.stepping[0], d.stepping[1],
                d.legLoad[0], d.legLoad[1], d.fallen ? "FALLEN" : "");
}

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "stand";
    float seconds = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 4.0f;
    Vec3 hitDir = argc > 4 ? Normalize(Vec3{static_cast<float>(std::atof(argv[3])), 0, static_cast<float>(std::atof(argv[4]))}) : Vec3{0, 0, -1};
    Scenario s;
    s.Run(1.0f); // settle
    if (mode == "bench") {
        auto t0 = std::chrono::steady_clock::now();
        const int n = 1200;
        for (int i = 0; i < n; ++i) s.world->Step(s.dt);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("%.3f ms per 1/120 s tick -> %.2f ms of engine+physics per 60 fps frame\n", ms / n, 2 * ms / n);
        return 0;
    }
    if (mode != "stand") std::printf("-- settled, pelvis y=%.3f\n", s.Pelvis().y);

    // The character faces +Z, so the shooter stands at +Z and fires towards -Z.
    auto hit = [&](BodyPart part, float dmg) {
        Vec3 pt = s.c.GetBodyPartTransform(part).position;
        HitEvent h;
        h.bodyPart = part;
        h.point = pt - hitDir * 0.1f;
        h.direction = hitDir;
        h.speed = 400;
        h.mass = 0.008f;
        h.impulse = 3.2f;
        h.damage = dmg;
        s.c.ApplyHit(h);
    };
    if (mode == "legright") hit(BodyPart::RightLowerLeg, 35);
    if (mode == "legleft") hit(BodyPart::LeftLowerLeg, 35);
    if (mode == "torso") hit(BodyPart::Chest, 45);
    if (mode == "armr") hit(BodyPart::RightLowerArm, 40);
    if (mode == "fatal") hit(BodyPart::Chest, 400);
    if (mode == "headleft") {
        NearMissEvent m;
        Vec3 hd = s.Head();
        m.closestPoint = hd + Vec3{0.2f, 0, 0.3f};
        m.direction = {0, 0, -1};
        m.distance = 0.15f;
        m.nearestPart = BodyPart::Head;
        s.c.ReportNearMiss(m);
    }

    float t = 0;
    const int every = 30;
    int steps = static_cast<int>(seconds / s.dt);
    for (int i = 0; i < steps; ++i) {
        s.world->Step(s.dt);
        t += s.dt;
        if (i % every == 0) Print(s, t);
    }
    std::printf("reason: %s\n", s.Info().reason.c_str());
    return 0;
}
