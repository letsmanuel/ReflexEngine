#include <doctest/doctest.h>

#include "reflex/injury.h"

using namespace reflex;

TEST_CASE("Injury: fresh body is fully capable") {
    InjuryModel m;
    CHECK(m.SupportCapacity(Side::Left) == doctest::Approx(1.0f));
    CHECK(m.SupportCapacity(Side::Right) == doctest::Approx(1.0f));
    CHECK(m.ControlLevel() == doctest::Approx(1.0f));
    CHECK_FALSE(m.IsDead());
    CHECK_FALSE(m.IsCritical());
}

TEST_CASE("Injury: a leg hit sharply lowers that leg's support capacity only") {
    InjuryModel m;
    m.ApplyDamage(BodyPart::RightLowerLeg, 35.0f);
    CHECK(m.SupportCapacity(Side::Right) < 0.35f);
    CHECK(m.SupportCapacity(Side::Right) > 0.05f);
    CHECK(m.SupportCapacity(Side::Left) > 0.95f);
    CHECK(m.WeakerLeg() == Side::Right);
    CHECK(m.LastHitPart() == BodyPart::RightLowerLeg);
    CHECK_FALSE(m.IsCritical());
    CHECK(m.ControlLevel() > 0.5f);
}

TEST_CASE("Injury: pain fades so the leg partially recovers, never beyond its health") {
    InjuryModel m;
    m.ApplyDamage(BodyPart::LeftUpperLeg, 35.0f);
    float early = m.SupportCapacity(Side::Left);
    for (int i = 0; i < 60 * 30; ++i) m.Update(1.0f / 60.0f);
    float late = m.SupportCapacity(Side::Left);
    CHECK(late > early);
    CHECK(late < 0.65f);
}

TEST_CASE("Injury: bleeding makes a bad wound worse over time") {
    InjuryModel m;
    m.ApplyDamage(BodyPart::RightUpperLeg, 60.0f);
    float h0 = m.Part(BodyPart::RightUpperLeg).health;
    for (int i = 0; i < 60 * 10; ++i) m.Update(1.0f / 60.0f);
    CHECK(m.Part(BodyPart::RightUpperLeg).health < h0);
}

TEST_CASE("Injury: arm hit weakens only that arm") {
    InjuryModel m;
    m.ApplyDamage(BodyPart::LeftLowerArm, 40.0f);
    CHECK(m.ArmCapacity(Side::Left) < 0.35f);
    CHECK(m.ArmCapacity(Side::Right) > 0.95f);
    CHECK(m.WeakerArm() == Side::Left);
    CHECK(m.SupportCapacity(Side::Left) > 0.95f);
}

TEST_CASE("Injury: heavy torso damage reduces control gradually, then collapse state") {
    InjuryModel m;
    m.ApplyDamage(BodyPart::Chest, 90.0f);
    float c0 = m.ControlLevel();
    for (int i = 0; i < 60 * 3; ++i) m.Update(1.0f / 60.0f);
    CHECK(m.ControlLevel() < c0 + 1e-3f);
    CHECK(m.IsCritical());
}

TEST_CASE("Injury: fatal damage kills and consciousness fades gradually to zero") {
    InjuryModel m;
    m.ApplyDamage(BodyPart::Head, 120.0f);
    CHECK(m.IsDead());
    m.Update(0.1f);
    CHECK(m.Consciousness() > 0.0f); // not instant
    for (int i = 0; i < 60 * 4; ++i) m.Update(1.0f / 60.0f);
    CHECK(m.Consciousness() == doctest::Approx(0.0f));
    CHECK(m.ControlLevel() == doctest::Approx(0.0f));
}

TEST_CASE("Injury: reset restores a healthy body") {
    InjuryModel m;
    m.ApplyDamage(BodyPart::Chest, 200.0f);
    m.Reset();
    CHECK(m.Vitality() == doctest::Approx(1.0f));
    CHECK(m.ControlLevel() == doctest::Approx(1.0f));
}
