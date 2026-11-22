#include <doctest/doctest.h>

#include "reflex/authority.h"

using namespace reflex;

TEST_CASE("Authority: defaults are fully procedural and fully powered") {
    AuthorityBlender a;
    CHECK(a.Global() == doctest::Approx(1.0f));
    CHECK(a.Part(BodyPart::Head) == doctest::Approx(1.0f));
    CHECK(a.AnimationWeight() == doctest::Approx(0.0f));
}

TEST_CASE("Authority: instant set when blend time is zero") {
    AuthorityBlender a;
    a.SetGlobal(0.25f, 0.0f);
    CHECK(a.Global() == doctest::Approx(0.25f));
}

TEST_CASE("Authority: blend reaches the target after the blend time, monotonic in between") {
    AuthorityBlender a;
    a.SetGlobal(0.0f, 0.5f);
    float prev = a.Global();
    for (int i = 0; i < 30; ++i) {
        a.Update(0.01f);
        CHECK(a.Global() <= prev + 1e-6f);
        prev = a.Global();
    }
    CHECK(a.Global() > 0.0f);
    CHECK(a.Global() < 1.0f);
    for (int i = 0; i < 30; ++i) a.Update(0.01f);
    CHECK(a.Global() == doctest::Approx(0.0f));
}

TEST_CASE("Authority: effective authority multiplies game settings with injury state") {
    AuthorityBlender a;
    a.SetGlobal(0.8f);
    a.SetPart(BodyPart::RightUpperArm, 0.5f);
    CHECK(a.Effective(BodyPart::RightUpperArm, 1.0f, 1.0f) == doctest::Approx(0.4f));
    CHECK(a.Effective(BodyPart::RightUpperArm, 0.5f, 0.5f) == doctest::Approx(0.1f));
    CHECK(a.Effective(BodyPart::Head, 1.0f, 1.0f) == doctest::Approx(0.8f));
}

TEST_CASE("Authority: hand control back to animation smoothly") {
    AuthorityBlender a;
    a.ReleaseToAnimation(1.0f);
    a.Update(0.5f);
    CHECK(a.AnimationWeight() > 0.3f);
    CHECK(a.AnimationWeight() < 0.7f);
    a.Update(0.6f);
    CHECK(a.AnimationWeight() == doctest::Approx(1.0f));
    a.TakeProceduralControl(0.0f);
    CHECK(a.AnimationWeight() == doctest::Approx(0.0f));
}

TEST_CASE("Authority: passive ragdoll when global authority is zero") {
    AuthorityBlender a;
    a.SetGlobal(0.0f);
    for (int i = 0; i < kBodyPartCount; ++i)
        CHECK(a.Effective(static_cast<BodyPart>(i), 1.0f, 1.0f) == doctest::Approx(0.0f));
}
