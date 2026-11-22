#pragma once
/// @file jolt_backend.h
/// Default physics backend built on Jolt Physics (MIT).

#include <memory>

#include "reflex/export.h"
#include "reflex/physics_backend.h"

namespace reflex {

struct JoltBackendOptions {
    unsigned maxBodies = 2048;
    unsigned maxBodyPairs = 8192;
    unsigned maxContactConstraints = 8192;
    int collisionSteps = 1;   ///< Jolt collision sub-steps per Step() call
    int velocitySteps = 10;   ///< constraint solver velocity iterations (more = stiffer ragdoll)
    int positionSteps = 4;
};

/// Creates the Jolt-based backend (single threaded and deterministic).
REFLEX_API std::unique_ptr<IPhysicsBackend> CreateJoltBackend(const JoltBackendOptions& options = {});

} // namespace reflex
