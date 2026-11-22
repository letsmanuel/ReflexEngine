#include "reflex/types.h"

namespace reflex {

const char* BodyPartName(BodyPart p) {
    static const char* names[] = {"Pelvis",        "Spine",         "Chest",          "Neck",          "Head",
                                  "LeftUpperArm",  "LeftLowerArm",  "LeftHand",       "RightUpperArm", "RightLowerArm",
                                  "RightHand",     "LeftUpperLeg",  "LeftLowerLeg",   "LeftFoot",      "RightUpperLeg",
                                  "RightLowerLeg", "RightFoot"};
    int i = static_cast<int>(p);
    return i >= 0 && i < kBodyPartCount ? names[i] : "?";
}

} // namespace reflex
