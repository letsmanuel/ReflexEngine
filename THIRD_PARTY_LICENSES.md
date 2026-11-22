# Third-party licenses

ReflexEngine itself is MIT licensed (see `LICENSE`). Dependencies are fetched by CMake at configure time; none is copied into this repository.

| Dependency | Version | License | Source | Used for |
|---|---|---|---|---|
| Jolt Physics | v5.4.0 | MIT | https://github.com/jrouwe/JoltPhysics | default physics backend |
| doctest | v2.4.11 | MIT | https://github.com/doctest/doctest | tests only |
| raylib | 5.5 | zlib | https://github.com/raysan5/raylib | demo window, input, drawing (demo only) |

## Assets

The demo currently draws the character procedurally from its physics bodies (capsules, boxes, a sphere), so no third-party art assets are shipped. If a CC0 character model is added it will be listed here with its source URL and license.

## Clean-room statement

No proprietary or leaked code (including Rockstar's RAGE or NaturalMotion's Euphoria) was consulted. The systems use public techniques: linear inverted pendulum / capture-point stepping, two-bone IK, PD joint motors, virtual model control and utility-based decision making.
