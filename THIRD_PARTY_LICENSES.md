# Third-party licenses

ReflexEngine itself is MIT licensed (see `LICENSE`). Dependencies are fetched by CMake at configure time; none is copied into this repository.

| Dependency | Version | License | Source | Used for |
|---|---|---|---|---|
| Jolt Physics | v5.4.0 | MIT | https://github.com/jrouwe/JoltPhysics | default physics backend |
| doctest | v2.4.11 | MIT | https://github.com/doctest/doctest | tests only |
| raylib | 5.5 | zlib | https://github.com/raysan5/raylib | demo window, input, drawing (demo only) |

## Assets

| Asset | License | Source | Used for |
|---|---|---|---|
| CesiumMan (`demo/assets/CesiumMan.glb`) | CC BY 4.0, (c) 2017 Cesium | https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CesiumMan | optional skinned character in the demo (press `M`) |

CesiumMan is redistributed unmodified under CC BY 4.0 (https://creativecommons.org/licenses/by/4.0/). The demo can also draw a primitive figure from the physics bodies, which needs no asset.

## Clean-room statement

No proprietary or leaked code (including Rockstar's RAGE or NaturalMotion's Euphoria) was consulted. The systems use public techniques: linear inverted pendulum / capture-point stepping, two-bone IK, PD joint motors, virtual model control and utility-based decision making.
