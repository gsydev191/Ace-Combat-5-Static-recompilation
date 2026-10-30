# AC5 Native Port

This project is licensed under Apache-2.0; see `LICENSE`.

This directory is an independent C++20/Vulkan starting point for a future
standalone repository. It does not include, link, or compile code from the
PS2 recompiler/runtime project. The renderer build requires SDL3, Vulkan 1.3,
and `glslc` (normally provided by the Vulkan SDK). The archive and asset
libraries can be built without those renderer dependencies.

The current executable opens a window, creates a Vulkan 1.3 device and
swapchain, reads an OBJ mesh, uploads indexed geometry, and draws it through a
shader-backed pipeline using dynamic rendering and synchronization2. Shaders
compile to SPIR-V during the CMake build. The renderer uses per-image depth
attachments, depth testing, indexed UV geometry, OBJ vertex normals with flat
normal generation when absent, per-draw inverse-transpose normal matrices,
diffuse lighting, and sRGB textures with GPU generated mipmaps where linear
blits are supported. When available, the renderer enables Vulkan portability
enumeration and the device portability subset for layered Vulkan platforms.
Textures with non-opaque alpha use a blending pipeline and are sorted by the
transformed mesh bounds center; opaque meshes keep depth writes enabled.
Scene replacement uploads are staged in separate Vulkan buffers, textures,
and descriptors; a failed upload leaves the previously uploaded scene usable.
OBJ/MTL and PPM/TGA importers,
platform-independent camera/scene/input types, native SDL input, and a DATA.PAC/ULZ
reader live entirely in this directory. The project does not yet decode the
game's packed model, texture, or animation
assets or contain game-accurate gameplay systems.

The native math uses column-major, right-handed matrices. Its perspective
projection uses Vulkan's zero-to-one depth range and inverts Y for a
positive-height viewport; `Camera` uses those same conventions for view,
projection, and view-projection matrices.

`UlzDecoder` exposes `begin` and `step` for callers that need to spread
decompression over multiple frames. Each `step` accepts a control-word budget;
the convenience `decode_ulz` function uses the generated routine's 2,500-word
budget repeatedly until the member is complete. The decoder keeps a non-owning
view of the packed bytes, so they must remain alive until decoding finishes.
Malformed input puts the decoder in a terminal failed state until the next
`begin`, preventing callers from resuming a partially consumed corrupt stream.
`DataArchive::begin_read_member` provides the same incremental interface for
DATA.PAC members through an `ArchiveMemberLoad`, which owns its packed input and
checks the decoded length against DATA.TBL when complete. Incremental callers
can inspect partial output, distinguish active/finished/failed loads, or transfer
the completed buffer with `take_output`.
Archive file reading is synchronous; the step budget applies to decompression.

## Build

The supplied CMake presets provide a standalone renderer build and a core-only
build. Both use Ninja and place generated files under `build/`:

```text
cmake --preset renderer
cmake --build --preset renderer
```

The renderer preset requires SDL3, Vulkan 1.3, and `glslc`. To build the native
archive and asset libraries without those dependencies, use:

```text
cmake --preset core
cmake --build --preset core
```

Equivalent direct configuration commands are available for environments that
use a different CMake generator:

```text
cmake -S . -B build
cmake --build build --target ac5_native
```

Run it with `--model path/to/model.obj` to load another OBJ file and optionally
`--texture path/to/image.ppm` or `--texture path/to/image.tga` to apply a
texture to the scene, overriding OBJ material maps. The importer accepts standard homogeneous
`v x y z w` positions, `vt` texture
coordinates, and the common optional RGB extension on `v` records (`x y z r g b`
or `x y z w r g b`). Faces retain shared position/UV/normal tuples as indexed
geometry. The OBJ loader preserves explicit normals, generates flat normals
when smoothing is off, averages generated normals for OBJ smoothing groups,
and ear-clips concave faces. It supports material sections and
`Kd` diffuse colors plus simple `map_Kd` references to P3/P6 PPM or true-color
TGA images. TGA loading supports uncompressed and RLE 24-bit or 32-bit images,
normalizes all four image origins, and preserves 32-bit alpha. Color-mapped and
interleaved TGA files are rejected. MTL
diffuse colors multiply OBJ vertex colors, and remain active when the command
line texture override is used. PPM `maxval` values from 1 through 65535 are
scaled to 8-bit textures, including 16-bit big-endian P6 samples. The scene API
supports multiple texture IDs and lets each draw select one. Each draw also
has a local model matrix and a shared scene transform ID. Material sections of
one imported model can therefore follow one entity transform while retaining
independent mesh and texture resources. `load_obj_scene` appends its model to
the destination scene transactionally, so a failed import leaves existing
scene contents intact and successful imports can build a multi-entity scene.
It stages only the new model assets, avoiding a copy of existing scene geometry.
Transforms can form a parent hierarchy by referencing an earlier transform;
the ordering rule prevents cycles, and the renderer resolves the world matrices
once per frame. Transform changes do not invalidate uploaded geometry or
textures.

The `ac5_extract`, `ac5_asset_core`, and `ac5_game_core` targets are independent
of SDL3 and Vulkan. Configure with `-DAC5_BUILD_RENDERER=OFF` to build the
native archive tools and reusable asset/scene libraries without the renderer:

```text
cmake -S . -B build-core -DAC5_BUILD_RENDERER=OFF
cmake --build build-core --target ac5_extract
```

With the renderer enabled, `ac5_vulkan_renderer` is a separate static library
for reuse by the game layer, and `ac5_platform` provides the SDL3 gamepad
adapter as a separate reusable target. The sample application links these with
OBJ scene loading. The renderer package exports `ac5::vulkan_renderer` and
`ac5::platform`; both dependencies are discovered by
`find_package(ac5_native_renderer CONFIG REQUIRED)`.

The core libraries, public headers, extractor, renderer library, and sample
application can be installed under a standalone prefix:

```text
cmake --install build --prefix install
```

Headers are installed under `include/ac5_native`, libraries under `lib` (or
the platform's CMake library directory), and executables under `bin`. The
renderer install also places its compiled shaders and demo assets under
`bin/shaders` and `bin/assets`, where the sample application looks for them.
SDL3 and Vulkan remain external renderer dependencies for downstream builds.
The dependency-free libraries have a CMake package that can be consumed with
`find_package(ac5_native CONFIG REQUIRED)` and targets `ac5::asset_core` and
`ac5::game_core`. Set `CMAKE_PREFIX_PATH` to the install prefix. For example:

```cmake
find_package(ac5_native CONFIG REQUIRED)
target_link_libraries(my_tool PRIVATE ac5::asset_core ac5::game_core)
```

The renderer has a separate `ac5_native_renderer` package with target
`ac5::vulkan_renderer`. Its config discovers Vulkan 1.3 and SDL3 for the
consumer; the core package does not discover or link either dependency. Use
`find_package(ac5_native_renderer CONFIG REQUIRED)` when linking the renderer.
All exported library targets propagate the C++20 requirement to consumers.
The public C++ API is in the `ac5` namespace (for example,
`ac5::DataArchive`, `ac5::Scene`, `ac5::SceneStateMachine`,
`ac5::ArchiveLoadPlan`, `ac5::ArchiveBatchLoad`, and `ac5::VulkanRenderer`).
`SceneDispatcher<State, Substate, Context>` maps typed
state/substate pairs to handlers. `SceneStateMachine<State, Substate>` keeps
requested state changes pending until `commit_transition()`, then resets the
substate and returns the old/new states for caller-managed lifecycle work.
`cancel_transition()` clears a queued transition before it is committed.
`advance_selection` provides the wrapping list-index behavior recovered from
one generated UI caller; native UI code supplies abstract forward/backward
edges and does not depend on guest action bits.
`ArchiveLoadPlan` accepts up to seven in-range member requests, retains their
order and duplicates, and provides a native boundary for later loading code.
`ArchiveBatchLoad` executes that plan incrementally, applying a per-step ULZ
control-word budget and returning completed member buffers in request order.
Archive file reads remain synchronous at each member's start. Plans are bound
to the archive generation they were created from, and a batch fails if the
archive is reopened while it is running.

The extractor can write a decoded member or one inner file by index:

```text
ac5_extract DATA.TBL DATA.PAC 251 251.bin
ac5_extract DATA.TBL DATA.PAC 251 0 aircraft.obj
ac5_extract --named DATA.TBL DATA.PAC pac_names.txt 251 plmodel/f04eo.mpl aircraft.mpl
ac5_extract --all DATA.TBL DATA.PAC pac_names.txt extracted
```

The extracted bytes remain in their original game format; the OBJ renderer
path currently consumes converted OBJ meshes. Name lookup accepts the
`member index path` text format and compares paths without case sensitivity.
Bulk extraction writes one directory per member and uses `__ac5_slots/index`
paths when no safe archive path is known or when multiple slots collide by
name, including case-insensitive and file-versus-directory collisions. It
normalizes slash styles, rejects unsafe and nonportable names, and refuses
output paths that resolve through symlinks outside the selected output
directory.

Arrow keys/WASD pitch and roll, Q/E yaw, R/F set throttle input, Space is
primary fire, and either Ctrl key is secondary fire. Fire controls expose held,
pressed, and released states to native game systems. Demo pitch, roll, and yaw
accumulate at a frame-rate-independent rate; throttle and fire are exposed to
native game systems but are not yet consumed by the demo. The left gamepad stick
controls pitch/roll, the right stick controls yaw, triggers provide throttle,
and South/East provide the two fire actions. Pitch, roll, and yaw currently
rotate the demo model's shared scene transform; throttle and fire are
available in `FlightInput` for the flight model that has not been ported yet.
The input system opens one connected gamepad at startup and scans the remaining
connected devices when the active gamepad is removed. It also handles devices
connected after startup.

## Porting rule

Treat `generated/ps2_code_*.c` as behavioral evidence, not as native game
source. Those files encode MIPS register state, PS2 address-space reads and
writes, delay slots, and hardware calls through `ps2_runtime.h`. Port a
subsystem by identifying its game-level inputs and outputs, then implementing
that behavior against native data types and services here. Keep PS2 emulation
out of this directory.
