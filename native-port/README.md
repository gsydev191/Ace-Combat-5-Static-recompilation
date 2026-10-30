# AC5 Native Port

This project is licensed under Apache-2.0; see `LICENSE`.

This directory is an independent C++20/Vulkan starting point for a future
standalone repository. It does not include, link, or compile code from the
PS2 recompiler/runtime project. The renderer build requires SDL3, Vulkan 1.3,
and `glslc` (normally provided by the Vulkan SDK). The archive and asset
libraries can be built without those renderer dependencies.

The current executable opens a window, creates a Vulkan 1.3 device and
swapchain, reads an OBJ mesh, uploads indexed geometry, and draws it through a
shader-backed pipeline using dynamic rendering and synchronization2. It
requires an sRGB nonlinear swapchain format so fixed-function transparency
blending operates on linear-light attachment values. Shaders
compile to SPIR-V during the CMake build. The renderer uses per-image depth
attachments, depth testing, indexed UV geometry, OBJ vertex normals with flat
normal generation when absent, per-draw inverse-transpose normal matrices,
diffuse lighting, and sRGB textures with GPU generated mipmaps where linear
blits are supported. When available, the renderer enables Vulkan portability
enumeration and the device portability subset for layered Vulkan platforms.
Native vertices and textures carry alpha into the shader; transparent meshes
are sorted by transformed bounds center and blended back-to-front with depth
writes disabled, while opaque meshes retain depth writes.
Draws can also select an environment texture for a depth-tested blended second
pass. Set it with `Scene::set_environment_texture`; vertices provide reflection
UVs, modulation color, and alpha in their environment fields. Convert game STQ
coordinates to `u = s / q`, `v = t / q` before assigning `environment_uv`; the
shader then perspective-interpolates those UVs across the triangle. The sample
does not yet populate these fields from decoded AC5 materials.
Scene replacement uploads are staged in separate Vulkan buffers, textures,
and descriptors; a failed upload leaves the previously uploaded scene usable.
OBJ/MTL, PPM/TGA/GIM importers, a bounds-checked ACM/VIF aircraft mesh loader,
platform-independent camera/scene/input types, native SDL input, and a
DATA.PAC/ULZ reader live entirely in this directory. The loader currently
supports static bundles with unambiguous part ownership and the first bundled
aircraft texture. Unowned shape-variant chains are skipped for the neutral
static pose. Nested aircraft LOD bundles include all five meshes and textures;
aircraft animation and game-accurate gameplay systems are still in progress.

The portable `model_shading` library exposes the plain, lit, and environment
vertex-program paths as value-in/value-out math. Environment mode returns the
lit base-pass vertex result and separate reflective UV/color data for a second
pass; the native scene and Vulkan renderer can draw that pass when supplied
with environment vertex data and a texture. The sample does not yet populate
those fields from decoded game material state. `model_mesh_adapter.hpp` converts
decoded model vertices through that shading routine into native vertices,
applies already-lit base colors without a second demo-lighting pass, and
dehomogenizes environment STQ while normalizing its 0–255 modulation channels.

The native math uses column-major, right-handed matrices. Its perspective
projection uses Vulkan's zero-to-one depth range and inverts Y for a
positive-height viewport; `Camera` uses those same conventions for view,
projection, and view-projection matrices.
The viewer derives its initial camera target, distance, and clipping planes
from transformed scene geometry, so imported models of different scales are
framed without a model-specific fixed zoom. Flight mode keeps that fitted
follow offset in aircraft-local space, rotating it with yaw, pitch, and roll
while tracking the aircraft root. Mouse-wheel zoom scales the local offset.

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

Run it with `--model path/to/model.obj` to load another OBJ file, or pass a
standalone `ACM` block, `.mpl` aircraft bundle, or `.bin` ACM bundle to load
its first supported ACM mesh. Nested `lodplane/*.bin` bundles load the first
LOD mesh and its companion ACM geometry. `ac5_model_info <model-file>` reports
imported mesh counts and assembled bounds without opening the renderer; for
nested LOD bundles it also lists all five level thresholds, mesh sizes, and
texture dimensions. The viewer uploads all five levels and switches meshes
and textures by camera distance using the recovered retail thresholds; use the
mouse wheel to move through the LOD ranges. `ac5_model_info --parts <model-file>`
reports the retained ACM part tree and each part's local mesh.
It also prints all eight source part-table words for every record, including
records without geometry, plus each part's local position and Euler rotation.
For placed-model bundles, `--pm-textures <pmtex-file>`
decodes the shared palette and lists full-resolution texture slots. The viewer
accepts `--material-textures <pmtex-file>` with an ACM model and binds each
geometry part's source word 3 to the matching texture slot. Current placed-
model loader evidence supports this mapping, but it has not been verified for
every model family.
`ac5_model_info --pm-materials <model-file> <pmtex-file>` lists geometry-part
material slot usage and flags slots without a matching texture.
`ac5_model_info --textures <model-file>` lists every supported indexed GIM
reachable through bundle slots and prints each nested slot path and dimensions.
To run the native flight model on an imported aircraft, provide its matching
400-byte flight rules bundle with `--flight-rules`. For F-4G, extract the
model and flight bundle from the supplied DATA.PAC first:

```text
ac5_extract BIN/DATA.TBL BIN/DATA.PAC 429 0 lodplane/f04gor.bin
ac5_extract BIN/DATA.TBL BIN/DATA.PAC 429 1 mis/mpl/f04g.pmb
ac5_native --model lodplane/f04gor.bin --flight-rules mis/mpl/f04g.pmb
```

To render an image as a heightfield, pass a supported PPM, TGA, or GIM image;
the red channel maps from `-height-scale/2` to `+height-scale/2` and adjacent
samples use the requested world spacing:

```text
ac5_native --heightmap terrain.ppm --height-scale 120 --height-spacing 8
```

The heightfield path accepts an extracted AC5 `.mha`/`.mhd` pair. `.mha`
selects the height blocks in the `.mhd` pool; the viewer accepts either
filename and finds its companion beside it. It assembles the overlapping
33-by-33 blocks into the generated 513-by-513 grid, applies the recovered
80-unit height step, and uses the mission's 1,024-unit spacing and centered
origin. For example, `ac5_native --heightmap m03a.mhd` loads `m03a.mha` too.
The preview is grayscale; mission terrain mode below also loads tile and
vertex-color layers.

To inspect an extracted mission's indexed map texture data, pass its matching
`.mti`, `.mtd`, and `.mtc` files:

```text
ac5_native --map-textures m03a.mti m03a.mtd m03a.mtc
```

The standalone preview decodes 64×64 indexed tiles through the 256-entry GS
palette, packs them into a guttered atlas, and draws the 64-column MTI entry
grid. The preview uses ten viewer units per cell because the map files do not
establish their alignment or scale relative to the MHD heightfield.

For a decompressed mission member in `DATA.PAC`, the viewer can decode and
render its native terrain layers directly. Pass the matching archive table,
archive, and the mission member index:

```text
ac5_native --mission-terrain BIN/DATA.TBL BIN/DATA.PAC 3
```

The mode assembles the mission height blocks, tile-index map, and atlas, then
renders them at their recovered world spacing. Member indexes refer to entries
in the supplied DATA.PAC table. M01A's terrain is member 3 and its mission
script pack is member 117. The native tools can extract an indexed member with
`ac5_extract` for inspection.

Mission terrain can also be combined with a native aircraft flight viewer.
Provide the matching ACM model, `.pmb` flight rules, and aircraft mission-entry
member so the six body extents are decoded from its AHM payload:

```text
ac5_native --mission-terrain BIN/DATA.TBL BIN/DATA.PAC 3 --model lodplane/f04gor.bin --flight-rules mis/mpl/f04g.pmb --aircraft-entry-index 252
```

`--aircraft-start x y z` overrides the default flight position `(0, 3000, 0)`;
coordinates use the same mission world units as the decoded terrain. For
example, start at the M01A origin with `--aircraft-start 0 0 0`. Add
`--aircraft-gear-down` to start with the landing gear deployed; the title shows
`MARKER TRACE` while the ACM I/K/J ground trace is being evaluated.

Flight advances at 60 Hz. When the aircraft root falls below the sampled
terrain plus the recovered 20-unit floor, the viewer checks the six body
extremes. A hit while airborne stops and freezes the flight session on that
tick; when already grounded, the aircraft is clamped back to the floor and the
ground-speed law applies. When landing gear is fully down and the ACM contains
I/K/J markers, the viewer samples them at the generated gear channels and runs
the native terrain trace/alignment before the airframe-floor collision pass.
Models without all three markers, or whose marker poses do not move with the
gear channels, retain root-height contact sampling. Terrain and spawn markers
keep fixed world transforms while the aircraft and follow camera move.

Add `--mission-units` followed by the mission pack's DATA.PAC member index to
overlay side-colored markers for decoded type-0 spawn orders. For M01A, use:

```text
ac5_native --mission-terrain BIN/DATA.TBL BIN/DATA.PAC 3 --mission-units 117
```

Absolute placements, fixed far-corner placements, and terrain-relative
altitudes are resolved from the mission data. Group-relative placements use
the group's resolved initial spawn transform when available. References to an
individual member are shown only when its decoded formation offset is zero;
other placements are counted as unresolved because this preview does not run
the mission unit simulation.

In this mode SDL pitch, roll, and yaw controls plus throttle drive the fixed
60 Hz native flight tick, and the camera follows the simulated aircraft. Press
P to toggle the recovered approach-control law; G toggles viewer gear state.
To apply the aircraft's companion animation config, pass its 80-byte, 20-float
subfile-6 payload with `--animation-rules aircraft-animation.bin`. This enables
the per-aircraft control-authority and speed-driven flap, sweep, and secondary
surface curves. The flight viewer keeps each ACM part's local transform and
parent link as separate scene draws, and evaluates translation and
Euler-rotation tracks from the recovered direct stick and throttle channels.
Shape tracks interpolate compatible alternate chains continuously through
per-frame dynamic vertex buffers; meshes with different topology use a
nearest-key fallback. Gear and flap channels ease toward their targets over
time. The ordinary asset viewer remains the default when `--flight-rules` is
omitted.
The native ACM loader decodes the observed VIF position, normal, color and UV
streams, triangle strips, chain-to-part mapping, and static part hierarchy.
It rejects duplicate or unresolved geometry-chain ownership, and loads the
first supported GIM from the bundle for the aircraft texture unless
placed-model material textures are explicitly supplied. ACM
translation, rotation, and compatible shape animation are supported in flight
mode. The stateful model-flag `0x4` channel-20 accumulator is advanced at the
native 60 Hz animation tick; its retail auxiliary matrix effect and other
controller state paths remain open. You can
override OBJ material maps and bundled
ACM textures with any of `--texture path/to/image.ppm`,
`--texture path/to/image.tga`, or `--texture path/to/image.gim`. The native GIM
decoder handles indexed 4bpp/8bpp retail variants
and their 8bpp CSM1 palette order. Aircraft bundle palette alpha is treated as
opaque because it does not encode material opacity. The importer accepts standard homogeneous
`v x y z w` positions, `vt` texture
coordinates, and the common optional RGB extension on `v` records (`x y z r g b`
or `x y z w r g b`). Faces retain shared position/UV/normal tuples as indexed
geometry. The OBJ loader preserves explicit normals, generates flat normals
when smoothing is off, averages generated normals for OBJ smoothing groups,
and ear-clips concave faces. It supports material sections, `Kd` diffuse colors,
scalar `d` dissolve and `Tr` transparency opacity, plus simple `map_Kd` references to P3/P6 PPM or true-color
TGA images. TGA loading supports uncompressed and RLE 24-bit or 32-bit images,
normalizes all four image origins, and preserves 32-bit alpha. Color-mapped and
interleaved TGA files are rejected. MTL
diffuse colors multiply OBJ vertex colors, while MTL opacity multiplies vertex
alpha; both remain active when the command-line texture override is used. PPM `maxval` values from 1 through 65535 are
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
textures. `Scene::clear_assets()` releases meshes, draws, and imported textures
while preserving transforms, and invalidates the renderer's uploaded asset
revision so the replacement state must upload its assets before drawing.

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

The renderer has a separate `ac5_native_renderer` package with targets
`ac5::vulkan_renderer` and `ac5::platform`. Its config loads the core package
and discovers Vulkan 1.3 and SDL3 for the consumer; the core package does not
discover or link either renderer dependency. Use
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
ac5_extract --named DATA.TBL DATA.PAC pac_names.txt 619 selplane/f16cosel.bin f16cosel.bin
ac5_native --model f16cosel.bin
ac5_extract --all DATA.TBL DATA.PAC pac_names.txt extracted
```

On Windows, the native app and extractor convert the Unicode process command
line to UTF-8 before interpreting file paths.

The extractor preserves original file formats; the renderer can consume OBJ
meshes and supported static ACM aircraft bundles directly. Name lookup accepts the
`member index path` text format and compares paths without case sensitivity.
Bulk extraction writes one directory per member and uses `__ac5_slots/index`
paths when no safe archive path is known or when multiple slots collide by
name, including case-insensitive and file-versus-directory collisions. It
normalizes slash styles, rejects unsafe and nonportable names, and refuses
output paths that resolve through symlinks outside the selected output
directory.

Arrow keys/WASD pitch and roll, Q/E yaw, R/F set throttle input, Space fires the
selected missile/special, either Ctrl key fires the gun, Tab switches the
selected missile/special, P toggles approach control in flight mode, and G
toggles the viewer gear state. These actions also map to gamepad South (selected
weapon), East (gun), North (weapon switch), West (gear), and D-pad Up (approach
control); the left stick controls pitch and roll, the right stick controls yaw,
and triggers provide throttle. In the
default model viewer, pitch, roll, and yaw rotate the demo model. `--flight-rules`
loads aircraft parameters from a `.pmb` file. Add
`--weapon-rules <aircraft-member.bin>` to load weapon records from the complete
aircraft archive member; it contains the loadout `.wxb`, gun `.wpb`, and missile
`.wpb` records alongside the `.mpl` model. Flight controls drive the aircraft
and weapon inputs advance on fixed flight ticks. The window title reports
ammunition and shot events. Simple gun/missile projectile meshes are rendered
and advance under the decoded launch speed and gravity rules.
`ButtonInput` exposes held, pressed, and released states for native game logic.
The dependency-free `weapon_controller` game-core module accepts portable
loadout rules and emits gun, missile, and special-weapon shot events. The ACM
loader decodes aircraft bundle slots 7/8/9, and
`ac5_model_info --weapons <aircraft-member.bin>` reports their loadout and
weapon parameters, including aircraft-specific gun mount pitch. The viewer
consumes those rules in flight mode, advances a
bounded native projectile pool at fixed ticks, and uploads its fixed-topology dynamic
mesh through the Vulkan renderer. Its 128 slots recycle shots after a 12-second
native presentation lifetime; this is not claimed as the retail lifetime.
Gun spread is applied on two axes with a portable, seedable deterministic
native random stream; it does not reproduce the retail PS2 random sequence.
The decoded aircraft gun mount pitch is applied to the gun aim direction.
When an animated ACM model exposes `@` attachment points, gun rounds launch
from alternating points using their evaluated world transforms, falling back
to the aircraft origin if the selected point is unavailable. Special-weapon
shot events do not create projectiles until their own launch parameters are
decoded.
The game-core API can optionally sweep those projectiles against caller-owned
spherical target bounds and return impact positions/ids. This is a generic
collision boundary only; the viewer does not currently supply targets, and
retail hit volumes and damage behavior remain undecoded.
`map_engine_lever` ports the generated throttle routine's split into dry power,
afterburner, and airbrake channels. `advance_airborne_speed` ports the scalar
airborne speed envelope, including density, climb/dive effects, spool response,
and stall factor, from recovered rules supplied by the caller.
`advance_flight_axis` ports the speed-dependent pitch/roll/yaw rate lookup and
input slew law. `advance_aircraft_flight_tick` composes those scalar updates,
applies recovered airborne trim and stall nose-drop, supports the explicit
approach-control stick law, integrates native body attitude, and advances
position with path lag and airborne sink terms. Approach mode captures the
reference pitch and suppresses steady trim and inverted-flight sink.
`AircraftFlightSession` supplies the reusable 60 Hz fixed-step clock for the
game core; it bounds catch-up to 100 ms and leaves both state and accumulated
time unchanged if a simulated tick rejects its input. Its per-tick callback
lets aircraft animation advance from each post-physics state; the viewer stages
animation, gear, approach-mode, weapon, and projectile state until all flight
ticks in the frame succeed. It runs the carrier-aware
tick wrapper, which reduces to the normal tick outside carrier modes.
`advance_frame_until` is also available when a caller must stop on a fixed tick
(for example, on ground collision); it commits the event tick and leaves the
unconsumed accumulator available to the owner of that stopped state.
`advance_frame_until_contact` samples pre-tick spatial contact per fixed step,
keeps it separate from script contact bits, and lets the observer adjust the
post-physics position before the following tick. `update_aircraft_ground_contact`
ports the generated runtime's half-second contact-loss latch for terrain-based
ground state, avoiding a one-tick ground/air mode flip when the contact sample
briefly misses. The core also exposes `normal_aircraft_ground_trace` and
`aircraft_ground_marker_attitude` for gear-dependent landing alignment. The
trace is renderer-independent: callers provide I/K/J marker samples and terrain
height, sweep, and vertical-snap callbacks. The viewer now samples ACM I/K/J
markers at the generated gear channels and traces against the rendered mission
heightfield mesh when the gear is fully down. The native ACM loader derives each
geometry-less attachment label from the part's no-link source word, and
`ac5_model_info --parts` displays it. This makes I/K/J discovery possible from
the model data without an external rig file.
`ac5_model_info --ground-markers` evaluates and prints the animated marker
origins in I/K/J order at neutral channel values; append a gear-channel value
to compare poses. `ac5_mission_info --terrain-query DATA.TBL DATA.PAC 3 x z`
reports both the recovered mission-height query and the rendered-mesh
intersection at that world coordinate.
`AircraftFlightFlags` keeps contact and mode bits as separate native bytes;
`set_aircraft_flight_mode` and `set_aircraft_approach_control` preserve unrelated
bits when changing modes. Each successful tick advances the recovered
command-2 contact-bit sequence after applying that tick's physics. The
ground-contact option uses the recovered
per-tick acceleration and maximum
speed clamp, with caller-supplied carrier scaling and bias; it does not perform
terrain collision or marker-based alignment.
`advance_aircraft_carrier_tick` adds the carrier hold window, two-second
throttle gate, reset behavior, and carrier acceleration adjustments while
preserving the source tick's delayed contact-flag transitions.
`load_aircraft_flight_rules` reads the standalone 400-byte
aircraft `.pmb` parameter file, and `ac5_flight_info` reports its decoded
speed and axis tables (for example, `ac5_flight_info f04g.pmb`). Terrain
collision, marker-based ground alignment, and a runtime ruleset selection UI
remain future work.
The dependency-free `count_airframe_surface_hits` helper ports the recovered
six-point body-axis collision query. It takes model extents and a caller-owned
terrain sampler, and `aircraft_body_axes` supplies axes from native flight
attitude. Automatic flight-session collision response still needs terrain and
flight-session wiring; the `.pmb` flight parameters do not contain hit extents.
`ac5_mission_info --hit-extents DATA.TBL DATA.PAC <aircraft-entry-index>`
loads the six values directly from that aircraft's mission-entry AHM payload.
`decode_ac5_mission_units` reads the unit/script child of a decompressed mission
`.mmb` into native unit sides, formation members, object hit metadata, and the
unit -> set -> act -> order hierarchy. Spawn orders and type-2 waypoint orders
include decoded world positions; other order types retain their raw parameter
words. Additional member-associated set bundles are decoded and deduplicated by
source offset. Spawn and triggered-Act records also expose their decoded
trigger fields.
`evaluate_ac5_mission_trigger` evaluates the recovered time, player-proximity,
target-inactive, and mission-state conditions using caller-supplied native
runtime state. `step_ac5_mission_order` executes type-3 stop waits, type-5
conditional/counted jumps, and type-6 mission-state writes using per-unit
script state. The mission state table is 32-bit, matching the runtime store;
script operands remain signed 16-bit values. Stop mode 2 accepts live target
positions and side IDs and applies
the recovered friendly/hostile camp and distance rule. Movement and activation
orders remain separate systems. Waypoint completion uses caller-provided unit,
follow-target, terrain, and post-movement ground state while applying the
decoded timer, mission-state, radius, and ballistic-ground conditions.
`eligible_ac5_mission_acts` scans a Set's first orders in source order.
`select_ac5_mission_act` applies source-order arbitration and remembers consumed
spawn gates per phase (or permanently for state gates); the caller creates the
entities and applies the returned disappear movement mode. Consuming the Act's
type-0/type-1 first order advances its script cursor; type-4 lead orders also
advance after their no-op handler. `resolve_ac5_mission_spawn_transform`
resolves absolute and additive target-relative type-0 placements, including
optional terrain-relative altitude and model-bottom offset. Placement mode 1
accepts a live group/member position, and mode 2 rotates its vertical/rearward
offset by the target yaw before adding it to the target position.
`build_ac5_mission_spawn_preview` resolves the statically available initial
spawns, follows group-relative dependencies, and returns native transforms plus
an unresolved-order count. It lives in `ac5_game_core` and has no renderer or
SDL dependency. Individual member targets are resolved only when member zero's
decoded formation offset is zero.
`begin_ac5_mission_steering_order` initializes waypoint speed and velocity;
it also applies the recovered movement-mode and payload flags and stores the
order's `+0x3c/+0x3d` parameters.
`advance_ac5_mission_steering` handles modes 0–8, including direct-turn,
turn-limited, takeoff, ballistic, trail displacement, arrival-attitude blending,
and keyframed paths. Mode-8 path keys are decoded from the order's secondary
bundle and follow the live group-0 reference position. A mode-8 order without a
complete path header/key table is rejected during decode; the runtime steering
and order-step APIs likewise report a missing path as unsupported rather than
silently completing the movement order.
`ac5_mission_info <decompressed-unit-table.mmb>` summarizes a raw unit-table
block, or `ac5_mission_info --archive DATA.TBL DATA.PAC <member-index>` follows
the selected mission-pack member's slot 0 and then the wrapped mission-entry
slot 0 before decoding its unit table. Both paths use native core APIs and do
not need the renderer or extraction tools. Runtime AI/object creation remains
separate integration work.
The dependency-free `build_heightfield_mesh` helper converts validated row-major
world-height samples into indexed `MeshData` with upward-facing triangles,
smoothed normals, and normalized UVs. This is the native terrain geometry stage;
`build_heightfield_mesh_lod` creates lower-density full-extent variants for
distance-based terrain selection. The viewer builds stride-1, 2, 4, and 8
meshes and switches between them using native world-distance thresholds.
`decode_ac5_terrain_height_lanes` additionally converts the generated terrain
path's signed 32-bit height lanes to world elevations using the observed scale
of 80. The observed `.mhd` row decoder is available; `.mii` semantics, patch
selection, and retail LOD seams still require further format work.
`sample_heightfield_surface` evaluates the exact piecewise-planar triangles
used by the native terrain mesh, so callers can query terrain elevation at a
world X/Z point without depending on Vulkan. It reports out-of-bounds queries
instead of silently clamping them.

The dependency-free asset library also exposes `decode_ac5_map_texture_set`
for extracted mission `.mti`, `.mtd`, and `.mtc` files. It validates the four
64-by-64 through 8-by-8 indexed tile mip regions, decodes the 256-entry GS RGBA palette,
and returns a compact tile atlas plus square 16-bit map entries (including the
64-by-64 extracted map form).
For decompressed mission containers, `decode_ac5_mission_terrain` consumes the
inner files returned by `DataArchive::split_member`: height files 0/1, vertex
color files 2/3/4, tile-index files 5/6, and tile pool/palette files 7/8. It
assembles overlapping 16-by-16 blocks into 513-by-513 elevation and color
grids plus a 256-by-256 tile-index map, then decodes the atlas. The recovered
grid spacing is 1,024 world units per height sample, centered at vertex
(256, 256); tile cells span 2,048 world units. `build_ac5_mission_terrain_mesh`
turns those layers into mission-scale textured geometry with vertex tint,
height-derived normals, and per-tile UVs. This decoder uses only native
asset/core APIs and does not need the renderer or extraction tools.
The input system opens one connected gamepad at startup and scans the remaining
connected devices when the active gamepad is removed. It also handles devices
connected after startup.

## Tests

Configure and build the dependency-free core, then run its tests with:

```powershell
cmake --preset core
cmake --build --preset core
ctest --test-dir build/core --output-on-failure
```

The `ac5_native_tests` target covers tiled image layout round-trips, palette
decode, ADPCM, and model shading. It needs only the core libraries.

## Porting rule

Treat `generated/ps2_code_*.c` as behavioral evidence, not as native game
source. Those files encode MIPS register state, PS2 address-space reads and
writes, delay slots, and hardware calls through `ps2_runtime.h`. Port a
subsystem by identifying its game-level inputs and outputs, then implementing
that behavior against native data types and services here. Keep PS2 emulation
out of this directory.
