# Generated Code and Porting Review

This is a review note for the source checkout. References to `generated/`,
`runtime/`, and `config/` describe read-only source material. The native
project does not build or link any of those files.

The standalone boundary was checked against `native-port/CMakeLists.txt` and
the source/include tree: its targets compile only files under `native-port/src`
and shaders under `native-port/shaders`; there are no includes or links to the
parent project's `generated/`, `runtime/`, `config/`, or `third_party/` trees.
The `core` preset builds the C++20 archive/game libraries and CLI tools without
SDL3 or Vulkan. The optional renderer uses installed SDL3, Vulkan 1.3, and
`glslc`; Windows command-line parsing links the system `shell32` library.
These are explicit toolchain/platform requirements rather than dependencies on
the source checkout. The installed core package was consumed by a separate
CMake project linking the exported `ac5::game_core` target. Both the `core` and
`renderer` presets built successfully in this workspace; their test executable
was compiled but not run.

## Findings

1. `generated/ps2_code_0000.c` through `ps2_code_0042.c` are MIPS translations
   emitted as C. They preserve register-level state, guest addresses, delay
   slots, and hardware calls. Renaming these files to C++ would keep the PS2
   runtime dependency and would not create native gameplay modules.
2. `runtime/src/ps2_*` supplies the guest CPU, memory, VU, GS, SIF, CDVD, IOP,
   SPU2, and kernel behavior needed by those translations.
3. `runtime/src/rn` already replaces parts of the GS/VU drawing path with
   Vulkan work, but its input contract remains PS2-shaped: GS state, VU mesh
   blocks, and `ps2_vk_*` records. The standalone renderer needs native
   camera, mesh, material, texture, and scene data instead.
4. `config/game_symbols.txt` identifies useful higher-level port boundaries,
   including `pac_ReadTable`, `pac_Poll`, `ulz_Step`, `pac_FileInMember`,
   `mission_Render`, `player_Render`, `hud_Draw`, and `main_Loop`.
5. The generated `main_Loop` (`0x0031C388`) is a guest-state frame loop that
   dispatches scene work and repeatedly calls other guest routines. The
   generated `player_Render` (`0x0012BEB0`) includes HUD and model-render work;
   it is not evidence of the aircraft flight-control update. Separate
   disassembly later identified the scalar speed routine (`0x0016D408`),
   flight-axis routine (`0x0016B828`), throttle mapping (`0x0016BF50`), and
   flight-state integration path. The native model ports these from their own
   control and state boundaries; none of its flight behavior is inferred from
   `player_Render`.
6. The frame loop calls `0x0032AD00`, whose generated body calls the Pad2
   state/profile/read entry points and translates controller state into game
   memory. It stores the current button mask at `+0x394` and a newly-pressed
   edge mask at `+0x396`. The native `InputSystem` replaces that device
   boundary with SDL3 keyboard/gamepad input and game-level flight axes/actions;
   `ButtonInput` exposes held and pressed states, plus the corresponding release
   edge. Copying the Pad2 structures or guest offsets would retain a
   hardware-specific ABI.
   SDL keyboard and gamepad button down/up events are also latched until the
   next sample, so a press and release received in one event-pump cycle still
   produce both native edges even if the final polled held state is up.
   Its current axis/button choices and dead zone are native demo defaults, not
   a faithful port of the title's profile, calibration, and button mapping.
   Inspection of `0x0032AD00` shows why a direct SDL button substitution would
   be inaccurate: the routine walks 16 mapping slots, derives each mapped bit
   from profile/configuration data and a sampled controller buffer, then
   stores the current mask at `+0x394` and rising edges at `+0x396`. It also
   checks device/profile state before accepting the sample. `0x00105FF0` is
   one consumer of the held mask; it passes the mask to `0x00105CE0`, which
   converts individual bits into a compact per-action state record. These
   routines establish the held/edge/action-state boundary, but not the
   user-facing meaning of the bits. That still needs to be recovered from the
   action-record consumers and profile/configuration data before native game
   actions can be bound faithfully.
   A separate consumer, `0x001F9BF0`, checks rising-edge bits `0x40` and
   `0x10` while updating a bounded list index and invoking selection feedback.
   This supports identifying those logical actions as opposite list-navigation
   inputs in that caller only; it does not identify the underlying physical
   pad buttons or establish aircraft-control meanings.
   For a non-empty list, that caller increments then decrements the signed
   index when the respective rising-edge bits are present, wrapping an index
   below zero to `count - 1` and an index at or above `count` to zero. The
   native `advance_selection` helper ports this index behavior for valid list
   state without copying the guest object layout, action bits, or UI callbacks.
7. `CParamPlaneBin::_Read` (`0x001AB9A0`) is a consumer of the game's parsed
   configuration tree, not a byte-level aircraft record reader. It performs
   four indexed named-field lookups and routes missing fields through the
   guest fatal-error routine. `CPlParamBs::_Read` (`0x001AF6B0`) reads one
   32-bit value through the same tree interface and errors when it is zero.
   Neither function alone identifies the field names, units, defaults, or
   complete aircraft parameter schema; porting flight behavior from these
   functions without the parser and real input data would encode guesses.
8. The shared `CTxtio::_Decode` (`0x00324130`) builds a typed entry table;
   `_GetInt` (`0x00324BC0`) walks entries at a fixed `0x18C` stride, checks
   their stored type, and only accepts integer-like types 5 and 6. Its result
   comes from the typed entry value rather than directly from the source
   bytes. A native port therefore needs the decoder and typed access rules
   before it can faithfully load the plane configuration; a generic
   `key=value` parser would not preserve this behavior. `_Decode` also calls
   `0x003234F8` throughout its source scan and table construction; that helper
   is a 608-instruction generated routine with no recovered name in the
   current symbol map. `_Decode` keeps separate tables at object offsets
   `+0x4C0` and `+0x4C8`, and entry records use `0x18C`-byte strides. The
   getter family (`_GetS8` through `_GetData`) each performs its own typed
   lookup. Those observations establish the storage shape, but not enough of
   the tokenizer and lookup helper contracts to implement a faithful
   standalone parser yet. Inspection of `0x003234F8` narrows its role: it scans
   signed source bytes, forms a temporary line, recognizes CR/LF and the `0xFF`
   end marker, and processes spaces, tabs, and `#` comments while building an
   entry. Its complete token grammar, type/value encoding, and effective
   input/output bounds remain unresolved, so this is not enough to implement
   the aircraft configuration loader faithfully.
9. `CObjUBin::_GetReadBuffSize` (`0x001ADDD8`) and `_Read` (`0x001ADEE8`)
   traverse counted children in the same typed object tree and delegate to
   `CSetBin`/`CActBin` readers. They describe hierarchical object/unit config,
   not the map mesh byte stream. `config/pac_names.txt` lists mission members
   such as `map/m01a/pmmdlm01a.bin` and `pmtexm01a.bin`, which are more direct
   model/texture investigation targets. Those archive members are not present
   in this checkout, and the current symbol map does not expose their geometry
   decoder.
10. `Mpl::_Control` (`0x001281B8`) has a 1,063-instruction generated body
    operating on a live object and calling dozens of other control/render
    routines. Its name and call shape identify it as runtime behavior, not a
    self-contained `pmmdlm*.bin` parser. No file magic, byte cursor, or bounded
    model-read contract is identified from that routine alone, so it is not a
    sound basis for implementing the model loader.
11. `ulz_Setup` (`0x00102A78`) stores the low 24-bit output size and data
    offsets from the 16-byte header, plus a shift-derived distance mask.
    `ulz_Step` (`0x00102BF8`) consumes control words most-significant-bit first,
    dispatches literal bytes or 16-bit back-reference tokens, and yields after
    2,500 control words. Its MIPS variable shifts mask counts to five bits.
    The native decoder follows those layout and scheduling rules while adding
    explicit input-range and invalid-back-reference checks.
12. `scene_Dispatch` (`0x0031CF00`) reads two byte-sized indices at object
    offsets `+0x08` and `+0x09`, then calls through a two-level handler table.
    `scene_Next` (`0x0031CF88`) treats the byte at `+0x0A` as a pending-state
    value with `0xFF` as the no-transition sentinel; otherwise it copies that
    value to `+0x08` and clears `+0x09`. Before advancing the allocator mark at
    object `+0x04`, it calls `CMalloc::_FreeMark` (`0x0031CE00`) with the byte at
    `+0x0B`. `_FreeMark` clears allocation records at or above the requested
    mark and moves the current mark back. `SceneStateMachine<State, Substate>`
    preserves deferred transitions and substate reset with typed state values;
    `SceneDispatcher` provides native state/substate handler lookup. Committing
    returns the old and new states so a native owner can release state-scoped
    resources. The allocator-mark lifetime contract remains unported.
13. `pac_FileInMember` (`0x001029E0`) is an accessor over the runtime's loaded
    member descriptors, not the parser for a raw decompressed PAC member. It
    masks its requested slot to eight bits, indexes descriptors at `+0x54` in
    `0x14`-byte strides, and bounds-checks the slot against the selected
    descriptor's count. The shared `0x0031E030` helper then resolves the
    selected offset through descriptor fields at `+0x04` and `+0x08`; it
    returns null for an out-of-range slot or zero offset. Separately, the
    runtime `pac_load_header` reads the decompressed member's leading count and
    32-bit offset array, rejects counts above `0x10000`, and validates offsets
    against the member length and table end. Native `DataArchive` ports this
    raw-byte layout with bounds-checked vectors; it deliberately does not
    reproduce the guest descriptor table or its pointer arithmetic.
14. `pac_QueueMember` (`0x00101D98`) appends an in-range member index to a
    seven-entry request array and does not deduplicate requests. Native
    `ArchiveLoadPlan` preserves those bounds, insertion order, and duplicate
    requests while using ordinary C++ indices. `ArchiveBatchLoad` executes
    those requests with resumable decompression and ordered native byte-vector
    results, and detects if its archive is reopened during the load.
    `pac_BufferSize` and `pac_Poll`
    still depend on archive-table layout, guest allocation, and CDVD state;
    their sizing and I/O lifecycle are not claimed as ported.
15. `tex_Upload` (`0x0031F2B0`) reads dimensions and transfer fields from an
    already prepared guest texture object, builds GS `BITBLTBUF`, `TRXPOS`,
    `TRXREG`, and `TRXDIR` state, then submits the transfer through the guest
    graphics path. The `CTim2::setVpos` routines similarly update resident
    texture state. These are PS2 upload operations, not evidence for the native
    `pmtex` decoder: the sampled placed-model bundles are independently decoded
    as nested indexed GIMs with a shared palette. Indexed aircraft GIMs also
    decode to `TextureData`; other terrain and GS material formats remain
    unknown.

## Work in `native-port/`

- A separate CMake project and C++20 source tree. Nothing in `generated/`,
  `runtime/`, or `config/` is compiled or included.
- A bounds-checked native ACM/VIF static mesh loader for standalone ACM blocks
  and `.mpl`/`.bin` model bundles. It converts decoded triangle strips to indexed
  `MeshData`, uses the ACM pre-order child counts and chain-owner indices to
  place geometry, and loads the first indexed GIM found in the bundle. For
  placed-model PM texture files, a separate decoder reads the shared RGBA
  palette in root slot 0 and the nested full-resolution indexed GIM set in
  root slot 1, applying CSM1 palette ordering. `--pm-textures` reports these
  textures; `--material-textures` binds geometry part word 3 to the nested
  texture slot when explicitly requested. This mapping is supported by the
  placed-model loader call sites and observed PM bundles, but remains
  provisional across model families.
  `ac5_model_info --pm-materials` cross-checks geometry-part word-3 values
  against nested texture slot IDs. Three sampled mission pairs have no
  unresolved references: M01A (1 geometry part, 2 textures), M02A (12 geometry
  parts using slots 0-6, 41 textures), and M03A (1 geometry part, 7 textures).
  This supports the slot interpretation for those bundles only. The
  `--textures` diagnostic enumerates every supported GIM and its nested
  bundle-slot path. The extracted F-4G model bundle has indexed GIMs in slots 8
  and 9 at 512x512 and 128x128; those aircraft texture slots remain distinct
  from the placed-model PM texture scheme.
  A direct extraction of M01A's standalone `m01a_sea.gim` is 8,304 bytes: its
  128x128 4-bit image plane, 16-byte palette block header, and 64-byte RGBA
  palette account for the complete file, so this image contains no embedded
  mip levels. M01A's PM texture bundle separately contains two 256x256 full-
  resolution images at nested slots `1/0` and `1/1`; these are distinct
  material textures, not a lower-resolution mip pair. The available sample
  evidence therefore supports the current GPU-generated mip chain for each
  imported texture, but does not establish PM texture-slot LOD selection.
  On the extracted F-4G selection bundle it produces 5,538 vertices and 2,920
  triangles with assembled bounds of about 59 x 26 x 96 game units, matching
  the measured reference model. The F-16 selection bundle produces 4,108
  vertices and 2,172 triangles with assembled bounds of about 49 x 26 x 79
  game units. The F-16 player bundle has 57 part records and 33 geometry chains;
  its 29 explicitly owned chains import to 3,102 vertices and 1,632 triangles.
  The four unowned chains are shape alternatives, so static import skips them.
  The flight viewer also has a part-preserving path that retains local meshes,
  parent indices, part kinds, track-reference indices, and translation,
  rotation, and shape animation keys. The viewer evaluates translation and
  Euler rotation tracks using the recovered direct stick/throttle channels.
  Compatible shape targets interpolate all decoded vertex attributes through
  per-frame dynamic Vulkan vertex buffers; differing topology falls back to
  nearest-key target selection. Aircraft-specific authority/configuration
  curves decode the 80-byte subfile-6 payload when supplied. Direct channel
  targets include the recovered speed curves and 60 Hz gear/flap easing. The
  four-state retail gear transition updates channels 33-39, derives its
  capability flags from part tags, and swaps fairing/leg visibility. The
  channel-5/6 roll mappings now use the ACM header flags and shared authority
  factor, including the low-speed `0x400` flap branch and `0x10`, `0x20`, and
  `0x40` alternatives. The recovered controller combines pitch, roll, and
  slip inputs at offsets `+4`, `+8`, and `+0x1c`; native slip remains zero
  until the flight state exposes a supported sideslip value. The `0x400`
  threshold comparison uses the recovered conversion from km/h to speed units
  per tick (`0.0237037018`). Aircraft-class meaning remains under
  investigation. The channel-8 speed curve and controller-flag `0x2` pair of
  channels 8/9 are also represented. Controller flag `0x8` selects the paired
  channel-10/11 pitch/roll mapping, with its `0x1000`/model-flag/slip override
  on channel 10. Controller flag `0x1` selects channels 1/2 mappings driven
  by model flags `0x1`, `0x2`, `0x80`, and `0x200`; without it, the `0x200000`
  slip/pitch variant applies to channel 1. Retail setup derives feature bits
  from populated animation channel slots 2, 4, 9, 11, and 25-27; the native
  viewer derives those bits from the union of decoded ACM translation,
  rotation, and shape tracks. The separate `0x1000` runtime override bit is
  not inferred by the viewer; both channel-16 and channel-17 override branches
  now accept the shared native controller value corresponding to `+0x20`.
  Channel 20 follows its ACM model-flag gate: `0x10000` selects the aircraft
  speed curve, `0x20000` selects the `+0x20` controller value, and other
  variants use the stateful channel-20 path under model flag `0x4`. The
  controller channel array begins at `CPlAnimCtrl +0x34`, making `+0x84`
  exactly channel 20. Retail advances that existing channel once per tick by
  config word 19 times `1/36`; model flag `0x2000` additionally scales the
  increment by `min(speed_units_per_tick / 35.5555534, 1)`. The resulting value
  wraps by one 100-frame cycle at either endpoint. The native tick preserves
  this accumulator and leaves it out of the generic easing step. The same
  `0x2000` branch also calls `0x379f78` and `0x388a88` with a global matrix at
  `0x443bb0`; that auxiliary matrix effect is not yet represented in the
  renderer. Controller flag `0x1000` also drives channel 24 to its full
  endpoint; its source remains unavailable in the native viewer.
  `ac5_model_info --parts` reports the hierarchy and decoded track/key counts
  without opening Vulkan. It now also prints the eight raw part-table words,
  local position, and Euler rotation for every part, including records without
  geometry. An extracted F-4G bundle has
  62 part records but only 31 geometry parts; the other records include
  repeated upper bits in word 4 and `0xffffffff` chain sentinels. Those raw
  values give further reverse-engineering evidence. For placed-model files
  specifically, word 3 is used as a PM texture slot by the opt-in native
  material path; this interpretation is validated against M01A, M02A, and
  M03A but remains unverified for other bundle families.
  The F-4G `P`-tagged records include multiple mirrored and centerline points,
  including some parented beneath animated gear parts. Their tag alone does
  not establish which points are missile or gun launch origins, so projectile
  spawning remains at the aircraft origin until the retail selection path is
  mapped.
- A Vulkan 1.3 renderer using SDL3, dynamic rendering, synchronization2,
  indexed textured meshes, push-constant transforms, per-image depth
  attachments, resize-aware swapchain recreation, and staging uploads that
  flush non-coherent host-visible memory when required by the selected device.
  It selects an sRGB nonlinear swapchain format to keep direct transparent
  attachment blending in linear light and reports an explicit initialization
  error when the surface offers no supported sRGB format. A failed frame-fence
  reset or queue submission marks the renderer unusable so a later call cannot
  hang on an unsignaled fence or stale acquire semaphore.
- Native `Mat4`, `Camera`, `Scene`, `SceneStateMachine`, `MeshData`, `Vertex`, and `TextureData` types, plus
  OBJ/MTL, P3/P6 PPM, and true-color TGA importers for converted assets. OBJ material sections
  become separate scene draws, `Kd` multiplies vertex color, scalar `d` and
  `Tr` set vertex opacity, and `map_Kd` images are loaded as textures. The
  scene uses shared transform IDs for entity-level movement, ordered parent
  transforms for articulated pivots, and local per-draw matrices for
  parts/materials. The parent ordering rule prevents cycles and resolves world
  matrices once per frame. OBJ scene imports stage only new assets and
  append transactionally, so separate entities can share one scene without
  copying existing geometry or tying transforms to GPU resources. The
  importer validates and retains OBJ normals, computes flat or smoothing-group
  normals when absent, and ear-clips concave polygons while retaining shared
  position/UV/normal tuples in indexed geometry. Generated smooth normals are
  shared across UV seams. The Vulkan vertex path transforms normals and applies
  a simple directional diffuse light with ambient fill.
- The SDL sample runs its demo frame update through the native typed scene
  dispatcher and queues shutdown as a deferred state transition. This exercises
  the native state API without claiming to reproduce AC5's unresolved scene
  lifecycle and allocator-mark behavior.
- SDL keyboard and gamepad input represented as ordinary native flight axes
  and actions. `FlightInput` itself has no SDL or Vulkan dependency, so a
  future simulation layer can consume the same native action contract.
- `map_engine_lever` ports the throttle mapping at generated routine
  `0x0016BF50` into dry-power, afterburner, and airbrake channels. It accepts
  the native -1..1 lever and has no PS2 memory or runtime dependency.
- `advance_airborne_speed` ports the scalar envelope in generated routine
  `0x0016D408`: altitude density scaling, climb/dive speed changes, throttle
  target selection, spool response, and stall factor. It consumes explicit
  native parameter and state values; contact acceleration and world
  displacement remain separate work.
- `advance_flight_axis` ports the table interpolation, stall/full-control rate
  ramp, and input-dependent slew from generated routine `0x0016B828`. It returns
  native radians-per-tick state without requiring the guest aircraft structure.
- `decode_aircraft_flight_rules` reads the 400-byte `.pmb` layout recovered
  from `CParamPlaneBin::_Read` and the aircraft setup routine. `ac5_flight_info`
  shows the decoded F-4G tables directly from a local extracted parameter file.
- `advance_aircraft_flight_tick` composes the native speed and axis routines,
  applies recovered airborne trim and stall nose-drop, and integrates body
  attitude and airborne position with the generated routine's Euler-matrix
  order and path-attitude lag. Approach-control mode has its own native stick
  law and captured-pitch state. Ground contact has a native acceleration law,
  configurable carrier scale/bias, and envelope speed cap. Terrain collision
  and marker-based alignment remain separate. `AircraftFlightFlags` exposes
  independent contact and mode bytes, and its native setters preserve
  unrelated bits as the recovered mode setter does. Successful flight ticks
  also apply the recovered post-physics contact-flag transitions.
  `advance_aircraft_carrier_tick` preserves the carrier wrapper's hold window,
  throttle gate, contact acceleration modifiers, and special flag timing.
  `AircraftFlightSession` owns the bounded fixed-step clock in the dependency-
  free game core, leaving the SDL/Vulkan application as an input/render adapter.
  Its per-tick callback now advances flight animation from each post-physics
  state, so speed-dependent channels follow the same fixed ticks as flight;
  animation, gear, approach-mode, weapon, and projectile state is staged until
  the frame's flight ticks all succeed. The application now checks projectile
  advancement and supported gun/missile launches inside that staged callback;
  an invalid projectile state aborts the entire frame commit instead of being
  silently ignored.
- A bounded, resumable ULZ decoder and `DATA.TBL`/`DATA.PAC` reader. The
  decoder can yield after a caller-selected number of control words, and
  `ArchiveMemberLoad` exposes that budget at the member-loading boundary. Its
  blocking convenience API uses the generated routine's 2,500-word step
  budget. Variable shift counts follow MIPS semantics by masking to five bits,
  matching `ulz_Setup` and `ulz_Step`; packed reads remain bounds checked.
  These port the data behavior associated with the named archive
  routines while returning native byte vectors instead of writing into guest
  memory. `ac5_extract` exports a decoded member, indexed/named file, or an
  entire archive tree. Build it without SDL3 or Vulkan by configuring with
  `AC5_BUILD_RENDERER=OFF`.
- A native `ArchiveLoadPlan` matching the seven-entry member request limit
  recovered from `pac_QueueMember`. It retains request order and duplicates
  but leaves execution to native archive-loading code rather than guest/CDVD
  state. `ArchiveBatchLoad` steps this plan with the bounded ULZ decoder; its
  disk reads are synchronous, and it does not reproduce the guest/CDVD state
  machine.

## Data-format modules ported from runtime evidence

These pieces come from the runtime's pure-data code (layout tables and
arithmetic, not guest CPU or hardware state) and are covered by
`tests/native_tests.cpp` (`ctest`):

- `gs_image` - the tiled page/block/pixel layout of the game's image memory
  for 32/16/16S-bit and 8/4-bit indexed images, palette lookup (including the
  8-bit palette bit-3/4 exchange) and RGBA8 conversion. `tiled_bit_offset` was
  differentially checked against the original address routine over 450,000
  pixel positions for all five formats with identical results. The sampled
  placed-model `pmtex` bundles use nested indexed GIM images with an external
  shared palette and do not use this GS-memory layout; other terrain and GS
  material formats remain to be identified.
- `adpcm` - the 16-byte-block sound codec (filter table, shift, loop flags).
- `model_shading` - the plain, lit, and environment-mapped paths of the model
  vertex program as ordinary math. Environment mode returns the lit base pass
  and a second pass with transformed reflection coordinates and capped
  reflection color, following the environment path in
  `runtime/src/rn/rn_vp_model.c`. The Vulkan sample does not yet consume this game-specific
  shading output automatically; scene draws can select an environment texture
  and use vertex environment coordinates/color in a blended depth-tested pass.
  `model_mesh_adapter.hpp` converts the shading output's projective STQ and
  integer modulation channels into the native vertex representation and marks
  base-pass colors as pre-lit so the demo diffuse light is skipped. Wiring the
  game-specific output still requires decoded render state. The reference
  `rn_vp_model.c` reads clip rows, three light-direction rows, the light-color
  transform, environment rows, fog values, and mode/depth flags from VU state;
  those values are not carried by the ACM vertex streams parsed here. The next
  grounded step is tracing the render-state producer and material mode that
  populate those VU registers before connecting the adapter to ACM draws.
  `rn_vp_model.c`'s setup path reconstructs matrices from VU memory blocks at
  `0x10`, `0x14`, `0x20`, `0x24`, and `0x28`, stores the combined clip rows at
  `0x30`, and selects plain/lit/environment mode from VU flags `0x08`/`0x02`.
  These are per-draw VU command inputs, so the next trace must follow the game
  render caller that emits those blocks rather than the ACM file parser.

The generated code is present as behavioral reference. Game archives are not
checked into this repository; local copies in Downloads were used read-only to
validate an F-4G ACM/GIM bundle through the native loader. ACM blocks with
ambiguous chain ownership, other texture formats, and animation still need
work.

## Current limits and next work

The renderer uses per-draw texture IDs with scene-wide Vulkan descriptor
allocation and a vertex-color/diffuse-lighting pipeline. Scene meshes are
concatenated and uploaded in one staging batch into device-local vertex and
index buffers; per-mesh offsets preserve individual draw ranges. Textures
generate a GPU mip chain when the selected sRGB format supports linear blits,
otherwise they use one level. MTL support handles `Kd`, scalar `d` dissolve,
scalar `Tr` transparency, and simple `map_Kd` paths to PPM or uncompressed/RLE
true-color TGA images. The sampler enables anisotropic filtering up to
16x when the device supports it. The native ACM/VIF loader decodes supported
static aircraft bundles, including all five LOD meshes, parent-level companion
ACM meshes, thresholds, and per-level textures from nested `lodplane/*.bin`
containers. The F-16C and F-4G bundles both import through this path. The viewer
selects among all five levels using the recovered distance thresholds and
mouse-wheel camera zoom. The GIM decoder handles indexed 4/8-bit textures.
Placed-model bundles can load a shared-palette full-resolution texture set and
bind source part word 3 to those slots when explicitly requested. The OBJ
importer remains an interchange path. Ambiguous ACM part ownership, mip-level
selection for PM textures, other texture formats, and broader model animation
remain open.

The dependency-free `heightfield_mesh` module now converts finite row-major
world-height samples into indexed native `MeshData`, with normalized UVs,
upward-facing winding, and normals derived from neighboring samples. It gives
future terrain decoders a renderer-ready geometry stage. The viewer's
`--heightmap` path loads a supported PPM/TGA/GIM image and maps its red channel
to centered elevations with configurable scale and sample spacing; an optional
`--texture` supplies terrain color. AC5 terrain/GS record decoding, tile
adjacency and LOD, material selection, and terrain collision remain separate
work; no game-specific field layout is inferred by this helper.

Scene upload builds replacement mesh, texture, and descriptor resources before
replacing the active scene. A scoped cleanup now releases staged Vulkan handles
when CPU allocation fails partway through upload, and the public upload API
reports allocation and length failures through its error string instead of
letting those exceptions escape. The old uploaded scene remains live when
replacement creation fails.

Textures containing non-opaque alpha and meshes containing non-opaque vertex
alpha use a separate blended Vulkan pipeline. Opaque draws retain depth writes;
transparent draws are sorted by the transformed mesh bounds center and rendered
back-to-front with depth writes disabled. This object-level ordering does not
correctly order intersecting or interleaved transparent surfaces, which need
finer-grained sorting or a different transparency technique.

The PM texture path covers the sampled full-resolution placed-model set;
remaining texture work includes mip selection and other texture families.
Game-accurate flight behavior can follow once aircraft parameter and control
routines have been mapped to native state. Archive files remain outside the
repository; the source checkout contains only the filename index and native
format code.

The generated `player_Render` entry (`0x0012BEB0`) is a large orchestration
routine, not a self-contained model renderer. Its early calls include
`0x001343E0`, `0x00133E00` (HUD), and `0x001344D0`; later branches call
`0x0012D580`, `0x0012DA70`, and `0x00131850`. The two short helpers
`0x001343E0`/`0x001344D0` resolve object data through offset `+0x430`, submit
it to shared scene helpers, then branch on packed state at `+0x4DA`. This
establishes that player rendering delegates model/object state through shared
helpers, but does not identify the VU constant producer. The next useful trace
point is the shared scene submission path (`0x00131850` and its callees) and
its caller-side matrix/state preparation; the generated renderer runtime can
explain consumption but must not become a native-port dependency.

Tracing those callees shows that `0x001343E0`/`0x001344D0` first resolve the
`+0x430` object through `0x001D5FA0`/`0x001D5FB8`; both wrappers tail-call
`0x0031F2B0`, with the second wrapper adding `0x30` to its lookup base. They
then call `0x003211A8`, `0x001D6E70`, and `0x001D70F8`. `0x001D6E70` writes a
small descriptor structure containing tagged values such as `0x10000008` and
`0x50000008`, initializes associated arrays, and dispatches by an input index.
The shared `0x00131850` path additionally calls `0x001EAD78` to look up an
indexed record, zeroes temporary four-vector blocks, and invokes vector-copy/
conversion helpers `0x00328998`/`0x00328B28`. This is enough to distinguish
object lookup and renderer-command preparation from the VU model interpreter,
but the exact descriptor fields and the producer of VU blocks `0x10`-`0x30`
remain unresolved. The next trace should follow the returned object data into
`0x0031F2B0` and the record consumer after `0x001D6E70` to identify the native
matrix/material source.

`0x0031F2B0` has now been inspected: it reads the word at object offset `+0x0C`
and, when nonzero, decodes byte/halfword fields from that referenced record
(offsets including `+0x02`, `+0x03`, `+0x08`, `+0x0A`, `+0x0C`, and `+0x0E`).
It packs those values into several temporary records using renderer tag words
such as `0x10000006` and `0x50000006`, then repeats similar conversion for
object words at `+0x10`, `+0x14`, `+0x18`, `+0x1C`, and `+0x20`. This confirms
`0x0031F2B0` is translating compact object data into command records; it does
not itself expose a native-ready material or world matrix. Identifying the
referenced record schema and command consumer is still needed before mapping
any of these fields onto the native scene API.

The native Vulkan path no longer hardcodes the viewer's diffuse light in
`mesh.frag`. `Scene::set_lighting` accepts a finite nonzero direction and an
ambient factor in `[0, 1]`; the renderer passes these values in a 128-byte
push-constant block, the Vulkan minimum. The current values remain viewer
defaults, not recovered game constants. Per-vertex `light_response == 0`
continues to preserve colors produced by the separate portable model-shading
adapter. A zero or overflowed transformed normal now contributes ambient light
only instead of normalizing a degenerate vector in GLSL. Shader loading also
bounds SPIR-V allocation and checks its magic word before creating Vulkan
modules. A future game scene can set lighting once per scene without adding
PS2 runtime state or dependencies.

Player fire input is connected to the viewer when flight rules and aircraft
weapon slots are loaded. The recovered
`Mpl::_Control` pad table uses logical action `0x0c` for the selected missile or
special weapon and `0x0d` for the gun; these are configurable actions, so the
native Space/Ctrl bindings are platform policy rather than fixed retail button
equivalents. The gun uses the aircraft's slot-8 record for its `0.15 s` shot
interval, muzzle speed, and gravity, and the aircraft slot-7 loadout record's
`+0x04` gun mount pitch, and decrements gun ammunition per shot. The mount
pitch is converted from degrees and applied to gun aim only. Slot-8 spread is
decoded and applied to gun direction on
two axes using independent integer samples mapped through `((sample % 200) -
100) / 100`, matching the recovered square-cone distribution. The portable
projectile system owns a caller-seedable xorshift32 stream (default seed
`0xAC5D15FE`) so firing stays deterministic and independent of the PS2 runtime;
the retail random generator remains open. The player firing path alternates
the `@` model attachment-point index, and the viewer resolves those ACM parts'
animated world transforms for gun launch positions, falling back to the aircraft
origin when the selected point is unavailable. The generated gun edge reaches
`func_00109718`, which selects and fills a shot record before calling
`func_00349DA0`; that wrapper only forwards into `func_00352EC0`, which in turn
delegates to the 110-instruction `func_00353460` generic effect/object path.
That downstream routine allocates or updates pooled records and dispatches to
multiple helpers. Its `func_00352D38`/`func_00352DC8` writers copy fields into
fixed-layout records and advance a global guest allocator counter through the
GP-relative address `r28 - 0x2768`; those writes establish a guest object-pool
boundary, not native projectile kinematics. The call chain does not establish
a standalone dispersion formula. Missile firing depends on slot-9 data, two
alternating pylons, per-pylon reload timers, and an optional lock target. The standalone
`weapon_controller` module models weapon switching, gun cadence/ammunition,
alternating standard-missile pylons, per-pylon reload timers, and ordinary
special-weapon pylon events. It accepts portable rules and emits native shot
events without linking runtime code. The projectile layer accepts gun and
standard-missile events only; it rejects special shots instead of borrowing
standard-missile speed, gravity, and guidance values. Special-weapon projectile
parameters and behavior remain open. The ACM loader decodes the aircraft
loadout/gun/missile slots, and `ac5_model_info --weapons` reports the decoded
values for extracted aircraft bundles. Weapon state advances on fixed flight
ticks, and the viewer reports ammunition and emitted event counts in its
window title. A bounded native projectile pool inherits the aircraft's actual
per-tick displacement velocity (including path lag and vertical sink), adds the
decoded launch speed along body forward, applies gun gravity only, and renders
through the existing dynamic-mesh path. Standard missiles use the decoded
`range2` distance for expiry, have no gravity term, skip steering outside their
seeker cone, and apply the recovered full turn step capped by decoded turn rate.
The 128-slot pool also applies a 12-second native presentation ceiling, which
is not recovered retail behavior; missile `range2` expiry will normally occur
first. Missile propulsion now adds 5 km/h per fixed 60 Hz update up to the
recovered 3,000 km/h cap, using a fractional-update accumulator. Projectile
advancement processes the full requested interval
up to that lifetime in 1/60-second substeps, clipping the last step to each
shot's remaining lifetime and checking that swept segment for impact before
expiring it. Long caller updates do not silently discard simulation time.
Standard missiles can now receive an
optional target point at
launch; target assignment checks the decoded seeker half-angle, and guidance
turning is capped by the decoded missile turn rate. A native API can refresh an
already assigned guidance point for a moving target; the viewer does not yet
acquire targets or call that API. A separate game-core overload sweeps each
substep's projectile segment against caller-owned spherical target bounds,
deactivates the first-hit projectile, and returns target id, projectile slot,
shot kind, and impact position. Target ownership, hit volumes, and damage
remain caller responsibilities; the viewer does not wire target bounds, and
this generic collision API is not a recovered retail rule. Pylon-specific
launch offsets and retail collision/damage behavior remain open. Special-weapon ids `8` and
`0x13` are surfaced as unsupported sequences instead of being treated as
ordinary single-pylon launches.

The generated input-state updater at `0x00103B18` compares the current masks
(`+0x24/+0x28`) with their previous values (`+0x38/+0x3c`), then stores the
current masks at `+0x38/+0x3c` and newly pressed bits at `+0x40/+0x44`.
Accordingly, `0x001044D0` reads the held/current mask at `+0x38`, while
`0x001047D0` reads the pressed-edge mask at `+0x40`. In `Mpl::_Control`, the
selected missile/special action `0x0c` is queried through `0x001047D0`, and the
gun action `0x0d` through `0x001044D0`. The native controller now fires the
selected missile or special weapon on a press edge, while gun fire remains held
and repeats at its decoded cadence. A short gun tap is preserved as one shot
when its press and release both arrive before a simulation tick. Physical key bindings remain native
platform policy; the logical action behavior is grounded in the generated code.
The native render loop can sample input more frequently than its fixed 60 Hz
simulation. SDL button transitions are latched through sampling, and the app
queues primary-fire, gun-fire, and weapon-switch press edges until simulation
ticks consume them. This prevents short taps from disappearing when an event
poll ends with the button already released or when a frame advances no
simulation tick. Overlapping keyboard/gamepad bindings produce combined
logical edges, so releasing one device does not release the action while the
other remains held.

The generated `Mpl::_Control` firing branch reads the 16-bit ammunition value
at `Mpl +0x1B1E`, compares it with `0x2710` (10,000), and skips the decrement
for that sentinel. The native controller names this as
`unlimited_ammunition_sentinel` and preserves the no-decrement behavior.

The local F-4G PAC member (member 252 in `config/pac_names.txt`) confirms the
container contract: slot 7 is the 48-byte `f04g.wxb` loadout, slot 8 is the
80-byte `gun2.wpb`, and slot 9 is the 80-byte `mis.wpb`. The loadout decoder
only reads through byte `0x21`; its former 64-byte minimum incorrectly rejected
this real member. The minimum is now `0x22`. `ac5_model_info --weapons` was
checked against an extracted copy and reports 800 gun rounds, 64 missiles, 10
special rounds (weapon 11), a 0.15-second gun interval, and a 1,000 km/h
missile launch speed. Since an `.mpl` contains the model rather than the full
aircraft member, `ac5_native` now accepts `--weapon-rules` separately from
`--model` and `--flight-rules`.

The older loose-file map preview inputs `m01a.mti`/`.mtd`/`.mtc` form a 64×64
inspection grid with 875 tile records and four indexed mip levels. They remain
supported as an asset preview, but they are not the full mission terrain
layers. The decompressed M01A container has a 256×256 tile-index raster in
subfiles 5/6, a tile pool/palette in 7/8, and 513×513 height and color grids
assembled from overlapping 33×33 blocks. Native code now decodes those actual
mission layers directly from the standalone DATA archive format; the archive
container is handled by the native `DataArchive` reader, without PS2 runtime
code.

The generated `runtime/src/rn/rn_vp_terrain.c` adapter is additional behavioral
evidence for the native terrain path. It receives terrain patch records with
either four coarse vertices or ten records for a 3×3 patch whose five edge
records can morph toward neighbor heights. The dependency-free
`morph_ac5_terrain_patch` helper now ports the ten-record path's edge indices,
neighbor pairs, clip-depth thresholds, and linear blend. It preserves the
source behavior before the near threshold and at or beyond the far threshold;
an intermediate record blends toward its two-neighbor average. The helper
accepts already-built positions and clip-W values, so MII decoding, patch
selection, camera matrix generation, and the four-record coarse path remain
separate. Each input height is read from the
second 32-bit lane as signed integer bits, converted with zero fractional shift,
and multiplied by `80.0f`; it is not a normalized byte converted directly to
world units. Patch origin is added to fixed local X/Z templates, and UV offsets
are applied separately. The ten-record path also uses the terrain depth range
to blend edge heights toward adjacent samples. This is enough to preserve the
observed patch-height units and LOD seam behavior in a portable renderer, but
the `.mii` stream and runtime code that selects patches and populates those
lanes still need to be decoded before the native port can reconstruct a
complete mission terrain. The native `decode_ac5_terrain_height_lanes`
helper now applies this signed-integer ×80 conversion transactionally and
checks float range before handing elevations to `build_heightfield_mesh`.
The earlier 32-byte-row interpretation of standalone `.mhd` files was
incorrect: matching file lengths to 32-byte rows and a 16-byte tail was only
arithmetic coincidence. The DATA.PAC name table identifies map subfiles 0/1
as `.mha` selector grid and `.mhd` height-block pool. M01A's `.mha` is 256
bytes, and its 11 selected blocks occupy 11×1089 bytes plus five trailing
bytes in the 11,984-byte `.mhd`; extraction from DATA.PAC member 3 produced a
byte-identical `.mhd` to the existing local asset. Each 33×33 block overlaps
its neighbors by one row and column, yielding a 513×513 grid. The native
`decode_ac5_mha_mhd_heightfield` assembles it and applies the observed ×80
height scale; `--heightmap` accepts either filename and loads its companion.
World spacing is 1,024 units, with vertex 256 at the map origin.

The former claim that `.mii` might be a height-validity mask was also wrong.
Map files 2/3/4 are `.mia` selector grid, `.mii` 33×33 color-index blocks, and
`.mid` RGBA palette. `decode_ac5_mission_terrain` now expands that registered
color layer into per-vertex RGBA8 using the GS 0..128 full-scale convention,
and the Vulkan mission mesh applies it as a vertex tint. The mission mesh now
uses generated parity-selected diagonals. The native mission query now handles
the regular path, the `0x400` even-sample path, and the `0x800` zero-height
class from the generated height routine.
The native viewer builds stride-1, 2, 4, and 8 full-extent heightfield meshes
through `build_heightfield_mesh_lod` and selects among them by camera distance.
`sample_heightfield_surface` now queries the same triangle diagonal and
piecewise-planar interpolation used by those meshes, providing a renderer-free
terrain elevation boundary for future native flight and ground systems. It
does not claim mission alignment, collision response, or recovered terrain
spacing; those inputs still need to come from decoded mission data.
These are native renderer LOD levels, not a claim to reproduce the retail
terrain patch scheduler; the generated routine's 4/10-record patch choice and
edge morphing still need mission patch topology and neighbor data.

The mission `.mtd` header stores a tile count, a 32-byte data offset, and
offsets for three following mip levels. Mission 01 confirms 875 indexed
64-by-64 tiles followed by matching 32-by-32, 16-by-16, and 8-by-8 levels;
the 1,024-byte `.mtc` companion is a 256-entry RGBA32 GS palette. `.mti` files
contain 64-entry rows of 16-bit words whose low 10 bits reference those tiles.
`decode_ac5_map_texture_set` validates and decodes the level-zero records into
a 2,304-pixel-wide atlas with four-pixel clamped gutters around each tile,
plus the original `.mti` words. `--map-textures` displays those tile indices as
a flat preview grid; its cell scale is for inspection only. Height-query flags
`0x400` and `0x800` are now decoded; the meanings of the remaining four high
bits are still open. The decompressed mission
container has a separate 256-by-256 tile-index raster in files 5/6, alongside
height block grid/data in files 0/1 and the tile pool/palette in files 7/8.
`decode_ac5_mission_terrain` now assembles these into a 513-by-513 heightfield
and a matching 256-by-256 map with decoded atlas. Block overlap is one sample
per 33-by-33 height block; spacing is 1,024 world units and the 513-sample
grid is centered at vertex (256, 256). Tile cells are 2,048 world units, so
the tile raster covers the heightfield at two height cells per tile. This
establishes mission height/index alignment without the PS2 runtime. The native
`build_ac5_mission_terrain_mesh` emits atlas UVs per tile, continuous
height-derived normals, alternates cell diagonals by the generated query's
XOR parity, and uses the recovered `0x800` tile flag to place zero-height
terrain. The separate `.mii` vertex color layer is decoded and
applied as a tint. The mission object also retains the raw `.mha` selector grid
and `.mhd` pool because the generated query addresses height samples within
selected 33-by-33 blocks, including even-coordinate samples for `0x400` tiles.
Its per-cell class comes separately from `.mta` selecting a 512-byte `.mti`
block and the local 16-by-16 tile coordinate selecting a 16-bit entry; the
native `surface.map_entries` stores that assembled 256-by-256 class raster.
`sample_ac5_mission_terrain_height` uses the selected raw height block at
`selector * 0x441`, matching the generated reader's byte addressing. Regular
cells use the 1,024-unit parity-selected triangles. Flag `0x400` selects the
separate 2,048-unit triangle path, reads the even local height vertices at
stride two, and selects its diagonal with the local x/z offset comparison.
Flag `0x800` returns zero. Out-of-range positions follow the source's edge
bands: negative coordinates wrap within the first 32 cells, and coordinates
at or beyond the positive bound wrap within the last 32 cells. This replaces
the earlier full-grid wrapping interpretation. Other tile classes and retail
patch scheduling/edge morphing remain open.
`ac5_native --mission-terrain DATA.TBL DATA.PAC <member>` now exposes this
asset path in the Vulkan viewer; the ordinary demo and extracted-file modes
remain independently selectable.

Mode-8 mission waypoint decoding now requires its secondary keyframed-path
header and complete offset/key records. A missing path is a decode error, and
the runtime steering/order-step APIs report a missing path as unsupported;
they no longer treat an absent path as immediate waypoint completion. This
keeps malformed or incomplete mission data from silently skipping scripted
movement.

The native mission API also distinguishes the spawn trigger kind at payload
`+0x38` from the placement mode at `+0x3c`. Absolute type-0 placements now
resolve position, pitch/yaw, optional terrain height from flag `+0x3d` bit 0,
and the caller-supplied model-bottom offset. Placement mode 1 adds the order
offset to either its fixed far-corner target or an explicitly supplied live
group/member position. Placement mode 2 adds the source's vertical/rearward
offset after rotating it by target yaw plus the payload angle; its output yaw
adds the payload yaw to target yaw and folds once into `(-pi, pi]`.

The group member transform path is now narrowed to `0x179bb8`. It iterates
members at the runtime `0x410` stride, reads a six-float local transform
(position followed by Euler angles in degrees) from the member's linked object
data, and writes the composed world position and attitude to `+0x1f0` and
`+0x200`. The generated path delegates composition to `0x388ad0` and
`0x32a1f0`. The remaining mapping is how that linked six-float source record
is populated from the decoded MMB member's `+0x0c..+0x14` formation offset and
unit reference. The viewer still uses a shared leader transform for member
models until that mapping is established.

`0x197a20` provides another use of the same linked-record shape: byte `+0x18`
selects a member in a referenced group, its world position is subtracted from
the record's xyz, and the result is rotated by the record's three degree
angles before being added to that member's current position. This confirms a
member-relative transform variant exists, but does not yet prove that the MMB
formation vector is copied into this linked record or identify the group
reference established by the script/spawn path.

The archive-backed mission unit reader follows both observed slot-0 bundle
layers: the DATA.PAC mission-pack member's sparse slot, then the wrapped
mission-entry slot containing the unit table. Script parsing now mirrors
`tools/parse_mmb_scripts.py`: begin at the root slot of the member table, read
its children as acts, then each Act's children as orders. Additional sets are
found in the sparse bundle at member-record `+0x2c`; the bundle's child set
nodes are checked against their own header/table count and duplicate order
source offsets are removed. The previous native path added an unsupported set
level and interpreted nested order payloads as script objects, producing false
zero-position markers.

M01A confirms the corrected traversal: unit 2 / Act 0 / order 0 is read from
wrapper offset `0x2d40` (type byte `0x2d50`) with coordinates
`(-110000, 5000, 0)`, matching the generated order table. The native preview
also matches unit 0's initial spawn at `(-57344, 5000, 50688)`. Across M01A,
native decoding matches the runtime trace export's 167 orders, 21 spawns, and
50 waypoints; unit 0's 57 orders are recovered across four sets. The preview
resolves 11 statically available spawn markers and leaves 10 orders unresolved
because their placements require live group/member state. The order model and
CLI retain/show source offsets and raw order coordinates to keep parser and
transform discrepancies inspectable. A second takeoff mission, M06A, also
matches the generated export exactly at 619 orders, 61 spawns, and 145 waypoints
after its member-associated sets are included.

The generated free-flight path calls `airframe_hits` with six body-axis
extremes and treats any point at or below the sampled surface as a collision;
there is no speed or attitude threshold. The native core now exposes this
portable six-point calculation as `count_airframe_surface_hits`, together with
the native flight model's body-axis basis. It requires an explicit surface
sampler and six model extents, so it has no terrain, renderer, SDL, or legacy
runtime dependency. The native `.pmb` flight-rule decoder does not provide
those extents; generated `tools/export_ruleset.py` reads them from the tail of
an `AHM` payload nested in mission-entry bundle slot 1. Native
`load_ac5_aircraft_hit_extents` now mirrors that bounded lookup and the
`ac5_mission_info --hit-extents` command exposes it. The generated flight path
only performs the six-point query below `max(20, max(0, terrain) + 20)` world
units; an aircraft that does not hit is clamped back to that rolling floor.
The native mission-terrain flight viewer samples the root floor per fixed tick,
performs the six-point query only below it, and clamps clear ticks back to the
floor. It uses the explicit `--aircraft-entry-index` for AHM extents and
`--aircraft-start x y z` for reproducible world placement.
`advance_frame_until_contact` samples spatial contact before every fixed tick
and sets a ground-contact override without rewriting script contact bits. A
root within one unit of the floor updates the generated ground-contact latch;
the ground bit remains set through the original half-second miss-decay window
before clearing. A latched root uses the native ground acceleration law; the
reduced speed cap is evaluated at half scale while the unhalved speed tables
remain available to the control-axis update. Below the floor, airborne hits
stop the session, while a tick that began grounded clamps the plane. Runtime
exercise with extracted F-4G data and M01A member 3 confirmed that starting at
`(0, 0, 0)` displays `AIRFRAME CONTACT` and leaves the Vulkan process
responsive. The flight core ports the generated normal ground-trace alignment
and anchor snap behind renderer-independent marker and terrain callbacks. The
ACM loader derives geometry-less attachment labels from the native no-link
source word, matching the generated exporter's attachment rule; the viewer
samples I/K/J from those labels at the generated gear channels. Its terrain
segment query intersects the same checkerboard triangles used by the native
mission terrain mesh and classifies zero-height tiles as kind 2. The viewer
invokes the trace when mission terrain is loaded and the aircraft gear is fully
down and the marker poses differ between the half/full gear channels. Models
without all three markers or without gear-dependent marker travel retain the
root-height contact fallback. The viewer exposes `--aircraft-gear-down` for a
reproducible deployment start; `ac5_mission_info --terrain-query` reports both
the source terrain query and a vertical intersection with the rendered mesh.

Ground-trace evaluation stages attitude and position changes in a local flight
state and commits them only after all terrain callbacks and anchor validation
succeed. A missing or invalid callback result therefore cannot leave the
caller's aircraft partially aligned.
