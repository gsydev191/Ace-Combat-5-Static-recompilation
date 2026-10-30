# Generated Code and Porting Review

This is a review note for the source checkout. References to `generated/`,
`runtime/`, and `config/` describe read-only source material. The native
project does not build or link any of those files.

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
  without opening Vulkan. It now also prints the eight raw part-table words for
  every part, including records without geometry. An extracted F-4G bundle has
  62 part records but only 31 geometry parts; the other records include
  repeated upper bits in word 4 and `0xffffffff` chain sentinels. Those raw
  values give further reverse-engineering evidence. For placed-model files
  specifically, word 3 is used as a PM texture slot by the opt-in native
  material path; this interpretation is validated against M01A, M02A, and
  M03A but remains unverified for other bundle families.
- A Vulkan 1.3 renderer using SDL3, dynamic rendering, synchronization2,
  indexed textured meshes, push-constant transforms, per-image depth
  attachments, resize-aware swapchain recreation, and staging uploads that
  flush non-coherent host-visible memory when required by the selected device.
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
  the frame's flight ticks all succeed.
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
  game-specific output still requires decoded model/material data.

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

Player fire input is connected to the viewer when flight rules and aircraft
weapon slots are loaded. The recovered
`Mpl::_Control` pad table uses logical action `0x0c` for the selected missile or
special weapon and `0x0d` for the gun; these are configurable actions, so the
native Space/Ctrl bindings are platform policy rather than fixed retail button
equivalents. The gun uses the aircraft's slot-8 record for its `0.15 s` shot
interval, muzzle speed, spread, and gravity, and decrements the aircraft's gun
ammunition per shot. Missile firing depends on slot-9 data, two alternating
pylons, per-pylon reload timers, and an optional lock target. The standalone
`weapon_controller` module models weapon switching, gun cadence/ammunition,
alternating standard-missile pylons, per-pylon reload timers, and ordinary
special-weapon pylon events. It accepts portable rules and emits native shot
events without linking runtime code. The ACM loader decodes the aircraft
loadout/gun/missile slots, and `ac5_model_info --weapons` reports the decoded
values for extracted aircraft bundles. Weapon state advances on fixed flight
ticks, and the viewer reports ammunition and emitted event counts in its
window title. A bounded native projectile pool inherits the aircraft's actual
per-tick displacement velocity (including path lag and vertical sink), adds the
decoded launch speed along body forward, applies the weapon gravity value, and
renders through the existing dynamic-mesh path. Its 128 slots recycle shots
after a 12-second native presentation lifetime; that lifetime is not recovered
retail behavior. Pylon-specific launch offsets, missile homing, target
acquisition, collision, and damage remain open. Special-weapon ids `8` and
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
