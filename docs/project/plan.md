# Splat-and-Seek: PolarFire FPGA Design Contest 2026 (consolidated plan)

## Context
The two-person NMS team enters the **PolarFire FPGA Design Contest 2026, Track 2 (PolarFire SoC
Icicle Kit)** with a **new project built from the ground up**. It takes the 3DGS direction from
`explorations/gaussian_splatting/` and the NMS build habits (golden model first, self-checking
GHDL testbenches, numeric gates, deterministic latency, measured claims only). No NMS RTL is reused.
**Proposal deadline: 25 Oct 2026, 12:29 PM Sri Lanka time** (24 Oct, 11:59 PM PDT). The portal
needs an overview (≤300 words), innovation & impact (≤250 words), a block-diagram PDF on a public
drive, and the track.

**Title:** *Splat-and-Seek: A Few-Watt Gaussian Splatting Localizer for Robots in Radio-Silent Sites,
on PolarFire SoC*

## 1. The system
**Problem.** Robots inspecting hazardous, secure or GPS-denied sites must know where they are with
no reliable link. GPUs are power-hungry and radiation-fragile; lidar maps carry no visual record
for the human inspector.

**Flow.** Survey (handheld camera now, drone later) → base-station GPU builds a 3DGS map (COLMAP,
gsplat, scale from AprilTags, floor alignment, compaction, coarse LOD) → the map is loaded onto the
robot → the operator gives a goal (coordinates/waypoints; language in the Should tier) → the
**Kobuki localizes itself on board with no link** and drives.

**Core idea.** Monte Carlo localization (3-DoF) whose **measurement model is a 3DGS renderer in
PolarFire fabric**: per update, ~300 particles × tiny 32×24 renders of an LSRAM-resident local map
(~45 K Gaussians), each scored against the Kinect frame (gradient/census RGB + depth). Motion model:
Kobuki wheel + gyro odometry at 50 Hz. Precedent: **Loc-NeRF** (MCL with a neural map on a real
Jackal; map from a person-carried camera; 0.9–2.5 Hz on an RTX 5000 laptop GPU). Our contribution
is making that class real-time at a few watts, with the whole render pipeline (sorting included)
in fabric.

**Positioning.** Not a better lidar AMCL. A **camera-only, few-watt, map-based localizer whose map
is also the site's visual twin**: a basic camera drone can map, and the human inspector gets visual
cues. Lidar is a later extension through a pluggable measurement-model interface.

**Why PolarFire SoC** (each claim verified before use): low static power; flash-based,
SEU-immune configuration (radiation sites); design security (secure sites, if used in the demo,
otherwise stated as platform suitability); hard RISC-V Linux + AMP real-time core sharing DDR with
the fabric; a migration path to the MPFS095T (cost) or RT PolarFire (radiation) on one toolchain.

## 0. Business case (evidence; opens the proposal)
**Unifying problem: "return-to-site autonomy".** A site is surveyed once and then revisited by
robots that have **no GNSS, no reliable link, and a hard budget** (battery, radiation dose, or both).
They must localize against the prior map on board, quickly, on low-power compute that survives the
environment.
| segment | evidence | what it needs from us |
|---|---|---|
| **Nuclear decommissioning (beachhead)** | Fukushima Unit 2, Feb 2017: TEPCO's "scorpion" robot hit its radiation limit in ~2 h (one-fifth of plan, 210 Sv/h); a cleaning robot lost two cameras after ~2 h; robots abandoned in place. UK civil legacy clean-up ~£115 bn over 100 years, ~70% at Sellafield; Sellafield trialled Spot. Nuclear-robotics market ~US$1.6–2.3 bn, inspection the fastest-growing segment (vendor reports, directional) | **Mission time is dose**: a robot that knows where it is drives straight to the target instead of wandering. Few-watt compute; configuration immune to upsets; RT PolarFire path |
| **Space surface mobility (scale-up path)** | Perseverance's Lander Vision System matched descent images to an **onboard map**, accelerated by an **FPGA** coprocessor (the first FPGA accelerator on a NASA rover). Ingenuity scouted ahead of the rover (survey-then-traverse). **RT PolarFire SoC engineering samples available since Jul 2025** (Microchip) | The same core on radiation-tolerant PolarFire SoC: map-relative localization for rovers, with no GPS and delayed links |
| **Underground mining** | LKAB used a SLAM drone to inspect Kiruna after a 2020 seismic event; Luleå tested autonomous drones in a GPS-denied mine; vendors (Emesent, Inkonova) are lidar-SLAM based | Re-entry after an event against a prior map (map partly stale → robust likelihood); camera-only, lighter payload |
| **GNSS-denied drones** | Jamming/spoofing drives a drone-navigation market of ~US$2.4 bn (2025) → ~5.1 bn (2032) (market reports, directional) | Map-based visual localization; the 6-DoF mode is the extension |
**Pitch structure:** beachhead = nuclear inspection (strongest evidence); the same core scales to
space (Microchip's own RT PolarFire SoC) and mining; drones are the 6-DoF extension.
**Honesty notes:** commercial PolarFire is not total-dose rad-hard; RT PolarFire is the radiation
product. The Kinect and Kobuki are demo scaffolding, not radiation-capable. Market figures are
vendor estimates, quoted as directional.

**Review of the business case (CTO / evaluator / architect) → adopted**
1. **Turn "mission time is dose" into a measurement**: demo metric = time-to-goal and goal error,
   odometry-only vs with the localizer, over the same route.
2. **TRL framing**: the contest takes the core from TRL 3 to TRL 4 (validated in the lab, on a
   robot); RT PolarFire SoC is the later step, stated as a roadmap, not a claim.
3. **Operator assist**: nuclear operators often teleoperate. Localization against the twin is
   valuable even without autonomy (position on the visual map, return-to-point).
4. **Proposal focus**: one beachhead (nuclear inspection) + one sentence of scale paths (space via
   RT PolarFire SoC, mining). Civil framing; drones only as "extension".
5. **Radiation-aware design hooks** (cheap, architect's requirement trace): CRC on map load, ECC on
   the LSRAM map if the configuration supports it (verify), watchdog + ESS health stop, a bounded
   update latency (fits a safety case).

## 1a. The core is the product; everything else is demo scaffolding
**What we pitch, build and are judged on: the Splat-and-Seek core**, a pose-batched 3DGS
measurement engine plus the reference localizer firmware, on PolarFire SoC.
| part | contents | role |
|---|---|---|
| **Engine (fabric IP)** | Local-map store (LSRAM); shared-order sort once per update; projection units; tiny-tile blend (colour + expected depth); compare unit → per-pose log-likelihood; optional single-pose full render (debug/operator view) | **Core** |
| **Interfaces** | AXI4-Lite control/status; AXI4 or AXI4-Stream for map load, observation in, scores out; parameterized (projection units, max poses, resolution, cap) | **Core** |
| **Driver + API** | `load_map()`, `set_observation()`, `score_poses(poses[]) → scores[]`, `render(pose)`; Linux (UIO) and bare-metal variants | **Core** |
| **Reference localizer firmware** | MCL on the real-time core: predict from any odometry, batch to the engine, resample, health (ESS) | **Core** |
| **Golden model + verification** | Bit-exact Python model, vectors, self-checking testbenches, numeric gates | **Core** (evidence) |
| Robot, Kinect, odometry source, host map pipeline, path planning, language, UI | Proves the core works in a real loop | **Scaffolding** |
**Core metrics (the headline):** pose-scores per second, update rate at N poses, worst-case update
latency (bounded), joules per update (vs GPU), resource use, localization accuracy produced by the
reference firmware, bit-exactness.
**Scaffolding rule:** the cheapest reliable path, reused off the shelf, never on the critical path of
the core. It must be credible (real sensor data, a real robot) but isn't itself a deliverable.

## 1b. Engine architecture (refined in the final review)
- **Dataflow: Gaussian broadcast → pose lanes.** The engine streams the local map from LSRAM in
  one shared depth order; each Gaussian is broadcast to P pose lanes. Each lane holds one pose and
  its tiny 32×24 colour + depth buffer, and does projection + blend for its pose. Poses are processed
  in groups of P; the map is read N/P times per update (cheap from LSRAM).
- **3-DoF fast path.** Ground-robot poses are (x, y, θ) with known camera height, pitch and roll,
  so the view transform is a rotation about the vertical axis plus a translation. Per-Gaussian
  terms are precomputed in the gravity frame, which sharply cuts per-lane multipliers → more lanes
  per math block. 6-DoF remains a parameter (drone, Stretch).
- **Pre-cull + stream compaction.** A cheap frustum/depth test runs first; only visible Gaussians
  enter the heavy projection pipeline (LENS removed 24–60% this way), raising effective throughput.
- **Two MCL modes.** *Tracking* (particles within ~1 m): one shared order per update. *Global /
  re-localization* (particles spread over the map): cluster the particles and use one order per
  cluster, or the coarse LOD. The shared-order approximation error is measured per mode in the CP1 sweep.
- The likelihood (census/gradient + depth) is computed per lane on the 768-pixel buffer; the
  observation's census/gradient is computed once per frame.

## 2. Board vs host
| where | runs |
|---|---|
| **Host PC (GPU)** | COLMAP, gsplat training, scale/floor alignment, compaction, LOD, fixed-point map file; goal entry and path planning → waypoints; (Should) detector + local LLM → goal; operator UI; recorded-log player; benchmark harness |
| **Icicle, Linux (U54)** | Map load into LPDDR4; UDP server (goal/waypoints in, pose/telemetry out); **Kinect via libfreenect** → DDR frame buffers; fabric driver (UIO, DMA); local-map selection/paging into LSRAM; logging |
| **Icicle, real-time core (AMP)** | MCL loop: odometry predict (50 Hz), particle batches to the fabric, weights, resampling; waypoint following; motor commands; bump/cliff stop |
| **Icicle fabric** | Frame input (AXI4-Stream video, MIPI-ready); projection; binning; tile-local sort; blend (colour + expected depth); compare unit → per-particle score; LSRAM local map |
| **Kobuki + Kinect** | Differential drive, encoders + gyro (50 Hz), bumpers/cliff/wheel-drop; Kinect RGB-D 640×480 at 30 fps, powered from the Kobuki's 12 V Kinect port; the Icicle from the 12 V/5 A port; Kobuki ↔ Icicle over the expansion-port UART |
Link: map once (~13 MB), goal/waypoints (bytes), telemetry (~1 KB/s). Localization never crosses it.

**Scaffold staging (decided)**
- **Scaffold v1 (Must, walking skeleton): a Raspberry Pi (or Jetson) on the Kobuki** runs the
  Kinect driver (libfreenect) and the Kobuki driver (`kobuki_core`/own parser), and streams the
  downsampled observation + odometry over GbE/UDP to the Icicle. The Icicle (engine + MCL firmware)
  returns the pose, and the companion drives. Its Wi-Fi is the operator link. Power: a 12→5 V/3 A
  buck from the Kobuki 12 V/5 A port (the Kobuki 5 V/1 A port is too weak for a Pi 4/5), or a
  power bank.
- **Pi or Jetson?** A Pi is clearly only a harness: it can't run the workload, so "why not run it
  on the companion?" doesn't arise. A Jetson on the same robot enables a **head-to-head baseline**
  (the same pose-batched MCL in CUDA: updates/s, J/update, latency jitter), the strongest possible
  answer to "why not a GPU?", but it invites the comparison live. Recommended: the Pi as the
  harness; the Jetson (if available) only as the measured baseline.
- **Availability is uncertain (one of the two is guaranteed), so the scaffold is
  companion-agnostic**: a plain Linux + libfreenect + Kobuki driver + UDP sender that runs on
  either. The GPU baseline always runs on the team PC; a Jetson baseline is added only if a Jetson
  turns up.
- **Scaffold v2 (Should): Icicle-only**: Kinect via Linux on the Icicle, Kobuki over the
  expansion-port UART to the real-time core, laptop removed. Same core and API; only the
  observation/odometry source changes.
The table above describes v2. In v1 the "Kinect via libfreenect" and Kobuki rows run on the laptop.

## 3. Numbers (estimates until measured)
- **Localization:** projection-bound. 300 × 45 K = 13.5 M projections/update; 4 projection units
  at 150 MHz ≈ **40 Hz**, 2 units ≈ 22 Hz. Blending at 32×24 is negligible. DDR is out of the loop.
- **Operator view (Should):** blend-bound from DDR, ≈ 22 fps at 720p if fabric↔DDR ≈ 2.5 GB/s.
- **Resources (MPFS250T):** 254 K LE, 784 math blocks, ~2.2 MB LSRAM. Binding: math blocks and LSRAM.
- **Targets to promise:** ≥ 20 Hz localization update; compute-module power ≤ 5 W (measured as
  delta); a measured worst-case update latency (per-tile cap); accuracy per the CP1 gate; ~100×
  faster than the same pipeline on the board's own RISC-V cores.

## 4. Prior work
LENS (arXiv 2610.25068, KR260, 3.5–4 W): render + SCR localization in fabric; **sort/binning on the
ARM, excluded from latency**; no closed loop. Loc-NeRF (MCL with NeRF, ~1–2.5 Hz on a GPU).
GSCore (ASPLOS'24), Neo (ASPLOS'26, frame-coherent sort, cited, not claimed), FAMERS (DATE'25),
129-fps ASIC (arXiv 2604.10223), Splat-Nav / ATLAS Navigator (GPU). Industry: lidar AMCL (incumbent),
cuVSLAM/Slamcore on Jetson, the discontinued Intel T265. Nothing on PolarFire found.

## 5. Tiers
- **Must (core):** golden model + CP1 gate; fabric engine (map store, shared-order sort, projection,
  blend with depth, compare); driver + API; MCL firmware on the real-time core; bit-exact
  verification; core metrics (rate, latency bound, J/update vs GPU, resources, accuracy, vs RISC-V).
- **Must (demo scaffold v1):** laptop-on-Kobuki feeding the Icicle over GbE; recorded-log replay
  through the board (accuracy testbed + fallback demo); live localization and goal driving.
- **Should:** scaffold v2 (Icicle-only: Kinect on Linux, Kobuki UART); language goal (detector +
  local LLM + planner); 720p operator view; map paging; renderer-only comparison at LENS's setting.
- **Stretch:** drone survey; MIPI camera; LENS-style SCR fallback on VectorBlox; Neo-style sort
  reuse; 2D-lidar measurement model; LangSplat.
- **Fallback (decided at CP1, not later):** Kobuki odometry + render-based correction at a lower
  rate (no visual odometry).

## 6. Phases and checkpoints (base case: kit arrives ~21 Dec)
| phase | dates | lead | content | exit gate |
|---|---|---|---|---|
| P0 Proposal | 24–25 Oct | A+B | Texts, diagram, submit | Submitted by ~10:00 SLT 25 Oct |
| P1 Groundwork (no board) | 26 Oct–28 Nov | A+B | Repo, toolchain, golden model, maps, MCL study, Kobuki + Kinect on a laptop | **CP1 28 Nov** |
| P2 RTL (no board needed) | 30 Nov–31 Jan | A | Blocks + testbenches + synthesis | **CP3 23 Jan** |
| P3 Bring-up (on kit arrival) | ~21 Dec–10 Jan | B | Board, DMA, Kinect, Kobuki UART, AMP | **CP2 ~10 Jan** |
| P4 Integration on board | 1–7 Feb | A+B | Renderer on board, bit-exact | **CP4 ~7 Feb** |
| P5 Localization on the robot | 8 Feb–7 Mar | B (+A) | MCL loop, replay HIL, live runs | **CP5 28 Feb** |
| P6 System + Should items | 1–14 Mar | B | Goal driving; language, operator view if on time | **CP6 14 Mar freeze** |
| P7 Evaluation + submission | 8–25 Mar | A+B | Benchmarks, report, README, video | **Submit 25 Mar** |

**Numeric gates**
- CP1-tool (2 Nov): Libero licence works; reference-design bitstream + Linux image built locally.
- CP1a (9 Nov): stage-1 map trained; fixed-point render ≥ 35 dB vs float.
- CP1b (21 Nov): MCL sweep done (particles × resolution × LOD × likelihood).
- **CP1 (28 Nov):** OpenLORIS-Scene median ≤ 0.20 m / ≤ 5°, ≥ 90% of frames tracked; own data
  ≤ 0.25 m at AprilTag checkpoints; at a compute budget that projects to ≥ 20 Hz. Fail → fallback.
- CP2 (~10 Jan): bandwidth measured; AMP mailbox; **walking skeleton runs**: the Kobuki drives on
  poses from MCL on the Icicle (software renderer, any rate).
- CP3 (23 Jan): math blocks ≤ 85%, Fmax ≥ 125 MHz for every block.
- CP4 (~7 Feb): one render byte-identical to the golden model; ≥ 10 Hz localization batch.
- CP5 (28 Feb): board scores bit-exact vs software MCL; replay error ≤ 1.2× software; live run
  passes the checkpoints.
- CP6 (14 Mar): feature freeze; untested features are cut and documented.

**Reconsider triggers**
| trigger | action |
|---|---|
| CP1 fails | Fallback becomes primary; render-and-compare at a lower rate for drift correction |
| Kit not here by 14 Dec | P2 continues on GHDL + Libero; P3 compresses to ~1.5 weeks on arrival |
| Scaffold v2 (Kinect on the Icicle) fails | Stay on scaffold v1 (laptop); no effect on the core |
| Math > 85% or Fmax < 125 MHz at CP3 | Fewer projection units/particles; accept 125 MHz |
| CP4 slips > 1 week | P6 shrinks to goal driving only |
| Behind at CP6 | Cut Stretch, then Should; the Must path is the demo |

## 7. Atomic tasks
A = RTL lead, B = systems lead (names assigned by the team). d = working days.

**P0 Proposal**
| id | task | owner | d |
|---|---|---|---|
| 0.1 | Verify PolarFire claims and the Libero licence; drop any claim without a source | B | 0.2 |
| 0.2 | Overview ≤ 300 words | A | 0.3 |
| 0.3 | Innovation & impact ≤ 250 words (LENS, Loc-NeRF, lidar AMCL positioning) | B | 0.3 |
| 0.4 | Block diagram + differentiation table + targets → PDF | A | 0.4 |
| 0.5 | Rubric pass (both); upload; private-window link test; submit; screenshot | A+B | 0.3 |

**P1 Groundwork**
| id | task | owner | d |
|---|---|---|---|
| 1.1 | New repo skeleton (uv, ruff, Makefile, GHDL, CI, CLAUDE.md, skills) | B | 1 |
| 1.2 | Toolchain end to end: licence → reference-design bitstream → HSS + Linux image (CP1-tool) | A+B | 3 |
| 1.3 | Kobuki bring-up on a laptop: serial parser, odometry log, teleop, battery runtime | B | 2 |
| 1.4 | Kinect via libfreenect on a laptop; intrinsics/extrinsics calibration; RGB-D + odometry logger | A | 2 |
| 1.5 | Site prep: AprilTags at measured positions, taped floor marks | A | 1 |
| 1.6 | Surveys: stage 1 (Kinect handheld, two heights), stage 2 (phone, two heights); robot test traversals | A | 2 |
| 1.7 | Map pipeline: COLMAP → gsplat → tag scale → floor alignment → compaction → LOD | B | 4 |
| 1.8 | Float reference renderer (from the exploration code), with depth output | A | 1 |
| 1.9 | Fixed-point golden model (projection, binning + cap, tile sort, blend + depth, compare) → widths, tile size, cap | A | 6 |
| 1.10 | Software MCL: odometry motion model; gradient/census RGB + depth likelihoods | B | 5 |
| 1.11 | MCL sweep on OpenLORIS-Scene + own data → particles, resolution, LOD, likelihood (CP1b) | B | 3 |
| 1.12 | Rate model from the golden model's real counts | A | 1 |
| 1.13 | Frozen spec v1: map record, widths, interfaces (normative, checked against Python/VHDL) | A | 1 |

**P2 RTL**
| id | task | owner | d |
|---|---|---|---|
| 2.1 | RTL interface spec (AXI-Stream formats) | A | 1 |
| 2.2 | Per-block vector generator | B | 2 |
| 2.3 | Projection unit + tb (bit-exact) | A | 5 |
| 2.4 | Binning unit (with cap) + tb | B | 4 |
| 2.5 | Tile-local sort (LSRAM) + tb | B | 4 |
| 2.6 | Blend array (LUT exp, early termination, depth accumulator) + tb | A | 6 |
| 2.7 | Compare unit (gradient/census + depth) + per-particle score + tb | B | 3 |
| 2.8 | Frame input (AXI4-Stream video from DDR) + tb | B | 2 |
| 2.9 | Per-block Libero synthesis table (CP3a 16 Jan, CP3 23 Jan) | A | ongoing |
| 2.10 | Core integration + full-pipeline tb vs golden | A | 4 |
| 2.11 | RISC-V software baseline of the pipeline (C, 4 cores) | B | 3 |

**P3 Bring-up**
| id | task | owner | d |
|---|---|---|---|
| 3.1 | Boot, console, network; reference design rebuilt from repo Tcl | B | 2 |
| 3.2 | Fabric ↔ LPDDR4 and LSRAM throughput | A | 2 |
| 3.3 | UIO driver + buffers + IRQ; toy accelerator round trip | B | 3 |
| 3.4 | Observation/odometry UDP protocol (laptop → Icicle), timestamps from one clock | B | 1 |
| 3.5 | **Walking skeleton (scaffold v1)**: laptop (Kinect + Kobuki) → Icicle MCL with a *software* renderer → pose → Kobuki drives | B | 4 |
| 3.6 | AMP real-time core + shared-memory mailbox; non-cached/flushed DMA buffers | B | 2 |
| 3.7 | Power from the Kobuki (12 V/5 A), draw measured; power-measurement method | B | 1 |
| (P6, Should) | Scaffold v2: Kinect on the Icicle (libfreenect, RISC-V), Kobuki UART to the real-time core | B | 4 |

**P4 Integration**
| id | task | owner | d |
|---|---|---|---|
| 4.1 | Core + DMA + UIO on board; byte-identical render | A+B | 3 |
| 4.2 | Batch scheduler (particles per update, local map in LSRAM) | A | 3 |

**P5 Localization on the robot**
| id | task | owner | d |
|---|---|---|---|
| 5.1 | MCL loop on the real-time core | B | 4 |
| 5.2 | Recorded-log replay harness (PC → board → pose → error vs GT) | B | 3 |
| 5.3 | Worst-case update latency (fabric timer, cap-saturating scenes) | A | 2 |
| 5.4 | Bit-exact scores vs software MCL; replay accuracy | B | 2 |
| 5.5 | Live runs: stationary → driving; checkpoint errors; video capture | A+B | 4 |

**P6 System**
| id | task | owner | d |
|---|---|---|---|
| 6.1 | Occupancy grid from the map + A* → waypoints; waypoint following + depth-based local stop | B | 3 |
| 6.2 | (Should) Detector on survey frames → 3D landmarks; local LLM → goal | B | 3.5 |
| 6.3 | (Should) 720p operator view to the PC | A | 2 |

**P7 Evaluation + submission**
| id | task | owner | d |
|---|---|---|---|
| 7.1 | Benchmarks (rate, latency bound, power, accuracy, vs RISC-V, vs Loc-NeRF/LENS figures) | A | 4 |
| 7.2 | Report ≤ 10 pages | A+B | 5 |
| 7.3 | README, clean-clone build check | B | 2 |
| 7.4 | Video 3–5 min (assets captured at every checkpoint since CP1) | A+B | 4 |
| 7.5 | Third-party IP/licences (gsplat Apache-2.0, COLMAP, libfreenect, datasets) + AI disclosure | B | 1 |
| 7.6 | Upload, link test, submit by 25 Mar | B | 0.5 |

Weekly: a 30-minute review against this table; re-plan at every gate. Capacity ≈ 100% (no buffer
beyond cutting Should).

## 8. Risks (for the feasibility section)
| risk | mitigation |
|---|---|
| MCL accuracy across traversals, cameras and heights | CP1 in software, two-height survey, gradient/census + depth likelihood, staged cameras |
| Toolchain/licence (NMS lost 18 days to one) | End-to-end build in P1 week 1 (CP1-tool) |
| Kit arrives late | RTL and synthesis need no board; P3 compresses |
| Kinect on the Icicle's USB | CP2 gate; webcam fallback |
| Math blocks / timing | Model-fixed widths, LUT exp, fewer units, 125 MHz |
| Legacy Kobuki (battery, support) | Documented protocol, own parser; battery checked in P1 |
| Over-claiming | Promise Must only; "few-watt" = compute module, measured |

## 9. Open items (not decided)
- Goals to the driving robot: solved for scaffold v1 (laptop Wi-Fi); for v2, a Wi-Fi bridge or preloaded goals.
- **Ground truth on own data**: AprilTag checkpoints only (the current CP1 wording), or add pseudo-GT by
  registering test frames to the map offline.
- RT PolarFire SoC availability (for the migration claim); VectorBlox on the Icicle (Stretch only).

## 9a. Review (evaluator, architect, CTO) and resulting changes
Scores: proposal as an evaluator would see it ≈ **7.7/10** (F 7.5, I 8.5, C 7, Im 7.5); final stage
if the Must tier is delivered ≈ 7.5–8.5. Changes adopted into the plan:
1. **The block-diagram PDF carries the feasibility story** (550 words can't): gate timeline strip,
   board/host split, Microchip IP list, targets, differentiation table.
2. **Name the Microchip IP explicitly**: PolarFire SoC MSS (U54 + E51), FIC0/FIC1, CoreAXI4Interconnect,
   CoreAXI4DMAController or the MSS PDMA, LSRAM/µSRAM, math blocks, MMUART, HSS, Linux BSP,
   (Stretch) CSI-2 RX, VectorBlox.
3. **Architecture reframed: pose-batched rendering.** Particles lie within ~1 m of each other, so
   the depth order is almost identical across them: **sort once per update, stream that order
   through all ~300 poses** (cross-particle coherence, the across-pose analogue of Neo's
   across-frame reuse). The sort cost is amortized ~300×, and projection is the unit to optimize.
   The approximation error of a shared order vs exact per-particle sorting becomes a CP1 sweep
   axis. The contribution claim becomes "one Gaussian stream, hundreds of candidate poses, in
   fabric", which is more original than "sort in fabric".
4. **Walking skeleton by CP2 (~10 Jan)**: Kinect → Linux → MCL on the real-time core with a
   *software* renderer at a low rate → Kobuki drives. Then swap the fabric renderer in (CP4). This
   removes the February big-bang integration, makes the fallback real, and gives demo footage
   early. New P3 task 3.8 (B, 4 d).
5. **System-level details added to P3/P5**: one shared hardware timer timestamps odometry (RT core)
   and frames (Linux); DMA buffers non-cached or explicitly flushed (FIC coherency); identical
   deterministic PRNG in Python and C so MCL replays bit-exactly; an effective-sample-size health
   monitor that stops the robot when localization degrades; a Kinect depth-noise model (σ ∝ z²,
   holes masked); double-buffered LSRAM regions for map paging (Should).
6. **GPU energy baseline**: the same pose-batched MCL on the team's PC GPU (gsplat batch render),
   with GPU power from `nvidia-smi` → joules per update vs the FPGA's. This answers "why not a
   GPU?" with a measurement, not an argument. New task 7.1b (B, 3 d).
7. **AI-tool rule (contest §13)**: disclose tool and use; proposal texts in the team's own words;
   every claim engineering-validated (gates, testbenches).
8. **Power story told honestly**: the robot (Kinect ~12 W, motors) isn't few-watt; the compute
   module is. Pitch the platforms where compute power dominates: small crawlers, drones, sensor heads.
9. **IP note (contest §11)**: Microchip receives a perpetual royalty-free licence to submitted
   materials. The team keeps ownership but not exclusivity; matters only if this becomes a product.

## 9b. How the proposal is written: CTO first, then evaluator, then architect
**Feasibility means: can this be completed with the given kit within the given time** (not team
experience). Evidence used: resource fit on the MPFS250T (math blocks, LSRAM, Fmax), the kit's
peripherals actually used (MSS U54 Linux + real-time core, fabric, LSRAM, math blocks, GbE,
LPDDR4), a 17-week gated timeline with fallbacks, and risks with mitigations.

**Order of persuasion**
1. **CTO (value first).** Overview opens with who needs it and what it replaces: robots in
   radio-silent hazardous/secure sites need on-board localization; GPUs are power-hungry and
   radiation-fragile, lidar maps carry no visual record, the Intel T265 class of localization
   modules is gone. Then the product: a localization core for PolarFire SoC, its measurable
   benefit (updates/s, J/update, bounded latency) and the device path (MPFS095T cost-down, RT
   PolarFire for radiation).
2. **Evaluator (rubric).** Each criterion visibly answered: problem → solution → kit use →
   expected outcome with numeric targets (overview); novelty, impact, scalability,
   commercialization (innovation); feasibility in the PDF (resource table, gate timeline, risks).
3. **Architect (depth, mostly in the PDF).** Block diagram (board/host, scaffold), engine dataflow
   (Gaussian broadcast → pose lanes, shared order, 3-DoF fast path, pre-cull), resource budget,
   Microchip IP list, verification method.

**Portal field → content**
| field | limit | content (in this order) |
|---|---|---|
| Project overview | 300 words | Problem and who has it (CTO) → the core and how it works in one plain sentence → target application (inspection robots; demo on a Kobuki) → expected outcomes as numeric targets |
| Innovation & impact | 250 words | What is new (pose-batched splat engine, first 3DGS localizer in PolarFire fabric; vs LENS, Loc-NeRF, lidar AMCL) → impact (link-free autonomy, few-watt compute, visual twin) → scalability and commercialization (MPFS095T/RT PolarFire, lidar extension) |
| Block-diagram PDF | — | p1 system diagram (host, Icicle MSS/fabric, scaffold); p2 engine dataflow + resource budget + IP list; p3 gate timeline + risks + targets + differentiation table |
| Track | — | Track 2: PolarFire SoC Icicle Kit |

## 9c. Reference proposal text (for the team to rewrite in its own words)

**Title:** Splat-and-Seek: A Few-Watt Gaussian Splatting Localizer for Robots in Radio-Silent
Sites, on PolarFire SoC

**Development platform:** PolarFire SoC Icicle Kit (Track 2)

### Project Overview
<!-- OVERVIEW START -->
**Problem.** Robots sent into nuclear plants, mines and other GPS-denied sites must know where they
are without a reliable radio link, on a hard battery and radiation budget. At Fukushima Daiichi
in 2017, a robot reached its radiation limit within about two hours. Every minute spent working out
where it is shortens the inspection. Current options need a power-hungry GPU or build lidar maps
with no visual record for the operator.

**Solution.** Splat-and-Seek is a localization core for PolarFire SoC. A site is surveyed once with
an ordinary camera and turned into a 3D Gaussian Splatting map: a photorealistic 3D model that also
serves as the inspection record. On the robot, the core answers one question hundreds of times per
update: if the robot stood here, what would its camera see, and how well does that match the real
image? A custom fabric engine renders tiny views of the local map, held in on-chip memory, for up to
300 candidate poses and scores each against the camera frame. The real-time RISC-V core runs a
particle filter over those scores, fused with wheel odometry, while Linux handles maps and
networking. The engine is sized for the MPFS250T: about 45,000 Gaussians in on-chip RAM and four
pose lanes in under half of its math blocks.

**Target application.** Return-to-site inspection robots in hazardous, secure or GPS-denied
facilities. We demonstrate the core on a Kobuki ground robot with a Kinect RGB-D camera; a small
companion computer only forwards sensor data.

**Expected outcome.** An Icicle Kit localizer with at least 20 pose updates per second, a
measured worst-case latency bound, under 5 W of compute power (joules per update compared against a
GPU running the same algorithm), median error within 0.20 m and 5°, and bit-exact
agreement with a Python reference model.
<!-- OVERVIEW END -->

### Innovation & Expected Impact
<!-- INNOVATION START -->
**Novelty.** Gaussian Splatting accelerators such as GSCore and Neo are ASIC designs for
drawing images, and the closest FPGA work, LENS (2026), still sorts on the CPU. Loc-NeRF showed that
a particle filter with a neural map can localize a real robot, but at one to two updates per second
on a laptop GPU. Splat-and-Seek turns rendering into a measurement engine: one depth-sorted stream
of Gaussians is broadcast to parallel pose lanes, so every candidate pose shares a single sort, and a
ground-robot fast path cuts the projection arithmetic. To our knowledge it is the first Gaussian
Splatting localizer in PolarFire fabric, using math blocks and LSRAM for the engine, the RISC-V
real-time core for the filter and Linux for I/O.

**Differentiators.** Camera-only mapping (a basic camera drone can survey), a map that doubles
as a visual inspection record, few-watt compute, deterministic latency, and no cloud or radio link
once deployed. Lidar can later plug into the same interface.

**Impact.** In nuclear decommissioning, a robot that knows where it is drives straight to its target
and spends its radiation budget on inspection instead of search. The UK expects its civil nuclear
clean-up to cost about £115 billion over 100 years; mining re-entry after seismic events has the same
need.

**Scalability.** The design targets the PolarFire SoC family: a lower-cost MPFS095T version for
volume, and Microchip's radiation-tolerant RT PolarFire SoC for harsher environments, including
planetary rovers, where Perseverance already used an FPGA for map-based visual navigation.
<!-- INNOVATION END -->

**Notes for the authors (not submitted)**
- Rewrite in your own words; keep a record of AI-tool use for the final disclosure (rules §13).
- Claim sources: Fukushima (Khaosod/AP, Feb 2017); £115 bn (NDA via NIST slide); LENS (arXiv
  2610.25068); Loc-NeRF (arXiv 2209.09050); GSCore (ASPLOS'24); Neo (ASPLOS'26); Perseverance VCE
  (FPGA robotics survey, arXiv 2009.06034); RT PolarFire SoC samples (Microchip, Jul 2025).
- Numbers are targets until measured; "to our knowledge" stays unless a prior PolarFire 3DGS work
  turns up.

## 9d. Block diagrams (Excalidraw), next step
**Tooling.** Install the `excalidraw-diagram` skill (coleam00/excalidraw-diagram-skill) project-scoped
into `~/UoM/HDL/splat-and-seek/.claude/skills/excalidraw-diagram/` by git clone (SKILL.md,
references/color-palette.md, element-templates.md, json-schema.md, render_excalidraw.py), check
`element-templates.md` against the user's pasted copy, then `uv sync` +
`uv run playwright install chromium`. Follow the skill: colours only from `color-palette.md`;
section-by-section JSON; mandatory render → view PNG → fix loop. **Style override from the user:
hand-drawn look** (`roughness: 1`, Excalifont `fontFamily: 1`) instead of the skill's default 0/3.
Output: `~/UoM/HDL/splat-and-seek/docs/proposal/diagrams/{system_overview,engine_dataflow}.excalidraw`
(+ rendered PNGs). The PDF comes later.

**Datasheet facts used (Microchip PolarFire SoC product table and MSS TRM)**
- MPFS250T: 254 K LE; **784 math blocks (18×18 MACC)**; **812 LSRAM blocks × 20 Kbit (≈ 2.0 MB)**;
  2,352 µSRAM (64×12); 17.6 Mbit total RAM; 470 Kbit µPROM; 36-bit DDR controller with SECDED.
- MSS: 1× E51 (RV64IMAC) monitor + 4× U54 (RV64GC); **2 MB L2** (cache / LIM / scratchpad).
- FICs: **FIC0, FIC1 = 64-bit AXI4, initiator and target**; **FIC2 = AXI4 target only** (fabric
  initiators into MSS/DDR); **FIC3 = 32-bit APB, MSS initiator** (fabric control registers); FIC4 =
  AHB to the user crypto core. Fabric side ≤ 250 MHz.
- Icicle Kit: MPFS250T, 2 GB LPDDR4 x32, 2× GbE, USB OTG, MMUARTs, 12 V input.
- Plan-level assignment: FIC3 APB → engine control/status; FIC0 AXI4 → map/observation DMA from
  LPDDR4 into the engine; scores back via FIC0 (or an AXI4 target window); the real-time hart polls/IRQ.

**Diagram 1: System overview** (left → right, multi-zoom)
- Top summary strip (timeline): Survey → Build map → Load → Localize → Drive.
- Section "Base station (host PC, GPU)": survey video → COLMAP → gsplat → compaction/LOD → map
  file. Evidence artifact: the 32-byte Gaussian record layout.
- Section "Icicle Kit: MPFS250T" (hero): MSS (4× U54 Linux: map loader, UDP server, UIO driver;
  real-time hart: MCL firmware), FIC0/FIC2/FIC3 labelled, LPDDR4 2 GB (full-site map + frame
  buffers), fabric **Splat-and-Seek engine**; GbE. **Core boundary** = engine + driver/API + MCL
  firmware (highlighted). Evidence: `score_poses(poses[300]) → scores[300]`.
- Section "Robot: demo scaffolding" (muted colours): companion Pi/Jetson ← USB → Kinect v1
  (640×480 RGB-D, 30 fps); ← USB serial 115200 → Kobuki (odometry 50 Hz, drive commands); GbE/UDP
  to the Icicle. Evidence: the UDP observation packet (32×24 RGB-D + x, y, θ, t).
- Legend: core vs scaffolding.

**Diagram 2: Engine dataflow** (inside the fabric)
- Assembly line: LSRAM local map (~45 K × 32 B) → pre-cull (frustum/depth, stream compaction) →
  **sort once per update** (shared depth order) → broadcast bus.
- **Fan-out** to P = 4 pose lanes, each: 3-DoF transform (x, y, θ) → 2D covariance/conic → blend
  into a 32×24 colour + depth buffer.
- Observation path: frame from LPDDR4 via FIC0 → downsample + census/gradient once per frame.
- **Convergence**: lanes + observation → compare unit (census/gradient + depth) → score FIFO →
  real-time hart (resample loop shown as a cycle arrow back to the "poses in" port).
- Control via FIC3 APB registers. Annotations: "poses in groups of P; map read N/P times",
  "300 × 45 K = 13.5 M projections/update ≈ 40 Hz @ 150 MHz (target)", budget "~240–320 / 784
  math blocks; ~1.4 MB / ~2.0 MB LSRAM".

## 10. Work after approval
1. Proposal drafts (location to be chosen by the user; default `docs/proposal/` in the new repo):
   `overview.md`, `innovation.md`, block-diagram source + PDF, `proposal_notes.md` (this plan, condensed).
2. NMS repo: one sentence in `docs/project/future_work.md` §10 pointing to Splat-and-Seek as the
   continuation, built from scratch.
3. Memory: update `project-direction-nms.md` (3DGS is now the competition project) and add a
   Splat-and-Seek project memory (track, title, deadlines, gates).

## 11. Verification
- `wc -w` on the overview (≤ 300) and innovation (≤ 250).
- Every number traces to the exploration, a cited paper, or is labelled a target.
- PolarFire claims each have a Microchip source, or are removed.
- NMS repo: `uv run pytest -q tests/` (Markdown link check) passes after the future_work.md edit.
- Block-diagram PDF readable at A4; the public link opens in a private window.
