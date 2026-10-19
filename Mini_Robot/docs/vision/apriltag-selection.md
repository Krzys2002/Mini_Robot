# AprilTag family, print size, and robot identity

Issue: [#49](https://github.com/Krzys2002/Mini_Robot/issues/49).
Prerequisites: catalog 11 → #31 and catalog 13 → #47, both closed and merged.
The maintainer confirmed #31 physical stream lifecycle acceptance on 2026-10-19.

## Decision and operating envelope

Use **AprilTag `tag36h11`**, OpenCV dictionary `DICT_APRILTAG_36h11`, with one
black border cell. Use the OpenCV ArUco detector as the baseline for the future
frame adapter (#50); no separate AprilTag library is needed for this selection.
Offline tooling supports OpenCV 4.2+ (the legacy ArUco API through 4.6 and the
objdetect API from 4.7), including Ubuntu 24.04's OpenCV 4.6. Actual validation
versions are recorded below; source compatibility is not a Linux execution claim.

The operator supplied these design ranges:

- A square mounting area of 0.02–0.04 m per robot, **including the white margin**.
- Initially 3–4 robots; allocate 16 distinct robot markers now.
- At least Full HD (1920 × 1080) acquisition; no camera model or fixed resolution.
- Table dimensions from 2 × 1 m to 4 × 4 m.

Select the **40 mm total footprint** wherever it fits: the measured outer black
square edge is **32 mm = 0.032 m**. The compact **20 mm total footprint** has a
**16 mm = 0.016 m** black edge and requires twice the linear image sampling.
These are two explicit print profiles; never enter the footprint as a pose
estimator's tag edge size. Intermediate robot mounting areas can use the 20 mm
profile; do not scale a sheet arbitrarily without recording the new black edge.

**These ranges cannot all be combined at Full HD.** A 4 × 4 m table needs higher
resolution or reduced per-camera coverage, even with the larger print. Multiple
cameras remain future work; do not assume fusion exists. The sampling constraints
below govern which combinations can be commissioned.

## Family comparison

All four families below are supported by the selected OpenCV dictionary/detector.
Distances and ID counts are from the
[OpenCV dictionary reference](https://docs.opencv.org/4.x/de/d67/group__objdetect__aruco.html).
Cells across include the one-cell black border on each side, excluding white.

| Family | Data grid | Cells across black edge | Minimum Hamming distance | Unique IDs | Decision |
| --- | --- | --- | --- | --- | --- |
| tag16h5 | 4 × 4 | 6 | 5 | 30 | Fits 16 robots and has larger cells for the same black edge, but weaker code separation |
| tag25h9 | 5 × 5 | 7 | 9 | 35 | Intermediate sampling and code separation, little expansion room |
| tag36h10 | 6 × 6 | 8 | 10 | 2320 | More IDs than required, weaker separation than 36h11 at the same sampling |
| **tag36h11** | **6 × 6** | **8** | **11** | **587** | Selected for stronger separation and ample IDs with existing OpenCV support |

Hamming distance is a codebook property, not a claim that the detector corrects
five errors under its default settings. Use conservative detector correction
settings and validate false positives in #50. The smaller 16h5 family could help
undersampling, but cannot make an 8.6-pixel-wide marker reliably observable.
Prioritize camera geometry over silently switching families.

[AprilRobotics recommends](https://github.com/AprilRobotics/apriltag#choosing-a-tag-family)
`tagStandard41h12` for general use and `tag36h11` for ArUco compatibility.
Standard41h12 is not among the selected OpenCV predefined AprilTag dictionaries;
adopting it would require a different detector/codebook path with no present need.

## Robot identity and mounting convention

The allocated key is `(family = tag36h11, tagId)`, not an unqualified integer:

- Tag IDs **0–15 map to `RobotId{tagId + 1}`**, yielding robot identities **1–16**.
  Use tags 0–3 for the first four robots. All 16 are supplied on both sheets.
- IDs **16–586 are unassigned**, reserved for deliberate future allocation,
  calibration or static references. They do not create robot identities.
- `RobotId{0}` remains valid in the domain/protocol; it is simply not allocated
  by this vision profile. Never change the global uint32 RobotId representation.
- Provision the physical robot's protocol Robot Info identity to the mapped
  RobotId. Do not derive identity from BLE address, discovery order, or image order.
- Only commissioned IDs with the expected family may become robot measurements.
  Unknown IDs/families are rejected as robot measurements. Duplicate printed IDs
  in one workspace are an ambiguous setup and must not be arbitrarily assigned.
  Later detection/association code must diagnose ambiguity rather than command it.
- One tag per robot; no two active robots share an ID. Reprints replace a damaged
  tag, and must not introduce a second visible copy. The two sheets are alternatives.

Mount the printed square flat, horizontal, centered over the robot pose reference
point. The canonical **top edge of the sheet marker points toward robot front**
(the label's `UP ^`). Do not mirror the print. The front direction is the vector
from the canonical bottom-edge midpoint to the top-edge midpoint; a detector's
decoded canonical corner ordering, rather than screen-top position, must preserve
it under rotation. Verify this convention physically before heading/control use.
An offset/rotated mount requires a measured per-robot rigid transform in later
pose conversion, not a change to identity mapping.

World headings remain radians, zero along +X, positive counterclockwise with
+Y up. Image +v points down. Transform marker geometry into the calibrated world
frame before deriving physical heading; image heading alone is not world heading.
Record tag-plane height: mapping an elevated marker through a tabletop-plane
homography introduces parallax, especially near image edges. Plane-aware
calibration or compensation belongs to the subsequent calibration/pose issues.

## Camera constraints independent of camera model

Let `W,H` be delivered image dimensions in pixels (after any crop/downsample),
`Cw,Ch` be the **actual full image coverage at the tag plane** in metres, and `S`
be black edge length in metres. A fronto-parallel first approximation is:

```text
q = min(W / Cw, H / Ch)       # limiting pixels/metre
p = S * q                    # projected black edge pixels
pixels_per_cell = p / 8
```

Set a **provisional minimum p ≥ 24 pixels** (3 px/cell), and a **preferred
p ≥ 40 pixels** (5 px/cell). These are engineering sampling budgets, **not
experimentally proven detection thresholds**. Require them at the worst sampled
corner/shortest projected edge after perspective and lens distortion. Blur,
compression, glare, focus, occlusion and detector downsampling can require more.
Never accept a setup solely because ideal synthetic markers decode.

This yields maximum full-frame coverage and minimum image dimensions:

```text
Cw_max = W * S / 24;  Ch_max = H * S / 24
W_min = ceil(24 * Cw / S);  H_min = ceil(24 * Ch / S)
# Substitute 40 for 24 for the preferred sampling budget.
```

Both axes must pass and the complete workspace must remain in view with a margin.
The Full HD floor is an additional requirement, not a detection guarantee.
There is no fixed upper image-resolution cap: use higher resolution only while
measured frame latency/cadence remain suitable for subsequent tracking/control.
Maximum table extent is 4 × 4 m; maximum **camera coverage** is the above budget,
which can be smaller than the table. Minimum coverage must contain the table plus
mounting/drift margin. Recompute if an adapter resizes frames or detection decimates.

For a 16:9 image, a 2 × 1 m table occupies at least 2 × 1.125 m of full image
coverage; a 4 × 4 m table occupies at least 7.111… × 4 m. Do not divide by the
table width alone when the square table leaves unused image columns.

| Image | Actual full coverage | 32 mm black edge p | 16 mm black edge p |
| --- | --- | --- | --- |
| 1920 × 1080 | 2 × 1.125 m | 30.72: provisional minimum only | 15.36: fails |
| 3840 × 2160 | 2 × 1.125 m | 61.44: preferred | 30.72: minimum only |
| 1920 × 1080 | 7.111… × 4 m | 8.64: fails | 4.32: fails |
| 3840 × 2160 | 7.111… × 4 m | 17.28: fails | 8.64: fails |
| 7680 × 4320 | 7.111… × 4 m | 34.56: minimum only | 17.28: fails |

These ideal examples omit coverage margins. At 4 m vertical coverage, the 32 mm
profile requires at least **3000 vertical pixels** for minimum sampling and 5000
for preferred; the 16 mm profile requires 6000 and 10000 respectively. A higher
resolution stream must be available at the processing boundary, not just the
camera's still-photo mode. For 1080 image height, maximum square coverage is
1.44 m for the 32 mm profile and 0.72 m for the 16 mm profile at minimum sampling.

Reproduce the table with standard Python (no OpenCV dependency):

```sh
python3 - <<'PY'
for w, h in [(1920,1080), (3840,2160), (7680,4320)]:
    for tw, th in [(2,1), (4,4)]:
        cw, ch = max(tw, th*w/h), max(th, tw*h/w)
        q = min(w/cw, h/ch)
        print(w, h, (tw,th), f'coverage {cw:.3f}x{ch:.3f}',
              f'32mm: {0.032*q:.2f}px; 16mm: {0.016*q:.2f}px')
PY
```

### Height and field of view

Measure height `z` from the **tag plane**, not the floor. For a centered, normal
camera with horizontal/vertical FoV `ax,ay`, use radians in computation:

```text
Cw = 2*z*tan(ax/2); Ch = 2*z*tan(ay/2)
z_min = max(table_width/(2*tan(ax/2)), table_height/(2*tan(ay/2)))
fx = W/(2*tan(ax/2)); fy = H/(2*tan(ay/2))
z_max = S*min(fx,fy)/24
```

Add coverage margin to `z_min`. If `z_min > z_max`, that camera mode/tag profile
cannot cover the table at the minimum sampling budget. For example, at 60°
horizontal FoV in a rectilinear 16:9 image, covering 2 × 1 m needs about 1.73 m
height; covering 4 × 4 m needs about 6.16 m. At Full HD, `fx ≈ fy ≈ 1663 px`, so
the 32 mm profile has `z_max ≈ 2.22 m` and the compact profile ≈ 1.11 m. Heights
are illustrative assumptions, not prescribed installation heights. Use calibrated
focal lengths and measured corner sampling for a tilted/distorted real camera.

## Printing and reproducibility

The checked-in [40 mm](markers/tag36h11-40mm-a4.svg) and
[20 mm](markers/tag36h11-20mm-a4.svg) sheets are vector A4 portrait SVGs, 16 tags
per sheet. They have 10 cells per footprint: 8 across the black square plus one
white quiet cell per side. Quiet margins are 4 mm per side on the large profile
and 2 mm on the compact profile. Print matte black on opaque white material.
Keep the complete white square when cutting; labels are outside that square.
No code cells, borders, or quiet margins may be covered by mounting hardware.

Print at **100% / actual size**, with no fit-to-page or nonuniform scaling.
Measure the 50 mm scale bar, black edge, and total footprint with a ruler/caliper
before mounting. If the white substrate blends into the background, retain it
physically anyway. Printing tolerance, lighting and detection performance need
measurement; no physical print accuracy is claimed by software verification.

The offline generator uses the OpenCV predefined dictionary, never hand-copied
bit patterns. It generates deterministic SVGs with physical millimetre units.
It is opt-in and adds no dependency to the default desktop/domain targets:

```sh
cmake -S . -B build/issue-49 -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DMICRO_ROBOT_BUILD_MARKER_TOOLS=ON -DBUILD_TESTING=ON
cmake --build build/issue-49 --parallel 3
build/issue-49/tools/markers/micro_robot_print_markers --generate docs/vision/markers
ctest --test-dir build/issue-49 --output-on-failure --no-tests=error
```

The tooling needs OpenCV core/imgproc/imgcodecs and aruco (OpenCV ≤4.6) or
objdetect (≥4.7); Ubuntu's `libopencv-dev` supplies these. GoogleTest resolution
follows the [existing test guide](../development/testing.md). Desktop/replay can
be disabled for a Qt-free tool build. Tests check ID decoding under all four
quarter-turns, quiet margin/border geometry, invalid inputs, and byte equality
between shipped sheets and regenerated dictionary output.

To independently rasterize and decode the **actual shipped SVG** (requires
`rsvg-convert`, supplied by librsvg/librsvg2-bin, only for this optional check):

```sh
rsvg-convert --dpi-x 300 --dpi-y 300 docs/vision/markers/tag36h11-40mm-a4.svg -o build/issue-49/sheet-40.png
rsvg-convert --dpi-x 300 --dpi-y 300 docs/vision/markers/tag36h11-20mm-a4.svg -o build/issue-49/sheet-20.png
build/issue-49/tools/markers/micro_robot_print_markers --verify-sheet build/issue-49/sheet-40.png
build/issue-49/tools/markers/micro_robot_print_markers --verify-sheet build/issue-49/sheet-20.png
```

Each must decode exactly IDs 0–15 once. This validates print content in ideal
raster conditions, not physical detection range. Inspect the raster visually as
well; labels and the scale bar must remain legible and outside quiet zones.

## Validation record and required physical commissioning

Software validation results for this issue are recorded in its GitHub checklist
and implementation report. Physical camera/print measurements have **not** been
performed for this selection. Before using these tags for pose/control:

1. Measure each print and mount, record profile/black edge, verify front alignment,
   and check that protocol RobotId matches its printed tag mapping.
2. Record camera identity, effective resolution, height above tags, FoV/focus,
   workspace bounds, exposure, lighting, tag-plane height, and any crop/decimation.
3. At center and all corners, measure shortest marker edge in pixels. Enforce the
   minimum sampling budget; prefer 40 px or more. Repeat at rotated headings.
4. With 3–4 tags and then all 16 distinct tags, record per-ID success rate,
   false positives/identity swaps, detection latency and frame cadence over a
   repeatable static and moving sequence. Include blank/non-tag backgrounds and
   partial occlusion. Record actual counts and durations; do not infer rates
   from synthetic tests. Verify recovered heading while turning a robot.
5. Retain timestamped recorded input and configuration for #50 and subsequent
   localization benchmarks. Establish measured thresholds before autonomous use.

Failures require tighter coverage, larger usable print area, improved optics or
more delivered pixels. Family count alone does not establish reliable identity,
metric accuracy, real-time operation, or safety under lost tracking.
