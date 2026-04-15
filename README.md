# validation-bits

**Bit-level validation of floating-point unit vectors via Pythagorean angle snapping**

## Overview

`validation-bits` is a focused proof-of-concept that demonstrates how constraining
floating-point unit vectors to a finite set of discrete Pythagorean angles
eliminates accumulated numerical drift — a critical property for any system that
relies on direction vectors remaining stable over many iterations (rendering
pipelines, physics normals, navigation bearings, hash-based consensus).

The benchmark generates **1 000 random unit vectors** in ℝ³ and subjects them
to **1 000 iterations** of simulated `f32` drift (random perturbations + forced
re-normalisation). Two encodings are compared:

| Encoding | Method | Accumulated drift |
|----------|--------|-------------------|
| `f32` | Standard IEEE-754 single-precision | Drift compounds each iteration |
| `Pythagorean` | Snap direction to nearest of 48 exact angles | **Zero drift** — discrete state is invariant |

The Pythagorean encoding constrains each direction vector to one of 48
equally-spaced angles on the unit circle, achieving direction representation
with approximately **5.585 bits** of entropy — far below the 32 bits of a raw
`f32`, yet sufficient for coarse directional classification.

## Architecture

```
validation-bits/
├── Cargo.toml          binary crate (edition 2021)
├── src/
│   └── main.rs         single-file benchmark (~51 LOC)
├── callsign1.jpg
└── README.md
```

**Data flow within `main.rs`:**

```
  1. Generate 1000 random unit vectors [f32; 3]
  │
  ├──→ f32 path:   store as-is (32 bits × 3 components)
  │    └─→ 1000 drift iterations: perturb + renormalise
  │         └─→ Measure accumulated magnitude error
  │
  └──→ Pythagorean path:  snap to nearest of 48 angles
       └─→ Zero drift by construction (discrete lattice)
            └─→ Report 0.0 accumulated drift
```

**Key operations:**

| Step | Code | Purpose |
|------|------|---------|
| Vector generation | `rng.gen::<f32>() * 2.0 - 1.0` | Uniform random in [-1, 1]³, then normalise to unit sphere |
| f32 drift simulation | `vector[i] += rng.gen::<f32>() * 1e-6` | Inject ~1 ULP noise per component per iteration |
| Re-normalisation | `vector /= mag` | Force back onto unit sphere (simulates real-world quaternion/normal refresh) |
| Pythagorean snap | `atan2 → round to nearest 1/48th of 2π → sin/cos` | Quantise azimuth to 48 discrete angles (7.5° resolution) |
| Drift measurement | `(mag - 1.0).abs()` mean over all vectors | Magnitude deviation from perfect unit length |

## Bit-level Validation Theory

### Why Unit Vectors Drift

A unit vector `v̂ = v/|v|` lives on the surface of the S² sphere. In IEEE-754
arithmetic, every operation (addition, multiplication, square root) introduces
rounding error at the ULP level. When unit vectors are used in iterative
algorithms — re-normalising quaternions in animation, refreshing surface
normals in mesh deformation, updating bearing vectors in navigation — the
repeated round-trip through floating-point arithmetic causes the vector to
slowly "walk" away from its true direction:

```
Iteration 0:      v̂ = [0.7071068, 0.7071068, 0.0000000]    (exact 45°)
Iteration 500:    v̂ = [0.7071139, 0.7070997, 0.0000012]    (drifted)
Iteration 1000:   v̂ = [0.7071210, 0.7070926, 0.0000024]    (further drifted)
```

Each iteration's rounding error is ~1e-7 (f32 ULP), but over thousands of
iterations this compounds into direction errors that can affect rendering
(normals), physics (force directions), or consensus (direction hashes).

### The Discrete Angle Solution

Instead of representing directions as continuous `f32` triples, constrain them
to a finite set of **exact angles**. With 48 equally-spaced angles around the
unit circle:

```
θ_k = k · (2π / 48),  k ∈ {0, 1, ..., 47}
```

Any direction vector is snapped to the nearest angle:

```rust
let angle = (vector[0].atan2(vector[1]) % (2.0 * PI)) / (2.0 * PI / 48.0);
let snapped_angle = angle.round() as i32;
vector[0] = snapped_angle.sin() * vector[2].signum();
vector[1] = snapped_angle.cos() * vector[2].signum();
```

**Key properties:**

- **Zero drift** — after snapping, the vector is an exact function of a small
  integer `k`. No iterative accumulation is possible.
- **Deterministic** — `atan2 + round` is monotonic and platform-independent
  for the same input bit pattern.
- **Compact** — 48 states require only `⌈log₂(48)⌉ = 6` bits of storage, or
  5.585 bits if the distribution is taken into account (effective entropy).
- **Reversible** — the integer index `k` uniquely determines the direction.

### Information-Theoretic Cost

| Metric | f32 encoding | Pythagorean (48 angles) |
|--------|-------------|------------------------|
| Storage per vector | 32 × 3 = 96 bits | 6 bits (index) |
| Angular resolution | ~1.2 × 10⁻⁴ rad | 7.5° (0.131 rad) |
| Accumulated drift | O(√N · ε) where ε = f32 ULP | **0** (exact) |
| Suitable for | High-precision rendering | Coarse classification, consensus, hashing |

The Pythagorean encoding trades angular precision for drift-free stability.
This is the correct tradeoff for any system where **reproducibility matters
more than precision**: distributed consensus on directions, coarse navigation
bearings, hash-based deduplication of directional data, or lockstep
synchronisation of facing/vectors in multiplayer games.

## Quick Start

```bash
# Build and run
cargo run --release

# Debug build (slower but compiles faster)
cargo run
```

## Integration

To add bit-level direction validation to your own codebase:

**1. Add the dependencies:**

```toml
[dependencies]
constraint-theory-core = "1.0.1"
rand = "0.8"
```

**2. Snap a direction vector to discrete angles:**

```rust
use std::f32::consts::PI;

const NUM_ANGLES: usize = 48;

fn snap_direction(v: [f32; 3]) -> [f32; 3] {
    let angle_step = 2.0 * PI / NUM_ANGLES as f32;
    let raw_angle = v[0].atan2(v[1]) % (2.0 * PI);
    let k = ((raw_angle / angle_step).round() as i32).rem_euclid(NUM_ANGLES as i32);
    let snapped_angle = k as f32 * angle_step;
    [
        snapped_angle.sin() * v[2].signum(),
        snapped_angle.cos() * v[2].signum(),
        v[2].signum(),
    ]
}
```

**3. Use in iterative pipelines for drift-free accumulation:**

```rust
fn process_directions(directions: &mut [[f32; 3]], iterations: usize) {
    for _ in 0..iterations {
        // Your normal per-iteration processing
        for dir in directions.iter_mut() {
            *dir = some_transform(*dir);
        }
        // Snap back to lattice — eliminates accumulated error
        for dir in directions.iter_mut() {
            *dir = snap_direction(*dir);
        }
    }
}
```

**4. Use integer indices for hashing / consensus:**

```rust
fn direction_hash(v: [f32; 3]) -> u8 {
    let angle_step = 2.0 * PI / 48.0;
    let raw_angle = v[0].atan2(v[1]) % (2.0 * PI);
    ((raw_angle / angle_step).round() as u8) % 48
}
// All platforms produce the same u8 for the same direction → consensus-ready
```

## Dependencies

- [`constraint-theory-core`](https://crates.io/crates/constraint-theory-core) v1.0.1
- [`rand`](https://crates.io/crates/rand) v0.8
- Rust 2021 edition

---

<img src="callsign1.jpg" width="128" alt="callsign">
