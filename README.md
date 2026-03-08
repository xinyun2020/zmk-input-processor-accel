# ZMK Input Processor Accel

Apple Magic Trackpad-style acceleration for ZMK keyboards.

## The Problem

[`zip_xy_scaler 5 1`](https://zmk.dev/docs/keymaps/input-processors/scaler) = always 5x. Jittery precision or sluggish swipes - pick one.

## The Fix

- **Slow** → ~1x (precise, jitter-free)
- **Fast** → ~5x (covers screen)
- **Smooth transition** → true sigmoid curve, not linear approximation

## Features

Based on research from HCI literature and production implementations:

- **True sigmoid curve** - Lookup table with interpolation, matches the characteristic S-curve
- **Combined XY velocity** - Uses motion vector magnitude for consistent diagonal behavior
- **Velocity averaging** - Smooths across 4 samples to prevent erratic acceleration changes
- **1 Euro filter** - Adaptive low-pass filter reduces jitter at slow speeds without adding lag

## Installation

Add to `config/west.yml`:

```yaml
manifest:
  remotes:
    - name: xinyun2020
      url-base: https://github.com/xinyun2020
  projects:
    - name: zmk-input-processor-accel
      remote: xinyun2020
      revision: main
```

## Usage

```c
#include <behaviors/input_processor_sigmoid_accel.dtsi>

&sigmoid_accel {
    min-scale = <50>;     // 0.5x slow
    max-scale = <400>;    // 4.0x fast
    threshold = <35>;
    steepness = <20>;
};

&glidepoint_listener {
    input-processors =
        <&sigmoid_accel>
        , <&zip_xy_transform (...)>
        , <&zip_xy_scaler 2 1>;
};
```

## How It Feels

```
Scale                               Fixed: ════════════════════ (always 5x)
  ^
5x|                    ___________ ← fast swipes
  |                 __/
4x|              __/
  |           __/
3x|        __/    ← sigmoid curve
  |     __/
2x|  __/
  |_/
1x|← precision
  +--------------------------------> Velocity
```

## Research

Implementation informed by:

- **1 Euro Filter** - Casiez, Roussel, Vogel (CHI 2012) - Adaptive low-pass filtering that reduces jitter without adding lag. Used in Chrome, Unreal Engine, and major input systems. [gery.casiez.net/1euro](https://gery.casiez.net/1euro/)

- **libinput pointer acceleration** - Freedesktop's reference implementation for Linux. Documents velocity calculation, tracker systems, and device-specific tuning. [wayland.freedesktop.org/libinput](https://wayland.freedesktop.org/libinput/doc/latest/pointer-acceleration.html)

- **Transfer functions for pointing** - Casiez, Vogel (CHI 2008) - Foundational research on pointer acceleration curves and user performance

## License

MIT
