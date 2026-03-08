# ZMK Input Processor Accel

Apple Magic Trackpad-style acceleration for ZMK keyboards.

## The Problem

[`zip_xy_scaler 5 1`](https://zmk.dev/docs/keymaps/input-processors/scaler) = always 5x. Jittery precision or sluggish swipes - pick one.

## The Fix

- **Slow** → ~1x (precise, jitter-free)
- **Fast** → ~5x (covers screen)
- **Smooth transition** → true sigmoid curve

## Features

- **Sigmoid curve** - Lookup table with interpolation
- **Combined XY velocity** - Consistent diagonal behavior
- **Velocity smoothing** - EMA prevents erratic acceleration
- **1 Euro filter** - Reduces jitter at slow speeds without lag

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
    input-processors = <&sigmoid_accel>, <&zip_xy_scaler 2 1>;
};
```

## How It Feels

```
Scale
  ^
5x|                    ___________  fast swipes
  |                 __/
4x|              __/
  |           __/
3x|        __/    sigmoid curve
  |     __/
2x|  __/
  |_/
1x| precision
  +--------------------------------> Velocity
```

## Research

- [1 Euro Filter](https://gery.casiez.net/1euro/) - Casiez et al., CHI 2012
- [libinput acceleration](https://wayland.freedesktop.org/libinput/doc/latest/pointer-acceleration.html) - Freedesktop

## License

MIT
