# ZMK Input Processor Accel

Apple/Steam Deck-style trackpad feel for ZMK keyboards.

## The Problem

[`zip_xy_scaler 5 1`](https://zmk.dev/docs/keymaps/input-processors/scaler) = always 5x. Jittery precision or sluggish swipes - pick one.

## The Fix

- **Slow** → ~1x (precise)
- **Fast** → ~5x (covers screen)
- **Smooth transition** → sigmoid curve ramps gradually, cursor speed matches your expectation

## Features

- **Sigmoid acceleration**: 0.7x → 5x based on velocity
- **Inertial scrolling**: Cursor slides after finger lift

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
#include <behaviors/input_processor_inertial.dtsi>

&sigmoid_accel {
    min-scale = <70>;     // 0.7x slow
    max-scale = <500>;    // 5.0x fast
    threshold = <25>;
    steepness = <25>;
};

&inertial {
    decay-rate = <88>;    // smooth decay
    min-velocity = <40>;  // coast until slow
    tick-ms = <12>;       // ~83fps
    lift-timeout-ms = <35>;
};

&glidepoint_listener {
    input-processors =
        <&sigmoid_accel>
        , <&inertial>
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

## License

MIT
