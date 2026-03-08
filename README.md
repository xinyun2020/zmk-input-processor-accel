# ZMK Input Processor Accel

A [ZMK input processor](https://zmk.dev/docs/keymaps/input-processors) that brings Apple Magic Trackpad-style acceleration to keyboards with trackpads like the [Cirque Pinnacle](https://zmk.dev/docs/features/pointers). Unlike the built-in [`zip_xy_scaler`](https://zmk.dev/docs/keymaps/input-processors/scaler) which applies a fixed multiplier, this module uses a [sigmoid curve](https://en.wikipedia.org/wiki/Sigmoid_function) that adapts to your movement speed: slow movements stay precise (~1x) for pixel-perfect targeting, fast swipes are amplified (~5x) for crossing the screen. The implementation uses a [1 Euro Filter](https://gery.casiez.net/1euro/) for jitter reduction at slow speeds without adding lag, and follows velocity calculation principles from [libinput](https://wayland.freedesktop.org/libinput/doc/latest/pointer-acceleration.html).

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

## Configuration

```c
#include <behaviors/input_processor_sigmoid_accel.dtsi>

&sigmoid_accel {
    min-scale = <50>;     // 0.5x at slow speeds
    max-scale = <400>;    // 4.0x at fast speeds
    threshold = <35>;     // velocity where curve centers
    steepness = <20>;     // transition sharpness
};

&glidepoint_listener {
    input-processors = <&sigmoid_accel>, <&zip_xy_scaler 2 1>;
};
```

| Parameter | Description |
|-----------|-------------|
| `min-scale` | Scale at slow speeds (/100, so 50 = 0.5x) |
| `max-scale` | Scale at fast speeds (/100, so 400 = 4.0x) |
| `threshold` | Velocity midpoint - lower = earlier acceleration |
| `steepness` | Curve sharpness - higher = more abrupt transition |

## How It Works

The sigmoid curve maps velocity to scale factor, similar to how [libinput handles touchpad acceleration](https://wayland.freedesktop.org/libinput/doc/latest/pointer-acceleration.html):

![Pointer acceleration curve](https://wayland.freedesktop.org/libinput/doc/latest/_images/ptraccel-touchpad.svg)

*Slow speeds (left) have lower acceleration, fast speeds (right) are amplified.*

Jitter is reduced using the 1 Euro Filter adaptive low-pass algorithm:

![1 Euro Filter](https://gery.casiez.net/1euro/1euroAlgorithm.png)

*Cutoff frequency adapts to speed - low cutoff smooths slow movements, high cutoff preserves fast movements.*

## License

MIT
