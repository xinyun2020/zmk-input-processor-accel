# ZMK Sigmoid Acceleration Input Processor

Steam Deck-style trackpad/mouse acceleration for ZMK keyboards.

## Features

- **Precision at slow speeds**: Small, careful movements remain 1:1
- **Amplification at fast speeds**: Quick swipes cover more distance
- **Smooth sigmoid transition**: No jarring jumps between modes
- **Configurable curve**: Tune min/max scale, threshold, and steepness

## Installation

Add to your `config/west.yml`:

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

In your keymap or config dtsi:

```c
#include <behaviors/input_processor_sigmoid_accel.dtsi>

&glidepoint_listener {
    input-processors =
        <&sigmoid_accel>   // Add acceleration first
        , <&zip_xy_transform ...>
        , <&zip_xy_scaler 3 1>;  // Can reduce scaler since accel handles fast movements
};
```

## Configuration

Override defaults to customize the curve:

```c
&sigmoid_accel {
    min-scale = <100>;    // 1.0x at slow speeds (100 = 1.0)
    max-scale = <400>;    // 4.0x at fast speeds (400 = 4.0)
    threshold = <30>;     // Lower = acceleration kicks in earlier
    steepness = <20>;     // Higher = sharper transition
};
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min-scale` | 100 | Scale factor at slow speeds (÷100, so 100=1.0x) |
| `max-scale` | 300 | Scale factor at fast speeds (÷100, so 300=3.0x) |
| `threshold` | 50 | Velocity where curve is centered |
| `steepness` | 10 | How sharp the transition is (÷100, so 10=0.1) |

### Curve Visualization

```
Scale
  ^
4x|                    ___________
  |                 __/
3x|              __/
  |           __/
2x|        __/
  |     __/
1x|____/
  +--------------------------------> Velocity
       ^threshold
```

## How It Works

1. Measures instantaneous velocity from input deltas and time
2. Applies sigmoid function: `scale = min + (max-min) * sigmoid((velocity-threshold) * steepness)`
3. Scales the input value by the calculated factor
4. Tracks remainder for sub-pixel precision

## License

MIT
