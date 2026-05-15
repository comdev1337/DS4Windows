## TLDR

Main Purpose of this patch:
* unfuck throttle-dependent variable yaw sensitivity in Realistic Combat Drones (2.3.4) mod to be FLAT
* Use gyro => L2/R2 for throttle up/down with haptics

## ARMA Drone Yaw Model

Reference for the ARMA-profile drone-yaw pipeline in DS4Windows. Covers how
stick input, DS4Windows boost, the drone mod's `yawEnvelope`, and the game's
hidden stick response curve interact to produce actual yaw torque.

> **Scope.** Unless you edit the drone mod, `m_fYawSensitivity`, `yawEnvelope`, and the engine's stick
> curve are all fixed. The only knobs are in DS4Windows: `LSSens` and the
> `_gameStickCurve` calibration table.

---

## Drone constants

From `Prefabs/Drones/FPVBase/FPVBase.et`:

| Constant | Value | Notes |
|---|---:|---|
| `RigidBody.Mass` | **0.8 kg** | Flight physics mass. |
| `m_iMaxThrustRPM` | 20 000 | **Cosmetic only** — visual rotor spin & sound. Does not affect flight. |
| `m_fYawSensitivity` | **16** | Multiplier on stick input for yaw. |
| `m_fPitchSensitivity` | 16 | |
| `m_fRollSensitivity` | 16 | |

HUD current: `HUD_A = 0.8 × 40 × m_iThrottle = 32 × m_iThrottle`.

## Yaw torque — the actual physics

Yaw torque is **not** limited by the rotor RPM clamp. The clamp
(`Math.Clamp(targetRPMs[i], 0, m_iMaxThrustRPM)`) only affects the cosmetic
rotor spin animation and sound. Actual yaw torque applied to the rigid body
is simply:

```
yawTorque = m_iYaw × 20
```

where `m_iYaw` comes from the input pipeline (see below). This means yaw
authority is controlled entirely by the input chain — there is no downstream
physics limiter.

## Input pipeline

Two code paths exist depending on `GetLastUsedInputDevice()`:

### Gamepad path

```
raw_stick → [game stick curve] → GetActionValue → × yawSens(16) × yawEnvelope → m_iYaw
```

DS4Windows inserts itself before the game sees the virtual stick:

```
               DS4Windows                          Game engine
raw_stick → × LSSens × boost → byte [0–255] → [stick curve] → norm → × 16 × env → m_iYaw
```

### Keyboard path (Q/E)

```
GetActionValue → × yawSens(16) / 4 → m_iYaw
```

Keyboard gives a constant `m_iYaw = ±4` regardless of throttle. No envelope.
`GetLastUsedInputDevice()` switches between paths based on the last physical
input — touching the controller while using Q/E causes a sudden jump from
`×4` to `×16×env`, which at hover (`env=1`) is a 4× increase.

## yawEnvelope

```csharp
yawEnvelope = 1.0 - Math.Abs((m_iThrottle - 0.5) * 2.0);
yawEnvelope = Math.Clamp(yawEnvelope, 0.3, 1.0);
```

| HUD A | m_iThrottle | env |
|---:|---:|---:|
| 0 | 0.000 | 0.30 |
| 4.8 | 0.150 | 0.30 |
| 8 | 0.250 | 0.50 |
| 12 | 0.375 | 0.75 |
| 16 | 0.500 | **1.00** (hover) |
| 20 | 0.625 | 0.75 |
| 24 | 0.750 | 0.50 |
| 27.2 | 0.850 | 0.30 |
| 32 | 1.000 | 0.30 |

Symmetric tent peaking at hover. The goal of DS4Windows compensation is to
make yaw feel the same at every throttle by boosting the stick to cancel this
envelope.

## The game's stick response curve

The Enfusion engine applies a severe non-linear transform to gamepad stick
input before `GetActionValue` returns it. This is **not** a simple power law
— the curve is nearly linear (slope ~0.5) from norm 0.3 to 0.7, then
hockey-sticks to 15× steeper between 0.95 and 1.0. Likely a hand-tuned
spline in the engine's `FilterPreset`.

This curve was reverse-engineered empirically at hover (16A, `env=1.0`,
`boost=1.0`) by varying `lsSens` and measuring heading-based yaw RPM over 5s
windows. RPM is proportional to the curve output `f(norm)`, normalized so
`f(1.0) = 1.0`:

| lsSens | norm | RPM | f |
|---:|---:|---:|---:|
| 0.25 | 0.244 | 0 | 0.000 |
| 0.30 | 0.299 | 4.120 | 0.045 |
| 0.40 | 0.394 | 8.361 | 0.091 |
| 0.50 | 0.496 | 13.126 | 0.144 |
| 0.60 | 0.598 | 17.945 | 0.196 |
| 0.70 | 0.693 | 22.769 | 0.249 |
| **0.75** | **0.748** | **26.343** | **0.288** |
| 0.80 | 0.795 | 30.126 | 0.329 |
| 0.82 | 0.819 | 32.419 | 0.354 |
| 0.83 | 0.827 | 33.239 | 0.363 |
| 0.85 | 0.843 | 34.944 | 0.382 |
| 0.90 | 0.898 | 42.099 | 0.460 |
| 0.93 | 0.921 | 47.702 | 0.522 |
| 0.94 | 0.937 | 50.714 | 0.554 |
| 0.95 | 0.945 | 54.654 | 0.598 |
| 0.96 | 0.953 | 59.362 | 0.649 |
| 0.97 | 0.969 | 70.018 | 0.766 |
| 0.98 | 0.976 | 75.656 | 0.827 |
| 0.99 | 0.984 | 81.222 | 0.888 |
| 1.00 | 1.000 | 91.464 | 1.000 |

The game-side stick deadzone is at approximately norm 0.244 (below this,
zero output). DS4Windows's own `LSDeadZone` of 0.05 is well below this.

### Why this matters

A naive `boost = 1/env` only cancels the `yawEnvelope` if the stick curve
were linear. Since `f` is severely non-linear, pushing the stick from norm
0.748 to (say) 1.0 doesn't multiply the game's output by 1.0/0.748 = 1.34×
— it actually multiplies it by `f(1.0)/f(0.748) = 1.0/0.288 = 3.47×`.

The correct compensation must invert both the envelope and the stick curve
simultaneously.

## GetYawBoost — how compensation works

Goal: at any throttle, full-stick yaw RPM = hover full-stick yaw RPM.

At hover (`env=1`), the game sees norm = `HoverNorm = 0.748` (from
`lsSens=0.75`) and produces `f(0.748) = 0.288`. At a different throttle with
envelope `env`, we need to find a norm such that:

```
f(norm) × env = f(HoverNorm)
```

Solving:

```
neededF = f(HoverNorm) / env
neededNorm = f⁻¹(neededF)
boost = neededNorm / HoverNorm
```

The boost is capped at `1.0/HoverNorm ≈ 1.337` because the virtual stick
byte saturates at 255 (norm = 1.0). Beyond that cap, the game sees full
stick and yaw equals `f(1.0) × env × 16`, which may still be above the
hover baseline due to the steep top of the curve.

### Saturation boundary

The byte saturates when `neededNorm ≥ 1.0`, i.e. when:

```
env ≤ f(HoverNorm) / f(1.0) = 0.288
```

Since `env` floors at 0.3, compensation is **almost always sufficient** — the
only saturation is at the very extremes (HUD < ~5A or > ~27A) where `env`
hits 0.3 and neededF = 0.288/0.3 = 0.960. `f⁻¹(0.960)` is about 0.988,
giving boost ≈ 1.321. Still below the 1.337 cap, so even at the envelope
floor the compensation is nearly exact.

### Measured results

After calibration, yaw RPM at full stick across throttle:

| HUD A | Yaw RPM | vs hover |
|---:|---:|---:|
| 16 | 26.3 | 1.00× |
| 20 | 26.2 | 1.00× |
| 21 | 26.7 | 1.01× |
| 29.3 | 26.0 | 0.99× |
| 30.7 | 26.0 | 0.99× |

Effectively flat across the usable throttle range.

## Throttle estimation

DS4Windows needs to know the current `m_iThrottle` to compute `env`. From
the mod source:

```c
float rawInput = Math.Clamp(GetActionValue("DroneUp"), -1.0, 1.0);
m_iThrottle = (rawInput + 1.0) * 0.5;
```

Triggers go through the same engine curve as sticks, so for a trigger byte B:

```
rawInput = f(B / 255)
dT = |m_iThrottle − 0.5| = f(B / 255) / 2
```

No separate calibration table needed — `LookupDeltaT` just calls
`LookupGameCurve(b / 255.0) / 2.0`.

## Haptic feedback

Gyro throttle has no physical resistance, so haptics provide positional
awareness via the DS4's light (right) rumble motor. Controlled by
`EnableHaptics` in `ArmaController.cs`.

### Threshold pulses

Short one-shot buzz when crossing a configured amp value in either direction.
A 200ms cooldown prevents buzz-spam when oscillating at a boundary.

| Threshold | Duration | Intensity | Notes |
|---:|---:|---:|---|
| 8A | 50ms | 90 | deep descent |
| 12A | 40ms | 50 | moderate descent |
| 16A | 40ms | 30 | hover |
| 17A | 40ms | 50 | |
| 18A | 40ms | 70 | |
| 19A | 50ms | 90 | |
| 20A | 50ms | 110 | |
| 24A | 60ms | 140 | env=0.5 |
| 28A | 70ms | 180 | near env floor |

### Below-hover continuous rumble

Linear ramp on the light motor: 16A → 0, 11A → 40 (capped below 11A).
Provides constant tactile warning that the drone is descending.

### Output merging

Each frame: `finalLight = max(activePulse, continuousRumble)`. Rumble is
only written to the controller when non-zero (or transitioning to zero), so
game-generated rumble is not stomped during normal flight.

## Configuration

| Setting | Location | Value |
|---|---|---:|
| `EnableYawBoost` | `ArmaController.cs` | true |
| `EnableHaptics` | `ArmaController.cs` | true |
| `LSSens` (X, yaw) | `ARMA.xml` | 0.75 |
| `LSSens` (Y, pitch) | `ARMA.xml` | 0.75 |
| `HoverNorm` | `ArmaController.cs` | 0.748 |
| `_gameStickCurve` | `ArmaController.cs` | 27 points |
| `OutputAntiDeadZone` | `ArmaController.cs` | 0.22 |
| `AccumulateRate` | `ArmaController.cs` | 10.0 |
| DS4Windows `LSDeadZone` | DS4Windows UI | 0.05 |
