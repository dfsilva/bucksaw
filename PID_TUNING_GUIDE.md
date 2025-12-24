# Bucksaw PID Tuning Guide

A comprehensive guide to analyzing blackbox logs and tuning your drone for optimal flight performance.

---

## Table of Contents

1. [Getting Started](#getting-started)
2. [Understanding the Stats Tab](#understanding-the-stats-tab)
3. [Analyzing Step Response](#analyzing-step-response)
4. [Vibration Analysis](#vibration-analysis)
5. [PID Tuning Workflow](#pid-tuning-workflow)
6. [Filter Tuning](#filter-tuning)
7. [Common Issues & Solutions](#common-issues--solutions)

---

## Getting Started

### Recording a Blackbox Log

1. Enable blackbox logging in Betaflight Configurator
2. Set debug mode to `GYRO_SCALED` for best analysis
3. Fly a representative flight (include hover, forward flight, flips/rolls)
4. Download the `.BBL` or `.BFL` file from your FC

### Loading in Bucksaw

1. Run `cargo run` or launch the Bucksaw application
2. Click **🗁 Open File** and select your log
3. If multiple flights exist, select the one to analyze from the left panel

---

## Understanding the Stats Tab

The **📊 Stats** tab provides flight statistics to help identify issues.

### Stick Usage Histograms

| Pattern | Meaning | Action |
|---------|---------|--------|
| Centered peak (Roll/Pitch) | Normal flying | Good |
| Bimodal (two peaks) | Frequent stick corrections | May indicate wobble - check PIDs |
| Flat distribution | Very aggressive flying | Consider lowering rates |
| Heavy on one side | Trim or CG issue | Check mechanical trim |

### PID Term Statistics

**|Gyro|** (Mean Absolute Gyro)
- High values = drone moving a lot
- Compare across axes - should be similar for balanced flying

**|P-term|** (Mean Proportional Output)
- High P = aggressive P gain or lots of corrections needed
- Should be proportional to Gyro values

**|D-term|** (Mean Derivative Output, Roll/Pitch only)
- High D with low Gyro = D gain too high or vibrations
- Very low D = may need more D for propwash handling

**|I-term|** (Mean Integral Output)
- High I = constant offset being corrected (drift)
- May indicate mechanical issue or wind

### Motor Statistics

| Pattern | Meaning | Action |
|---------|---------|--------|
| All motors equal mean | Balanced | Good |
| One motor higher | That motor works harder | Check prop, motor, or arm twist |
| High standard deviation | Rapid corrections | Could be vibration or PID issues |

---

## Analyzing Step Response

The **⛭ Tune** tab shows how well your PIDs track setpoint changes.

### Reading Step Response Plots

```
Y-axis: Response ratio (1.0 = perfect tracking)
X-axis: Time in milliseconds

Ideal: Smooth rise to 1.0, stays there
```

### Step Response Patterns

| Shape | Diagnosis | Fix |
|-------|-----------|-----|
| ![Smooth rise to 1.0, no overshoot] | ✅ Well tuned | None needed |
| ![Overshoot > 10%] | P too high | Lower P by 10-20% |
| ![Oscillation after overshoot] | D too low or filters too aggressive | Raise D or relax filters |
| ![Slow rise, never reaches 1.0] | P/FF too low | Raise P and/or FF |
| ![Noisy/jagged line] | Vibrations or D too high | Lower D, check motors/props |

### Using Smoothing

Apply smoothing to see the underlying trend:
- **Off**: Raw data, shows all noise
- **Low**: Slight cleanup, preserves detail
- **Medium**: Good for most analysis
- **High**: Shows overall trend only

### Using Y Correction

Adjust the Y-axis offset to align the response if baseline is off:
- Positive: Shifts plot up
- Negative: Shifts plot down

---

## Vibration Analysis

The **💃 Vibe** tab shows frequency content in your flight data.

### Reading Spectrograms

- **X-axis**: Throttle percentage (0-100%)
- **Y-axis**: Frequency (Hz)
- **Color intensity**: Signal strength at that frequency/throttle

### What to Look For

| Frequency Range | Typical Source | Action |
|-----------------|---------------|--------|
| 50-150 Hz | Motor/prop vibration | Balance props, check motor bearings |
| 150-300 Hz | Frame resonance | Stiffen frame, add dampening |
| 300-500 Hz | ESC timing noise | Adjust ESC settings, use RPM filtering |
| Below 50 Hz | Low frequency oscillation | Check P/D gains |

### Identifying Problematic Vibrations

**Horizontal lines** at specific frequencies = constant vibration source
**Diagonal lines** rising with throttle = motor RPM harmonics
**Broadband noise** (fuzzy areas) = general vibration, check soft mount

---

## PID Tuning Workflow

### Step-by-Step PID Tuning

#### 1. Establish Baseline
- Fly with stock PIDs
- Record 2-3 minute flight with varied maneuvers
- Load in Bucksaw

#### 2. Analyze Step Response
- Check all three axes (Roll, Pitch, Yaw)
- Look for overshoot, oscillation, or slow response
- Note which axes need work

#### 3. Adjust P Gain

| Symptom | Change |
|---------|--------|
| Overshoot | Lower P by 10% |
| Slow response | Raise P by 10% |
| Oscillation at high throttle | Lower P by 15% |

#### 4. Adjust D Gain

| Symptom | Change |
|---------|--------|
| Propwash oscillation | Raise D by 10-15% |
| Hot motors, noisy | Lower D by 15-20% |
| Overshoot persists after lowering P | Raise D by 10% |

#### 5. Adjust I Gain

| Symptom | Change |
|---------|--------|
| Drift in hover | Raise I by 20% |
| Slow return to level | Raise I by 10% |
| Bounce back from flips | Lower I by 10% |

#### 6. Adjust Feedforward (FF)

| Symptom | Change |
|---------|--------|
| Stick feels slow | Raise FF by 15% |
| Snap back when releasing stick | Lower FF by 10% |
| Good tracking but hot motors | Let FF do more, lower P |

### Quick Reference: PID Changes

```
P: Controls immediate response strength
   ↑ More aggressive, ↓ Softer

D: Controls overshoot dampening
   ↑ Less overshoot but noisier, ↓ Smoother but more bounce

I: Controls long-term error correction
   ↑ Better hover stability, ↓ Less bounce back

FF: Controls predictive response
    ↑ Faster stick response, ↓ Smoother decel
```

---

## Filter Tuning

### Understanding Filters

Betaflight uses two main filter stages:
1. **Gyro lowpass filters** - Clean up gyro noise before PID
2. **D-term lowpass filters** - Filter D-term output (most important for noise)

### Using Vibe Tab for Filter Analysis

1. Set data source to **Gyro** to see raw noise
2. Look for noise peaks above your filter cutoff
3. If D-term FFT shows peaks, lower D lowpass cutoff

### Filter Tuning Guidelines

| Situation | Adjustment |
|-----------|------------|
| Hot motors, high D noise | Lower D lowpass cutoff (80→60 Hz) |
| Propwash not controlled | Raise D lowpass cutoff (100→120 Hz) |
| General noise throughout | Lower gyro lowpass cutoff |
| Dulled response | Raise filter cutoffs (be careful, test outdoors) |

### RPM Filtering (If Available)

If bidirectional DShot is enabled:
- RPM filtering removes motor noise at source
- Allows higher filter cutoffs = better propwash handling
- Check **eRPM** in Plot tab to verify it's working

---

## Common Issues & Solutions

### Issue: Oscillation on Quick Moves

**Symptoms:**
- Step response shows oscillation after setpoint change
- High P-term values on rapid input

**Solutions:**
1. Lower P by 15%
2. If still oscillating, raise D by 10%
3. Check for loose props or damaged motors

---

### Issue: Propwash Oscillation

**Symptoms:**
- Wobbles during descents or after flips
- Visible in low throttle regions of spectrogram

**Solutions:**
1. Raise D by 15-20%
2. Raise D lowpass filter cutoff (100→130 Hz)
3. Lower P slightly if D gets too noisy
4. Enable RPM filtering if available

---

### Issue: Hot Motors

**Symptoms:**
- Motors too hot to touch after flight
- High D-term noise in Vibe tab

**Solutions:**
1. Lower D by 20%
2. Lower D lowpass filter cutoff
3. Check props for damage/imbalance
4. Check motor bearings

---

### Issue: Drifting in Hover

**Symptoms:**
- Won't stay level hands-off
- High I-term in stats

**Solutions:**
1. Raise I by 15%
2. Check for bent motor shaft
3. Verify accelerometer calibration

---

### Issue: Sluggish Stick Feel

**Symptoms:**
- Step response slow to reach 1.0
- Feels delayed

**Solutions:**
1. Raise P by 10%
2. Raise FF by 15%
3. Check RC smoothing isn't too aggressive

---

## Summary: Tuning Cheat Sheet

| Tab | What to Check | Target |
|-----|---------------|--------|
| **Stats** | Motor balance | All motors ±5% of each other |
| **Stats** | D-term mean | Low but not zero |
| **Tune** | Step response | Smooth rise to 1.0, minimal overshoot |
| **Vibe** | Gyro spectrogram | No bright spots in 50-150 Hz |
| **Vibe** | D-term spectrogram | Minimal noise above filter cutoff |

---

*Happy tuning! Remember: always make one change at a time and test between changes.*
