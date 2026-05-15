using System;
using System.IO;

namespace DS4Windows
{
    /// ARMA profile integration.
    ///
    /// Ties the existing GyroMouseStick (Output Mode = "Mouse Joystick")
    /// plumbing to a held accumulator so gyro pitch/yaw doesn't snap
    /// back to center when the controller is still (throttle-like feel,
    /// not stick-like feel). Accumulator value is routed to *analog
    /// trigger output* L2/R2 rather than virtual LS/RS, because game-
    /// side stick-axis deadzones and cross-axis coupling made the
    /// virtual-stick approach too messy. Triggers are physically
    /// independent, and their deadzone is also about 21% in Reforger
    /// (measured: L2=54/255 is the smallest value that produces
    /// above-hover throttle), which we pre-compensate via
    /// OutputAntiDeadZone.R
    ///
    /// Yaw cancellation for the drone mod's throttle-dependent yaw
    /// envelope is still done via an LSSens multiplier, but the
    /// multiplier now acts only on LS X (hook in Mapping.cs, see
    /// GetYawBoost + patch notes at the end of this file). LS Y is
    /// untouched so pitch on LS Y is not scaled with throttle.
    ///
    /// Profile XML (Global.LSSens[], Global.LSModInfo[].verticalScale)
    /// is never mutated — everything that looks like "sensitivity
    /// boost" is applied at read time via GetYawBoost, so the values
    /// the user sees in the AxisConfig tab are the values that persist.
    internal static class ArmaController
    {
        private enum FlightMode { None, StickFlight, GyroFlight }

        private const string ProfileStickFlight = "ARMA";
        private const string ProfileGyroFlight  = "ARMA_FPV";

        // Throttle integration rate. Higher = less gyro rotation per
        // full-throttle sweep. Tune alongside the profile's
        // GyroMouseStick MaxZone and VerticalScale.
        public const double AccumulateRate = 10.0;

        // Output anti-deadzone, used when converting accumulator
        // magnitude → trigger byte so the first non-zero tilt pops
        // past the game's ~21% trigger deadzone. Measured via binding
        // DroneUp to L2 and observing that L2=54/255 (21.2%) was the
        // smallest value that produced above-hover throttle.
        public const double OutputAntiDeadZone = 0.22;

        // Set to false to disable yaw envelope compensation. GetYawBoost
        // will return 1.0, letting the raw yawEnvelope through. Useful
        // for calibrating the game's stick curve at a known env.
        public const bool EnableYawBoost = true;

        // ---- GyroFlight mode (ARMA_FPV profile) ---- //
        // LS Y rate-based throttle: stick outside deadzone = rate of change.
        private const double GF_ThrottleDeadzone = 0.15;
        private const double GF_ThrottleRate = 8.0;

        // Gyro flight axis scaling (applied after DZ/maxZone normalization).
        // Higher = more sensitive tilt-to-stick deflection.
        private const double GF_YawScale   = 1.0;
        private const double GF_PitchScale = 1.0;
        private const double GF_RollScale  = 1.0;

        // ---- Haptic feedback for gyro throttle ---- //
        public const bool EnableHaptics = true;

        private const double HapticHoverAmps = 16.0;
        private const double HapticDeadBandAmps = 0.5;

        // Above hover: periodic short buzz every AboveHoverPulseIntervalMs.
        // Intensity scales linearly with distance from the dead band edge.
        private const long AboveHoverPulseIntervalMs = 250;
        private const double AboveHoverPulseDurationSec = 0.05;
        private const byte AboveHoverMaxIntensity = 180;

        // Silence the periodic above-hover buzz after being at max
        // throttle for this long (e.g. L1 held for sustained climb).
        private const double MaxThrottleAmps = 31.0;
        private const long MaxThrottleSilenceMs = 2000;

        // Acceleration-dependent threshold buzz: fires a short pulse
        // when crossing a step boundary while throttle is actively
        // increasing, preventing buzz-spam near thresholds.
        private const double AccelBuzzStepAmps = 1.6;        // 5% of 32A
        private const double AccelBuzzAboveAmps = 16.0;
        private const double AccelBuzzMinRate = 3.0;          // A/s
        private const int    AccelBuzzWindowMs = 200;
        private const double AccelBuzzDurationSec = 0.05;
        private const byte   AccelBuzzIntensity = 120;

        // Below hover: continuous light rumble, linear ramp from
        // HapticHoverAmps (zero) down to BelowHoverFloorAmps (max).
        private const double BelowHoverFloorAmps = 11.0;
        private const byte BelowHoverMaxIntensity = 40;

        // Double-click R1 snaps accumulator to center. The two toggles
        // cancel each other out so gyro on/off state is preserved;
        // only the throttle value resets.
        private const long DoubleClickWindowMs = 250;

        // If true, accumulator resets to center whenever a fresh ARMA
        // profile is loaded.
        private const bool ResetAccumOnProfileLoad = true;

        private static readonly double[] _accY = new double[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly double[] _effectiveThrottle = CreateEffectiveThrottle();
        private static readonly bool[] _lastTriggerPressed = new bool[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly long[] _lastTriggerRisingMs = new long[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly string[] _baselineProfile = new string[Global.TEST_PROFILE_ITEM_COUNT];

        // Haptic state per device
        private static readonly double[] _hapticPrevAmps = InitHapticPrevAmps();
        private static readonly long[] _hapticPulseEndMs = new long[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly byte[] _hapticPulseLight = new byte[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly long[] _hapticNextPulseMs = new long[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly bool[] _hapticWasActive = new bool[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly long[] _hapticMaxSinceMs = new long[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly double[] _hapticAccelRefAmps = InitHapticPrevAmps();
        private static readonly long[] _hapticAccelRefMs = new long[Global.TEST_PROFILE_ITEM_COUNT];
        private static readonly int[] _hapticAccelLastStep = new int[Global.TEST_PROFILE_ITEM_COUNT];

        // GyroFlight: L3 rising-edge detection for throttle reset
        private static readonly bool[] _lastL3 = new bool[Global.TEST_PROFILE_ITEM_COUNT];
        // GyroFlight: elapsed time tracking for LS Y rate accumulator
        private static readonly long[] _lastOverrideMs = new long[Global.TEST_PROFILE_ITEM_COUNT];

        private static double[] InitHapticPrevAmps()
        {
            var a = new double[Global.TEST_PROFILE_ITEM_COUNT];
            for (int i = 0; i < a.Length; i++) a[i] = 16.0;
            return a;
        }

        private static FlightMode GetFlightMode(int deviceNum)
        {
            string profile = SafeGetProfile(deviceNum);
            if (IsProfileNamed(profile, ProfileStickFlight)) return FlightMode.StickFlight;
            if (IsProfileNamed(profile, ProfileGyroFlight))  return FlightMode.GyroFlight;
            return FlightMode.None;
        }

        public static bool IsActive(int deviceNum) => GetFlightMode(deviceNum) != FlightMode.None;

        private static double[] CreateEffectiveThrottle()
        {
            var arr = new double[Global.TEST_PROFILE_ITEM_COUNT];
            for (int i = 0; i < arr.Length; i++) arr[i] = 0.5;
            return arr;
        }

        // Reforger's gamepad stick response curve, reverse-engineered at
        // hover (16A, env=1.0, EnableYawBoost=false) by varying lsSens
        // and measuring heading-based yaw RPM over 5s windows.
        // f(norm) = RPM / 91.464, normalized so f(1.0) = 1.0.
        // Also used for trigger → m_iThrottle conversion (same curve).
        private static readonly (double norm, double f)[] _gameStickCurve = new (double, double)[]
        {
            (0.000, 0.00000),   //                       (origin)
            (0.244, 0.00000),   // sens=0.25    0.0 RPM  (game deadzone)
            (0.299, 0.04504),   // sens=0.30    4.1 RPM
            (0.394, 0.09141),   // sens=0.40    8.4 RPM
            (0.496, 0.14350),   // sens=0.50   13.1 RPM
            (0.598, 0.19621),   // sens=0.60   17.9 RPM
            (0.693, 0.24895),   // sens=0.70   22.8 RPM
            (0.748, 0.28808),   // sens=0.75   26.3 RPM  ← hover baseline
            (0.795, 0.32937),   // sens=0.80   30.1 RPM
            (0.819, 0.35446),   // sens=0.82   32.4 RPM
            (0.827, 0.36342),   // sens=0.83   33.2 RPM
            (0.843, 0.38207),   // sens=0.85   34.9 RPM
            (0.858, 0.40234),   // sens=0.86   36.8 RPM
            (0.866, 0.41324),   // sens=0.87   37.8 RPM
            (0.874, 0.42417),   // sens=0.88   38.8 RPM
            (0.890, 0.44783),   // sens=0.89   41.0 RPM
            (0.898, 0.46029),   // sens=0.90   42.1 RPM
            (0.906, 0.47339),   // sens=0.91   43.3 RPM
            (0.913, 0.48676),   // sens=0.92   44.5 RPM
            (0.921, 0.52155),   // sens=0.93   47.7 RPM
            (0.937, 0.55448),   // sens=0.94   50.7 RPM
            (0.945, 0.59757),   // sens=0.95   54.7 RPM
            (0.953, 0.64903),   // sens=0.96   59.4 RPM
            (0.969, 0.76556),   // sens=0.97   70.0 RPM
            (0.976, 0.82720),   // sens=0.98   75.7 RPM
            (0.984, 0.88803),   // sens=0.99   81.2 RPM
            (1.000, 1.00000),   // sens=1.00   91.5 RPM
        };

        /// Look up f(norm) by piecewise-linear interpolation.
        private static double LookupGameCurve(double norm)
        {
            var tbl = _gameStickCurve;
            if (norm <= tbl[0].norm) return tbl[0].f;
            int last = tbl.Length - 1;
            if (norm >= tbl[last].norm) return tbl[last].f;
            for (int i = 1; i <= last; i++)
            {
                if (norm <= tbl[i].norm)
                {
                    double frac = (norm - tbl[i - 1].norm) / (tbl[i].norm - tbl[i - 1].norm);
                    return tbl[i - 1].f + frac * (tbl[i].f - tbl[i - 1].f);
                }
            }
            return tbl[last].f;
        }

        /// Inverse: given a target f value, find the norm that produces it.
        private static double InverseGameCurve(double targetF)
        {
            var tbl = _gameStickCurve;
            if (targetF <= tbl[0].f) return tbl[0].norm;
            int last = tbl.Length - 1;
            if (targetF >= tbl[last].f) return tbl[last].norm;
            for (int i = 1; i <= last; i++)
            {
                if (targetF <= tbl[i].f)
                {
                    double frac = (targetF - tbl[i - 1].f) / (tbl[i].f - tbl[i - 1].f);
                    return tbl[i - 1].norm + frac * (tbl[i].norm - tbl[i - 1].norm);
                }
            }
            return tbl[last].norm;
        }

        // The hover-baseline norm: (lsSens * 1.0) applied to full-stick
        // deflection of 127 from center, i.e. 0.75 * 127 / 127 = 0.75.
        // Hardcoded rather than read from the profile to keep the table
        // self-consistent with the measurements above.
        private const double HoverNorm = 0.748;

        /// Runtime-only multiplier applied on top of the configured
        /// LSSens, only on LS X, to cancel the mod's yawEnvelope while
        /// respecting the game's non-linear stick response curve.
        ///
        /// For each env, finds the stick norm whose game-curve output,
        /// multiplied by env, equals the hover baseline output. Returns
        /// the ratio of that norm to HoverNorm as the boost factor.
        ///
        /// When the needed norm exceeds 1.0 (env too low to compensate),
        /// the boost is capped at 1.0/HoverNorm ≈ 1.337 — the byte
        /// saturates at 255 and yaw will be above baseline there.
        ///
        /// LS Y (pitch/roll) is explicitly NOT boosted (Mapping.cs hook).
        /// Returns 1.0 when not on the ARMA profile.
        public static double GetYawBoost(int deviceNum)
        {
            if (!EnableYawBoost) return 1.0;
            if (!IsActive(deviceNum)) return 1.0;
            if (deviceNum < 0 || deviceNum >= _effectiveThrottle.Length) return 1.0;

            double throttle = _effectiveThrottle[deviceNum];
            double env = Math.Max(0.3, 1.0 - Math.Abs(2.0 * throttle - 1.0));

            double hoverF = LookupGameCurve(HoverNorm);
            double neededF = hoverF / env;
            double neededNorm = InverseGameCurve(neededF);
            double boost = neededNorm / HoverNorm;
            return Math.Min(boost, 1.0 / HoverNorm);
        }

        /// Replaces the final axisYOut byte computation in
        /// SixMouseStick. Integrates the gyro-derived magnitude into
        /// the accumulator. Returns 128/128 as the stick bytes so the
        /// downstream magnitude-wins write in SixMouseStick becomes a
        /// no-op (we don't want gyro driving any virtual stick axis).
        public static bool IntegrateSixMouseStick(int deviceNum,
                                                  double xNorm, double yNorm,
                                                  int signX, int signY,
                                                  double elapsedSec,
                                                  bool outputX, bool outputY,
                                                  out byte axisXOut,
                                                  out byte axisYOut)
        {
            if (!IsActive(deviceNum))
            {
                axisXOut = axisYOut = 128;
                return false;
            }

            RefreshBaseline(deviceNum);

            if (outputY)
            {
                _accY[deviceNum] = Math.Clamp(
                    _accY[deviceNum] + yNorm * signY * elapsedSec * AccumulateRate,
                    -1.0, 1.0);
            }

            // Tell SixMouseStick "stick at center" so no stick axis is
            // touched. Throttle is expressed on L2/R2 via OverrideTriggers.
            axisXOut = 128;
            axisYOut = 128;
            return true;
        }

        public static bool IsGyroFlight(int deviceNum) =>
            GetFlightMode(deviceNum) == FlightMode.GyroFlight;

        /// GyroFlight mode: maps all three gyro axes proportionally to
        /// virtual sticks. Yaw → LS X, Roll → RS X, Pitch → RS Y.
        /// Called from Mouse.sixaxisMoved INSTEAD of SixMouseStick when
        /// the ARMA_FPV profile is active.
        public static void ProcessGyroFlight(int deviceNum, SixAxis sixAxis)
        {
            RefreshBaseline(deviceNum);

            GyroMouseStickInfo msinfo = Global.GetGyroMouseStickInfo(deviceNum);
            int dz = msinfo.deadZone;
            int maxZ = msinfo.maxZone;
            double antiDZ = msinfo.antiDeadX;

            int rawYaw   = sixAxis.gyroYawFull;
            int rawPitch = sixAxis.gyroPitchFull;
            int rawRoll  = sixAxis.gyroRollFull;

            byte yawByte   = GyroAxisToByte(rawYaw,   dz, maxZ, antiDZ, GF_YawScale);
            byte pitchByte = GyroAxisToByte(rawPitch, dz, maxZ, antiDZ, GF_PitchScale);
            byte rollByte  = GyroAxisToByte(rawRoll,  dz, maxZ, antiDZ, GF_RollScale);

            Mapping.gyroStickX[deviceNum] = yawByte;
            Mapping.gyroRStickX[deviceNum] = rollByte;
            Mapping.gyroRStickY[deviceNum] = pitchByte;

            var mapData = Mapping.mapStickActionData[deviceNum];
            if (Math.Abs(yawByte - 128) > Math.Abs(mapData.LX - 128))
            {
                mapData.LX = yawByte;
                mapData.dirty = true;
            }
            if (Math.Abs(rollByte - 128) > Math.Abs(mapData.RX - 128))
            {
                mapData.RX = rollByte;
                mapData.dirty = true;
            }
            if (Math.Abs(pitchByte - 128) > Math.Abs(mapData.RY - 128))
            {
                mapData.RY = pitchByte;
                mapData.dirty = true;
            }
        }

        private static byte GyroAxisToByte(int rawDelta, int deadzone, int maxZone,
                                           double antiDZ, double scale)
        {
            int absDelta = Math.Abs(rawDelta);
            if (absDelta <= deadzone) return 128;
            int sign = Math.Sign(rawDelta);
            int clipped = Math.Min(absDelta, maxZone);
            double ratio = (double)(clipped - deadzone) / (maxZone - deadzone);
            ratio = Math.Min(ratio * scale, 1.0);
            double norm = antiDZ + (1.0 - antiDZ) * ratio;
            return (byte)(128 + sign * norm * 127);
        }

        /// Called from Mouse.sixaxisMoved with the raw R1 state.
        /// Snaps the accumulator to hover on double-click.
        public static void ObserveTrigger(int deviceNum, bool triggerPressed)
        {
            if (!IsActive(deviceNum)) return;
            if (deviceNum < 0 || deviceNum >= _lastTriggerPressed.Length) return;

            bool prev = _lastTriggerPressed[deviceNum];
            _lastTriggerPressed[deviceNum] = triggerPressed;

            if (prev || !triggerPressed) return;  // rising edge only

            long nowMs = Environment.TickCount64;
            long lastMs = _lastTriggerRisingMs[deviceNum];

            if (lastMs != 0 && (nowMs - lastMs) < DoubleClickWindowMs)
            {
                _accY[deviceNum] = 0.0;
                _effectiveThrottle[deviceNum] = 0.5;
                _lastTriggerRisingMs[deviceNum] = 0;
            }
            else
            {
                _lastTriggerRisingMs[deviceNum] = nowMs;
            }
        }

        /// Hook from Mapping.cs right after outputfieldMapping.PopulateState.
        /// Max-merges accumulator-derived analog trigger values with
        /// whatever the normal mapping pipeline already produced.
        ///
        /// Direction convention (confirmed via HUD observation):
        ///   accY > 0 → virtual R2 (Reforger RT = Thrust Down, HUD A drops)
        ///   accY < 0 → virtual L2 (Reforger LT = Thrust Up,  HUD A rises)
        ///   accY = 0 → both 0 = hover
        ///
        /// Using max-wins (not unconditional overwrite) means physical
        /// button → virtual trigger remaps — e.g. L1 mapped to "Left
        /// Trigger (max)" for instant full throttle up — still take
        /// effect. When the accumulator is at hover the remapped button
        /// wins; when the remapped button is not pressed the accumulator
        /// wins. Only active on ARMA profile; no-op elsewhere.
        public static void OverrideTriggers(int deviceNum, DS4State cState, DS4State mapState)
        {
            FlightMode mode = GetFlightMode(deviceNum);
            if (mode == FlightMode.None) return;
            if (mapState == null) return;

            // GyroFlight: LS Y rate → accumulator, suppress LY output
            if (mode == FlightMode.GyroFlight)
            {
                long nowMs = Environment.TickCount64;
                long prevMs = _lastOverrideMs[deviceNum];
                _lastOverrideMs[deviceNum] = nowMs;
                double dt = prevMs > 0 ? (nowMs - prevMs) / 1000.0 : 0.004;
                dt = Math.Min(dt, 0.05);

                double lyNorm = (mapState.LY - 128) / 127.0;
                if (Math.Abs(lyNorm) > GF_ThrottleDeadzone)
                {
                    double sign = Math.Sign(lyNorm);
                    double mag = (Math.Abs(lyNorm) - GF_ThrottleDeadzone)
                               / (1.0 - GF_ThrottleDeadzone);
                    _accY[deviceNum] = Math.Clamp(
                        _accY[deviceNum] + sign * mag * GF_ThrottleRate * dt,
                        -1.0, 1.0);
                }

                // L3 click → snap to hover
                bool l3Now = cState != null && cState.L3;
                if (l3Now && !_lastL3[deviceNum])
                {
                    _accY[deviceNum] = 0.0;
                    _effectiveThrottle[deviceNum] = 0.5;
                }
                _lastL3[deviceNum] = l3Now;

                // Suppress LS Y from reaching the game as DroneUp
                mapState.LY = 128;
                if (cState != null) cState.LY = 128;
            }

            // Merge accumulator-derived triggers (shared by both modes)
            double accum = _accY[deviceNum];
            byte accumR2 = 0;
            byte accumL2 = 0;
            if (accum > 0.0) accumR2 = AccumMagToTriggerByte(accum);
            else if (accum < 0.0) accumL2 = AccumMagToTriggerByte(-accum);

            if (accumR2 > mapState.R2) mapState.R2 = accumR2;
            if (accumL2 > mapState.L2) mapState.L2 = accumL2;

            if (cState != null)
            {
                if (mapState.R2 > cState.R2) cState.R2 = mapState.R2;
                if (mapState.L2 > cState.L2) cState.L2 = mapState.L2;
            }

            double l2DeltaT = LookupDeltaT(mapState.L2);
            double r2DeltaT = LookupDeltaT(mapState.R2);
            double signedDeltaT = l2DeltaT - r2DeltaT;
            _effectiveThrottle[deviceNum] = Math.Clamp(0.5 + signedDeltaT, 0.0, 1.0);
        }

        // Trigger and stick share the same engine curve shape, but the
        // trigger deadzone starts at ~54/255 (0.212) while the stick DZ
        // starts at 0.244.  We remap trigger norm through the trigger DZ
        // so that bytes 56+ (first non-zero accumulator output) produce
        // non-zero dT, matching the game's actual throttle response.
        private const double TriggerDeadZone = 0.212;   // ~54/255, measured
        private const double StickDeadZone   = 0.244;   // first non-zero in _gameStickCurve

        /// Returns |m_iThrottle - 0.5| for a given trigger byte.
        private static double LookupDeltaT(byte b)
        {
            double trigNorm = b / 255.0;
            if (trigNorm <= TriggerDeadZone) return 0.0;
            double postDZ    = (trigNorm - TriggerDeadZone) / (1.0 - TriggerDeadZone);
            double stickEquiv = StickDeadZone + postDZ * (1.0 - StickDeadZone);
            return LookupGameCurve(stickEquiv) / 2.0;
        }

        /// Haptic feedback driven by gyro throttle position. Call once
        /// per frame after OverrideTriggers (which updates _effectiveThrottle).
        /// Above hover: periodic short buzz whose intensity tracks amps.
        /// Below hover: continuous light rumble ramping with descent.
        public static void UpdateHaptics(int deviceNum, ControlService ctrl)
        {
            if (!EnableHaptics || !IsActive(deviceNum)) return;
            if (deviceNum < 0 || deviceNum >= _effectiveThrottle.Length) return;

            double currentAmps = _effectiveThrottle[deviceNum] * 32.0;
            long nowMs = Environment.TickCount64;

            double aboveEdge = HapticHoverAmps + HapticDeadBandAmps;

            // Max-throttle silence: track how long we've been pegged at max.
            bool atMax = currentAmps >= MaxThrottleAmps;
            if (atMax)
            {
                if (_hapticMaxSinceMs[deviceNum] == 0)
                    _hapticMaxSinceMs[deviceNum] = nowMs;
            }
            else
            {
                _hapticMaxSinceMs[deviceNum] = 0;
            }
            bool maxSilenced = atMax
                && (nowMs - _hapticMaxSinceMs[deviceNum]) >= MaxThrottleSilenceMs;

            // Above hover: periodic pulse, suppressed once max-silenced.
            if (!maxSilenced && currentAmps > aboveEdge
                && nowMs >= _hapticNextPulseMs[deviceNum])
            {
                double t = Math.Clamp((currentAmps - aboveEdge) / (32.0 - aboveEdge), 0.0, 1.0);
                byte intensity = (byte)(t * AboveHoverMaxIntensity);
                if (intensity > 0)
                {
                    _hapticPulseEndMs[deviceNum] = nowMs + (long)(AboveHoverPulseDurationSec * 1000);
                    _hapticPulseLight[deviceNum] = intensity;
                }
                _hapticNextPulseMs[deviceNum] = nowMs + AboveHoverPulseIntervalMs;
            }
            else if (currentAmps <= aboveEdge || maxSilenced)
            {
                _hapticNextPulseMs[deviceNum] = 0;
            }

            // Acceleration-dependent threshold buzz: every AccelBuzzWindowMs
            // check whether we crossed a step boundary while actively climbing.
            long accelElapsed = nowMs - _hapticAccelRefMs[deviceNum];
            if (accelElapsed >= AccelBuzzWindowMs)
            {
                double rateSec = accelElapsed / 1000.0;
                double rate = rateSec > 0
                    ? (currentAmps - _hapticAccelRefAmps[deviceNum]) / rateSec
                    : 0.0;

                int curStep = currentAmps > AccelBuzzAboveAmps
                    ? (int)((currentAmps - AccelBuzzAboveAmps) / AccelBuzzStepAmps)
                    : 0;

                if (rate >= AccelBuzzMinRate && curStep > _hapticAccelLastStep[deviceNum])
                {
                    _hapticPulseEndMs[deviceNum] = nowMs + (long)(AccelBuzzDurationSec * 1000);
                    _hapticPulseLight[deviceNum] = AccelBuzzIntensity;
                }

                _hapticAccelLastStep[deviceNum] = curStep;
                _hapticAccelRefAmps[deviceNum] = currentAmps;
                _hapticAccelRefMs[deviceNum] = nowMs;
            }

            // Below hover: continuous rumble, no dead band — kicks in immediately.
            byte continuousLight = 0;
            if (currentAmps < HapticHoverAmps)
            {
                double t = Math.Clamp((HapticHoverAmps - currentAmps) / (HapticHoverAmps - BelowHoverFloorAmps), 0.0, 1.0);
                continuousLight = (byte)Math.Max(t * BelowHoverMaxIntensity, 1);
            }

            byte pulseLight = nowMs < _hapticPulseEndMs[deviceNum]
                ? _hapticPulseLight[deviceNum] : (byte)0;

            byte finalLight = Math.Max(pulseLight, continuousLight);

            if (finalLight > 0)
            {
                ctrl.setRumble(0, finalLight, deviceNum);
                _hapticWasActive[deviceNum] = true;
            }
            else if (_hapticWasActive[deviceNum])
            {
                ctrl.setRumble(0, 0, deviceNum);
                _hapticWasActive[deviceNum] = false;
            }
        }

        /// Throttle in [0, 1] for UI readout, taking physical-button
        /// → trigger remaps into account (not just gyro accumulator).
        public static double GetThrottle(int deviceNum)
        {
            return (deviceNum >= 0 && deviceNum < _effectiveThrottle.Length)
                ? _effectiveThrottle[deviceNum]
                : 0.5;
        }

        public static void ResetThrottle(int deviceNum)
        {
            if (deviceNum < 0 || deviceNum >= _accY.Length) return;
            _accY[deviceNum] = 0.0;
            _effectiveThrottle[deviceNum] = 0.5;
        }

        private static void RefreshBaseline(int deviceNum)
        {
            string profile = SafeGetProfile(deviceNum);
            if (string.Equals(_baselineProfile[deviceNum], profile, StringComparison.Ordinal))
                return;

            _baselineProfile[deviceNum] = profile;

            if (ResetAccumOnProfileLoad)
            {
                _accY[deviceNum] = 0.0;
                _effectiveThrottle[deviceNum] = 0.5;
            }
        }

        /// Accumulator magnitude (0..1) → trigger byte (0..255) with
        /// output anti-deadzone so any non-zero magnitude pops straight
        /// past the game's trigger deadzone.
        private static byte AccumMagToTriggerByte(double mag)
        {
            if (mag <= 0.0) return 0;
            double adj = OutputAntiDeadZone + (1.0 - OutputAntiDeadZone) * mag;
            return (byte)Math.Clamp(Math.Round(adj * 255.0), 0.0, 255.0);
        }

        private static string SafeGetProfile(int deviceNum)
        {
            string[] profiles = Global.ProfilePath;
            return (profiles != null && deviceNum < profiles.Length)
                ? profiles[deviceNum] : null;
        }

        private static bool IsProfileNamed(string profilePath, string target)
        {
            if (string.IsNullOrEmpty(profilePath)) return false;
            string name = Path.GetFileNameWithoutExtension(profilePath);
            return string.Equals(name, target, StringComparison.OrdinalIgnoreCase);
        }
    }
}