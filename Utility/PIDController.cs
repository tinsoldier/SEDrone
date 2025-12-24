using System;
using VRageMath;

namespace IngameScript
{

    /// <summary>
    /// A PID (Proportional-Integral-Derivative) controller for managing control systems.
    /// </summary>
    /// <summary>
    /// A PID controller implementation suitable for hover thrust control.
    /// Error should be the difference between the target and current measurement.
    /// 
    /// Kp: proportional gain
    /// Ki: integral gain
    /// Kd: derivative gain
    ///
    /// Integral is computed over time, derivative is computed based on error change per unit time.
    /// stepsPerSecond should match your update rate (e.g., 60 for 60 ticks/sec).
    ///
    /// integralUpperLimit and integralLowerLimit can be used to prevent integral windup.
    /// </summary>
    public class PIDController
    {
        private readonly double _proportionalGain;
        private readonly double _integralGain;
        private readonly double _derivativeGain;
        private readonly double _integralUpperLimit;
        private readonly double _integralLowerLimit;

        private double _integral;
        private double _previousError;

        /// <summary>
        /// Initializes a new instance of the <see cref="PIDController"/> class with specified gains and integral limits.
        /// </summary>
        /// <param name="proportionalGain">Proportional gain (Kp)</param>
        /// <param name="integralGain">Integral gain (Ki)</param>
        /// <param name="derivativeGain">Derivative gain (Kd)</param>
        /// <param name="integralUpperLimit">Optional upper limit for the integral term (0 means no limit)</param>
        /// <param name="integralLowerLimit">Optional lower limit for the integral term (0 means no limit)</param>
        public PIDController(
            double proportionalGain,
            double integralGain,
            double derivativeGain,
            double integralUpperLimit = 0,
            double integralLowerLimit = 0)
        {
            _proportionalGain = proportionalGain;
            _integralGain = integralGain;
            _derivativeGain = derivativeGain;
            _integralUpperLimit = integralUpperLimit;
            _integralLowerLimit = integralLowerLimit;
            _integral = 0;
            _previousError = 0;
        }

        /// <summary>
        /// Computes the PID controller output based on the current error.
        /// error = (target - current)
        /// </summary>
        /// <param name="error">The current error value for the controller.</param>
        /// <returns>The computed PID output.</returns>
        public double Compute(double error, double deltaTime)
        {
            // Update integral with actual elapsed time
            _integral += error * deltaTime;

            // Apply integral limits if set (non-zero means limit is active)
            if (_integralUpperLimit != 0)
                _integral = Math.Min(_integral, _integralUpperLimit);
            if (_integralLowerLimit != 0)
                _integral = Math.Max(_integral, _integralLowerLimit);

            // Compute derivative based on error change per elapsed time
            double derivative = (error - _previousError) / deltaTime;

            _previousError = error;

            double output = (_proportionalGain * error)
                            + (_integralGain * _integral)
                            + (_derivativeGain * derivative);

            return output;
        }


        /// <summary>
        /// Resets the PID controller by clearing the integral and previous error.
        /// </summary>
        public void Reset()
        {
            _integral = 0;
            _previousError = 0;
        }
    }

    /// <summary>
    /// A 3D PID controller that wraps three independent PIDController instances for X, Y, and Z axes.
    /// Useful for position, velocity, or rotation control in 3D space.
    /// </summary>
    public class PIDController3D
    {
        private readonly PIDController _x;
        private readonly PIDController _y;
        private readonly PIDController _z;

        /// <summary>
        /// Initializes a new PIDController3D with the same gains for all axes.
        /// </summary>
        /// <param name="gains">PID gains to apply to all three axes</param>
        public PIDController3D(PIDGains gains)
        {
            _x = new PIDController(gains.Kp, gains.Ki, gains.Kd, gains.IntegralLimit, -gains.IntegralLimit);
            _y = new PIDController(gains.Kp, gains.Ki, gains.Kd, gains.IntegralLimit, -gains.IntegralLimit);
            _z = new PIDController(gains.Kp, gains.Ki, gains.Kd, gains.IntegralLimit, -gains.IntegralLimit);
        }

        /// <summary>
        /// Initializes a new PIDController3D with individual gains for each axis.
        /// </summary>
        public PIDController3D(PIDGains xGains, PIDGains yGains, PIDGains zGains)
        {
            _x = new PIDController(xGains.Kp, xGains.Ki, xGains.Kd, xGains.IntegralLimit, -xGains.IntegralLimit);
            _y = new PIDController(yGains.Kp, yGains.Ki, yGains.Kd, yGains.IntegralLimit, -yGains.IntegralLimit);
            _z = new PIDController(zGains.Kp, zGains.Ki, zGains.Kd, zGains.IntegralLimit, -zGains.IntegralLimit);
        }

        /// <summary>
        /// Initializes a new PIDController3D with explicit gain values for all axes.
        /// </summary>
        public PIDController3D(double kp, double ki, double kd, double integralLimit = 100)
        {
            _x = new PIDController(kp, ki, kd, integralLimit, -integralLimit);
            _y = new PIDController(kp, ki, kd, integralLimit, -integralLimit);
            _z = new PIDController(kp, ki, kd, integralLimit, -integralLimit);
        }

        /// <summary>
        /// Computes the PID output for a 3D error vector.
        /// </summary>
        /// <param name="error">The error vector (target - current) in 3D space</param>
        /// <param name="deltaTime">Time elapsed since last update in seconds</param>
        /// <returns>A Vector3D containing the PID output for each axis</returns>
        public Vector3D Compute(Vector3D error, double deltaTime)
        {
            return new Vector3D(
                _x.Compute(error.X, deltaTime),
                _y.Compute(error.Y, deltaTime),
                _z.Compute(error.Z, deltaTime)
            );
        }

        /// <summary>
        /// Resets all three axis controllers, clearing integral and derivative state.
        /// </summary>
        public void Reset()
        {
            _x.Reset();
            _y.Reset();
            _z.Reset();
        }
    }
}