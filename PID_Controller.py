import time


class PIDController:
    """
    Simple PID controller for error → velocity conversion.

    Takes pixel error, outputs velocity command.
    """

    def __init__(self, kp=0.01, ki=0.0, kd=0.05, max_output=1.0, min_output=-1.0):
        """
        Args:
            kp: Proportional gain (response to current error)
            ki: Integral gain (correction for steady-state error)
            kd: Derivative gain (damping, prevents oscillation)
            max_output: Max velocity command (m/s)
            min_output: Min velocity command (m/s)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def update(self, error):
        """
        Compute PID output for given error.

        Args:
            error: Current error (pixels or similar)

        Returns:
            velocity command: float in range [min_output, max_output]
        """
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0:
            dt = 0.001  # Prevent division by zero

        # Proportional term
        p_term = self.kp * error

        # Integral term (accumulate error over time)
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term (rate of change)
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Total output
        output = p_term + i_term + d_term

        # Clamp to valid range
        output = max(self.min_output, min(self.max_output, output))

        # Update state
        self.last_error = error
        self.last_time = current_time

        return output

    def reset(self):
        """Reset integral and derivative state (e.g., when switching modes)"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

    def __repr__(self):
        return f"PID(kp={self.kp}, ki={self.ki}, kd={self.kd})"