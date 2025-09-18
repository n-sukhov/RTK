
class PID_regulator:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, max_value: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_value = max_value
        self.prev_error = 0.0
        self.integral = 0.0

    def limit_output(func):
        def wrapper(self, *args, **kwargs):
            result = func(self, *args, **kwargs)
            return max(min(result, self.max_value), -self.max_value)
        return wrapper

    @limit_output
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    @limit_output
    def get_p_control_signal(self, current_value: float, goal_value: float) -> float:
        return self.kp * (goal_value - current_value)

    @limit_output
    def get_i_control_signal(self, current_value: float, goal_value: float) -> float:
        self.integral += (goal_value - current_value) * self.dt
        return self.ki * self.integral

    @limit_output
    def get_d_control_signal(self, current_value: float, goal_value: float) -> float:
        error = goal_value - current_value
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kd * derivative

    @limit_output
    def get_pi_control_signal(self, current_value: float, goal_value: float) -> float:
        return (self.get_p_control_signal(current_value, goal_value) + 
                self.get_i_control_signal(current_value, goal_value))

    @limit_output
    def get_pd_control_signal(self, current_value: float, goal_value: float) -> float:
        return (self.get_p_control_signal(current_value, goal_value) + 
                self.get_d_control_signal(current_value, goal_value))

    @limit_output
    def get_id_control_signal(self, current_value: float, goal_value: float) -> float:
        return (self.get_i_control_signal(current_value, goal_value) + 
                self.get_d_control_signal(current_value, goal_value))

    @limit_output
    def get_pid_control_signal(self, current_value: float, goal_value: float) -> float:
        return (self.get_p_control_signal(current_value, goal_value) +
                self.get_i_control_signal(current_value, goal_value) + 
                self.get_d_control_signal(current_value, goal_value))