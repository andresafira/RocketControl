class PIDController:
    """Discrete Representation of a basic PID Controller,
    based on Tustin transform
    C(s) = Kd.s + Kp + Ki/s"""
    def __init__(self, kp, ki, kd, sample_time, max_command):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.T = sample_time
        self.max_command = max_command
        self.name = "PID_incomplete"

        self.reset()
        self.calculate_factors()
    
    def calculate_factors(self):
        """multiplying factors of the Tustin implementation
        for the control command determination"""
        self.b0 = self.kp + (self.ki * self.T / 2) + (2 * self.kd / self.T)
        self.b1 = self.ki * self.T - (4 * self.kd / self.T)
        self.b2 = -self.kp + (self.ki * self.T / 2) + (2 * self.kd / self.T)

    def reset(self):
        self.ep = 0.0
        self.epp = 0.0
        self.up = 0.0
        self.upp = 0.0

    def control(self, yr, y):
        """Function that calculates the command u[k] for a given
        current error. It considers the previous error inputs and
        given commands, based on the Tustin transform"""
        error = yr - y
        u = self.upp + self.b0*error + self.b1*self.ep + self.b2*self.epp

        # Anti-Windup technique based on command saturation
        u = min(max(u, -self.max_command), self.max_command)

        self.epp = self.ep
        self.ep = error

        self.upp = self.up
        self.up = u

        return u


class PIDFilter:
    """PID pre-filter applied on the reference value to avoid
    unwanted zeros on the transfer function
    F(s) = Ki/(Kd.s^2 + Kp.s + Ki)"""
    def __init__(self, kp, ki, kd, sample_time):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.T = sample_time
        self.name = "PID_Filter"
        
        self.reset()
        self.calculate_factors()

    def calculate_factors(self):
        u0 = 4*self.kd + 2*self.kp*self.T + self.T*self.T*self.ki
        self.u1 = (2*self.T*self.T*self.ki - 8*self.kd)/u0
        self.u2 = (4*self.kd - 2*self.T*self.kp + self.T*self.T*self.ki)/u0
        self.xc = self.ki*self.T*self.T / u0

    def reset(self):
        self.xp = 0.0
        self.xpp = 0.0
        self.up = 0.0
        self.upp = 0.0

    def control(self, xr):
        u = self.xc*(xr + 2*self.xp + self.xpp) - self.u1*self.up - self.u2*self.upp
        self.upp = self.up
        self.up = u
        self.xpp = self.xp
        self.xp = xr

        return u


class FullPIDController:
    """Wraper class that represents the combined effect of the
    entire Controller: PID + Pre-Filter"""
    def __init__(self, kp, ki, kd, sample_time, max_command):
        self.PID = PIDController(kp, ki, kd, sample_time, max_command)
        self.filter = PIDFilter(kp, ki, kd, sample_time)
        self.name = "PID"
    
    def calculate_factors(self):
        self.PID.calculate_factors()
        self.filter.calculate_factors()

    def reset(self):
        self.PID.reset()
        self.filter.reset()

    def control(self, yr, y):
        """Control function that applies the filter to the input
        value and then performs the PID Control on the new reference"""
        yr_f = self.filter.control(yr)
        return self.PID.control(yr_f, y)


class PFController:
    """P+F controller for the velocity system, implemented directly"""
    def __init__(self, kx, kff, max_command):
        self.kx = kx
        self.kff = kff
        self.max_command = max_command
        self.name = "PF"

    def reset(self):
        # It doesn't involve any additional parameters
        pass
    
    def control(self, yr, y):
        # applying Proportional + Feedforward control and also
        # the command saturation
        u = self.kx * (yr - y) + self.kff * yr
        return min(max(u, -self.max_command), self.max_command)
