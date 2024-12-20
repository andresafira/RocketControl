class PIDController:
    """Discrete Representation of a basic PID Controller,
    based on Tustin transform
    C(s) = Kd.s + Kp + Ki/s"""
    def __init__(self, kp, ki, kd, sample_time, max_command, min_command):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.T = sample_time
        self.max_command = max_command
        self.min_command = min_command

        self.reset()
        self.calculate_factors()

    def update_constants(self, kp, ki, kd, max_command, min_command):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_command = max_command
        self.min_command = min_command
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
        u = min(max(u, self.min_command), self.max_command)

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
        
        self.reset()
        self.calculate_factors()
    
    def update_constants(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
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
    def __init__(self, sample_time, min_command, max_command, offset_command = 1, kp = 1, ki = 1, kd = 1):
        self.PID = PIDController(kp, ki, kd, sample_time, max_command-offset_command, min_command-offset_command)
        self.filter = PIDFilter(kp, ki, kd, sample_time)
        self.u_max = max_command
        self.u_min = min_command
        self.u0 = offset_command
        self.name = "PID"
    
    def update_constants(self, kp, ki, kd, offset_command):
        self.u0 = offset_command
        self.PID.update_constants(kp, ki, kd, self.u_max-offset_command, self.u_min-offset_command)
        self.filter.update_constants(kp, ki, kd)
    
    def reset(self):
        self.PID.reset()
        self.filter.reset()

    def control(self, yr, y):
        """Control function that applies the filter to the input
        value and then performs the PID Control on the new reference"""
        yr_f = self.filter.control(yr)
        return self.PID.control(yr_f, y) + self.u0

class PDController:
    def __init__(self, kp, kd, sample_time, max_command, min_command):
        self.kp = kp
        self.kd = kd
        self.T = sample_time
        self.max = max_command
        self.min = min_command

        self.reset()
        self.calculate_factors()

    def update_constants(self, kp, kd, max_command, min_command):
        self.kp = kp
        self.kd = kd
        self.max = max_command
        self.min = min_command
        self.calculate_factors()
    
    def calculate_factors(self):
        """multiplying factors of the Tustin implementation
        for the control command determination"""
        self.b0 = (2*self.kd + self.kp*self.T) / self.T
        self.b1 = (self.kp*self.T - 2*self.kd) / self.T

    def reset(self):
        self.ep = 0.0
        self.up = 0.0

    def control(self, yr, y):
        """Function that calculates the command u[k] for a given
        current error. It considers the previous error inputs and
        given commands, based on the Tustin transform"""
        error = yr - y
        u = self.b0 * error + self.b1 * self.ep - self.up
        u = (self.kp + self.kd / self.T) * error - self.kd / self.T * self.ep
        # Anti-Windup technique based on command saturation
        u = min(max(u, self.min), self.max)

        self.ep = error
        self.up = u

        return u

class PDFilter:
    """PD pre-filter applied on the reference value to avoid
    unwanted zeros on the transfer function
    F(s) = Kp/(Kd.s + Kp)"""
    def __init__(self, kp, kd, sample_time):
        self.kp = kp
        self.kd = kd
        self.T = sample_time
        
        self.reset()
        self.calculate_factors()
    
    def update_constants(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.calculate_factors()

    def calculate_factors(self):
        u_denom = 2 * self.kd +self.kp * self.T
        self.u = (2 * self.kd - self.kp * self.T) / u_denom
        self.x = self.T * self.kp / u_denom

    def reset(self):
        self.xp = 0.0
        self.up = 0.0

    def control(self, xr):
        u = self.u * self.up + self.x * (xr + self.xp)
        u = self.kd / (self.kp * self.T + self.kd) * self.up + self.kp / (self.kp + self.kd / self.T) * xr
        
        self.up = u
        self.xp = xr

        return u

class FullPDController:
    """Wraper class that represents the combined effect of the
    entire Controller: PD + Pre-Filter"""
    def __init__(self, sample_time, max_command, offset_command = 1, kp = 1, kd = 1):
        self.PD = PDController(kp, kd, sample_time, max_command-offset_command, -max_command-offset_command)
        self.filter = PDFilter(kp, kd, sample_time)
        self.u_max = max_command
        self.u0 = offset_command
    
    def update_constants(self, kp, kd, offset_command):
        self.u0 = offset_command
        self.PD.update_constants(kp, kd, self.u_max-offset_command, -self.u_max-offset_command)
        self.filter.update_constants(kp, kd)
    
    def reset(self):
        self.PD.reset()
        self.filter.reset()

    def control(self, yr, y):
        """Control function that applies the filter to the input
        value and then performs the PD Control on the new reference"""
        yr_f = self.filter.control(yr)
        return self.PD.control(yr_f, y) + self.u0
