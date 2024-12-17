from constants import *
from math import fabs, sin, cos
from rocket import Rocket
from control import FullPIDController, FullPDController
import matplotlib.pyplot as plt
import sys

def main():
    """Performs a simulation controlled by the user,
    so the rocket dynamics can be tested and explored"""
    speedCtrl = False
    windZ = 0
    windX = 0
    speed = FullPIDController(D_TIME, 0, MAX_THRUST)
    pos = FullPDController(D_TIME, pi/96)
    theta = FullPDController(D_TIME, MAX_NOZZLE_ANGLE)
    rocket = Rocket(locX = 0)
    rocket.set_controllers(speed_ctrl = speed, position_ctrl = pos, theta_ctrl = theta, speedCtrl = speedCtrl)
    rocket.playable = False
    run = True
    theta_r = []
    x = []
    x_r = []
    theta = []
    thrust = []
    alpha = []
    sz = []
    szr = []
    max_time = 50
    XR = 10
    Z_input = 100
    print("Running simulation...")
    for i in range(int(max_time//D_TIME)):
        if i > max_time/D_TIME/2:
            Z_input += 0.5
        t = rocket.applyCommand(Z_input, XR, windX, windZ)
        theta_r.append(t)
        alpha.append(rocket.nozzleAngle)
        thrust.append(rocket.thrust)
        x_r.append(XR)
        rocket.move(windX, windZ)
        x.append(rocket.locX)
        theta.append(rocket.theta)
        if speedCtrl:
            sz.append(rocket.speedZ)
        else:
            sz.append(rocket.locZ)
        szr.append(Z_input)

    print("simulation finished")
# Sample data (replace with actual data)
    t = [i*D_TIME for i in range(len(x))]  # Time

# Plot 1: theta_r and theta vs t
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 2, 1)
    plt.plot(t, theta_r, label="theta_r", color="blue")
    plt.plot(t, theta, label="theta", color="red")
    plt.xlabel("Time (t)")
    plt.ylabel("Angle (degrees)")
    plt.title("Theta_r and Theta vs Time")
    plt.legend()
    plt.grid()

# Plot 2: x_r and x vs t
    plt.subplot(2, 2, 2)
    plt.plot(t, x_r, label="x_r", color="green")
    plt.plot(t, x, label="x", color="orange")
    plt.xlabel("Time (t)")
    plt.ylabel("Position (units)")
    plt.title("X_r and X vs Time")
    plt.legend()
    plt.grid()

# Plot 3: thrust vs t
    plt.subplot(2, 2, 3)
    plt.plot(t, thrust, label="Thrust", color="purple")
    plt.xlabel("Time (t)")
    plt.ylabel("Thrust (units)")
    plt.title("Thrust vs Time")
    plt.grid()

# Plot 4: alpha vs t
    plt.subplot(2, 2, 4)
    plt.plot(t, alpha, label="Alpha", color="brown")
    plt.xlabel("Time (t)")
    plt.ylabel("Alpha (units)")
    plt.title("Alpha vs Time")
    plt.grid()

# Adjust layout and show the plots
    plt.tight_layout()
    plt.show()
    label = "Z speed" if speedCtrl else "Z position"
    plt.plot(t, sz, label=label, color = "red")
    plt.plot(t, szr, label="Reference" + label, color = 'blue')
    plt.title(label)
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
    sys.exit()
