import math
import numpy as np
import matplotlib.pyplot as plt

# Create the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Constants
ky = 2
km = 3

# Motor parameters
w       = np.array([1,1,1,1])
# w       = np.array([2,2,1,1])
# w       = np.array([2,1,1,1])
theta   = np.array([0, 0, 0, 0])
# theta   = np.array([180, 180, 0, 0])
# theta   = np.array([180, 180, 180, 180])
# theta   = np.array([-45, 45, 0, 0])
# theta   = np.array([45, -45, 0, 0])
# theta   = np.array([0, 90, 0, 0])
# theta   = np.array([90, 90, 90, 90])
# theta   = np.array([-90, 90, -90, 90])

# Convert thetha to rad
theta = np.radians(theta)

# Thrust and torque forces generated by each motor
F = ky * w
T = km * w

# Include negatives in the torque equations
T[1] = T[1] * -1
T[3] = T[3] * -1

# Motor 1
Fx1 = -F[0] * math.sin(theta[0])
Fy1 = 0
Fz1 = F[0] * math.cos(theta[0])
Tx1 = -T[0] * math.sin(theta[0])
Ty1 = 0
Tz1 = T[0] * math.cos(theta[0])

# Motor 2
Fx2 = -F[1] * math.sin(theta[1])
Fy2 = 0
Fz2 = F[1] * math.cos(theta[1])
Tx2 = -T[1] * math.sin(theta[1])
Ty2 = 0
Tz2 = T[1] * math.cos(theta[1])

# Motor 3
Fx3 = 0
Fy3 = -F[2] * math.sin(theta[2])
Fz3 = F[2] * math.cos(theta[2])
Tx3 = 0
Ty3 = -T[2] * math.sin(theta[2])
Tz3 = T[2] * math.cos(theta[2])

# Motor 4
Fx4 = 0
Fy4 = -F[3] * math.sin(theta[3])
Fz4 = F[3] * math.cos(theta[3])
Tx4 = 0
Ty4 = -T[3] * math.sin(theta[3])
Tz4 = T[3] * math.cos(theta[3])

# Compute the sum of forces
Fx = Fx1 + Fx2 + Fx3 + Fx4
Fy = Fy1 + Fy2 + Fy3 + Fy4
Fz = Fz1 + Fz2 + Fz3 + Fz4

# Torque
Tx = 0
Ty = 0
Tz = 0
# Force from rotation
Tx += Tx1 + Tx2 + Tx3 + Tx4
Ty += Ty1 + Ty2 + Ty3 + Ty4
Tz += Tz1 + Tz2 + Tz3 + Tz4
# Force from lateral movement
Tx += (Fz1 - Fz2) + (Fy1 - Fy2)
Ty += (Fz3 - Fz4) + (Fx3 - Fx4)
Tz += (Fx1 - Fx2) + (Fy3 - Fy4)

# Plot lateral forces
main        = np.array([[0, 0, 0, Fx, Fy, Fz]])
original    = np.array([[0, -1, 0, Fx1, Fy1, Fz1], [0, 1, 0, Fx2, Fy2, Fz2], [1, 0, 0, Fx3, Fy3, Fz3], [-1, 0, 0, Fx4, Fy4, Fz4]])
X1, Y1, Z1, U1, V1, W1 = zip(*main)
X2, Y2, Z2, U2, V2, W2 = zip(*original)
ax.quiver(X1, Y1, Z1, U1, V1, W1, color='C0')
ax.quiver(X2, Y2, Z2, U2, V2, W2, color='C1')

# Plot torque forces
plotter = np.linspace(0, 2*np.pi, 201)
Tx_plot = [0*plotter, Tx*np.cos(plotter), Tx*np.sin(plotter)]
Ty_plot = [Ty*np.cos(plotter), 0*plotter, Ty*np.sin(plotter)]
Tz_plot = [Tz*np.sin(plotter), Tz*np.cos(plotter), 0*plotter]
ax.plot(Tx_plot[0], Tx_plot[1], Tx_plot[2], color='C3' if Tx < 0 else 'C2')
ax.plot(Ty_plot[0], Ty_plot[1], Ty_plot[2], color='C3' if Ty < 0 else 'C2')
ax.plot(Tz_plot[0], Tz_plot[1], Tz_plot[2], color='C3' if Tz < 0 else 'C2')

# Set plot properties
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])
ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')

# Display plot
plt.show()