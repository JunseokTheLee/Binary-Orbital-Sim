from vpython import *  # Importing the VPython library for 3D visualization
from numpy import *  # Importing the NumPy library for numerical operations

# Time variables: t0 is the start time, tf is the end time
t0 = 0.; tf = 1000.  
t = t0  # Initializing the current time variable
h = 0.025  # Step size for time increment in the simulation
n = 18  # Number of components in the state vector 'y' and force function 'f'

# Initial positions (r) and velocities (v) of the 3 bodies
r1 = array([10., 0., 0.])  # Position of the first body
v1 = array([-0.1, 0., 0.1])  # Velocity of the first body
m1 = 1.  # Mass of the first body

r2 = array([-10., 0., 0.])  # Position of the second body
v2 = array([0., 0.1, -0.1])  # Velocity of the second body
m2 = 1.  # Mass of the second body

r3 = array([0., 0., 5.])  # Position of the third body
v3 = array([0.2, -0.1, 0.])  # Velocity of the third body
m3 = 1.  # Mass of the third body

# The state vector 'y' contains the positions and velocities of all bodies
y = concatenate((r1, r2, r3, v1, v2, v3))

# Creating a 3D scene to visualize the simulation using VPython
scene = canvas(x=0, y=0, width=700, height=700)  # Window size for the 3D scene

# Curves to represent the paths (or trajectories) of the three bodies
path1 = curve(color=color.blue, radius=0.13)  # Path of body 1
path2 = curve(color=color.red, radius=0.13)   # Path of body 2
path3 = curve(color=color.green, radius=0.13) # Path of body 3

# Spheres to represent the three bodies in the 3D scene
body1 = sphere(pos=vec(r1[0], r1[1], r1[2]), color=color.blue, radius=0.5 * m1**(1/3))  # Body 1
body2 = sphere(pos=vec(r2[0], r2[1], r2[2]), color=color.red, radius=0.5 * m2**(1/3))   # Body 2
body3 = sphere(pos=vec(r3[0], r3[1], r3[2]), color=color.green, radius=0.5 * m3**(1/3)) # Body 3

# Function to calculate the forces acting on the bodies based on their positions
def f(t, y):  # Force function based on gravitational interaction
    # Distances between the three bodies
    r12 = ((y[0] - y[3])**2 + (y[1] - y[4])**2 + (y[2] - y[5])**2)**(1/2)  # Distance between body 1 and body 2
    r23 = ((y[3] - y[6])**2 + (y[4] - y[7])**2 + (y[5] - y[8])**2)**(1/2)  # Distance between body 2 and body 3
    r31 = ((y[6] - y[0])**2 + (y[7] - y[1])**2 + (y[8] - y[2])**2)**(1/2)  # Distance between body 3 and body 1

    # Returning the derivatives of the positions and velocities (Newton's Law of Gravitation)
    return concatenate((y[9:18],
                m2 * (y[3:6] - y[0:3]) / r12**3 + m3 * (y[6:9] - y[0:3]) / r31**3,  # Force on body 1
                m1 * (y[0:3] - y[3:6]) / r12**3 + m3 * (y[6:9] - y[3:6]) / r23**3,  # Force on body 2
                m1 * (y[0:3] - y[6:9]) / r31**3 + m2 * (y[3:6] - y[6:9]) / r23**3)) # Force on body 3

# Runge-Kutta method for numerical integration
def rk4(t, h):  # Fourth-order Runge-Kutta method
    ydumb = zeros(n, float)  # Temporary array for calculations
    k1 = zeros(n, float); k2 = zeros(n, float)
    k3 = zeros(n, float); k4 = zeros(n, float)

    # Calculate the four k-values (steps) for the RK4 algorithm
    for i in range(n):
        k1[i] = h * f(t, y)[i]  # First step
    for i in range(n):
        ydumb[i] = y[i] + k1[i] / 2.  # Update for the second step
    k2 = h * f(t + h/2., ydumb)  # Second step
    for i in range(n):
        ydumb[i] = y[i] + k2[i] / 2.  # Update for the third step
    k3 = h * f(t + h/2., ydumb)  # Third step
    for i in range(n):
        ydumb[i] = y[i] + k3[i]  # Update for the fourth step
    k4 = h * f(t + h, ydumb)  # Fourth step

    # Combine the steps to update the state vector 'y'
    for i in range(n):
        y[i] = y[i] + (k1[i] + 2 * (k2[i] + k3[i]) + k4[i]) / 6.

    return y  # Return the updated state

# Main simulation loop
while t < tf:  # Loop until the final time is reached
    rate(500)  # Control the simulation speed (500 iterations per second)
    y = rk4(t, h)  # Update the state using the RK4 algorithm
    t += h  # Increment the time by step size 'h'

    # Update the positions of the bodies in the 3D scene
    body1.pos = vec(y[0], y[1], y[2])
    body2.pos = vec(y[3], y[4], y[5])
    body3.pos = vec(y[6], y[7], y[8])

    # Append the new positions to the paths to visualize the trajectory
    path1.append(pos=vec(y[0], y[1], y[2]))
    path2.append(pos=vec(y[3], y[4], y[5]))
    path3.append(pos=vec(y[6], y[7], y[8]))
