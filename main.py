"""
Bouncing Ball in a Rotating Octagon Simulation

This script simulates a ball bouncing inside a rotating octagon.
Gravity is included, and the octagon rotates slowly while the ball
bounces elastically off its moving boundaries.

To run:
    python main.py

Dependencies:
    numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Simulation parameters
dt = 0.01             # Time step [s]
g = -9.8              # Gravitational acceleration [units/s^2]
ball_radius = 0.3     # Radius of the ball
octagon_radius = 10.0 # Radius of circumscribed circle for the octagon
num_sides = 8         # Octagon has 8 sides
omega = 0.2          # Angular speed of octagon rotation [rad/s]

# Initial conditions
t_sim = 0.0           # Simulation time
theta = 0.0           # Initial rotation angle of octagon
ball_pos = np.array([0.0, 0.0])   # Initial position of the ball
ball_vel = np.array([8.0, 12.0])  # Initial velocity of the ball

def get_octagon_vertices(rotation_angle):
    """
    Returns the vertices of a regular octagon (counterclockwise order)
    rotated by rotation_angle (in radians).
    """
    angles = np.linspace(0, 2 * np.pi, num_sides, endpoint=False) + rotation_angle
    vertices = np.stack([octagon_radius * np.cos(angles),
                         octagon_radius * np.sin(angles)], axis=1)
    return vertices

def handle_collision(ball_pos, ball_vel, theta):
    """
    Checks for and handles collisions between the ball and the octagon's edges or vertices.
    Uses the moving-wall collision law: reflect the ball’s velocity relative to the wall’s
    (rotational) velocity.

    Parameters:
        ball_pos : numpy.array
            Current ball center position.
        ball_vel : numpy.array
            Current ball velocity.
        theta : float
            Current rotation angle of the octagon.

    Returns:
        (ball_pos, ball_vel) : tuple of numpy.array
            Updated ball position and velocity after collision handling.
    """
    vertices = get_octagon_vertices(theta)
    num_vertices = len(vertices)
    
    # Handle collisions with edges first
    for i in range(num_vertices):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % num_vertices]
        edge = p2 - p1

        # Compute inward normal (for CCW-ordered vertices, interior is to the left)
        n = np.array([-edge[1], edge[0]])
        n = n / np.linalg.norm(n)
        
        # Project ball center onto the line through p1 with normal n.
        d = np.dot(ball_pos - p1, n)
        
        # Find the closest point on the edge to the ball center
        edge_length_sq = np.dot(edge, edge)
        t = np.dot(ball_pos - p1, edge) / edge_length_sq
        
        # Only consider collisions with the edge segment (not extended line)
        if t < 0 or t > 1:
            continue
        
        if d < ball_radius:
            # Compute collision point on the edge
            collision_point = p1 + t * edge
            # Compute wall (edge) velocity at the collision point due to rotation
            v_wall = omega * np.array([-collision_point[1], collision_point[0]])
            # Relative velocity (ball relative to moving wall)
            v_rel = ball_vel - v_wall
            
            # Check if ball is moving toward the wall (relative velocity along n is negative)
            if np.dot(v_rel, n) < 0:
                # Reflect the relative velocity about the normal
                v_rel = v_rel - 2 * np.dot(v_rel, n) * n
                ball_vel = v_rel + v_wall
                # Reposition ball so its surface touches the edge exactly
                ball_pos = collision_point + n * ball_radius

    # Handle potential collisions with vertices
    for vertex in vertices:
        vec = ball_pos - vertex
        d = np.linalg.norm(vec)
        if d < ball_radius:
            n = vec / d if d != 0 else np.array([1, 0])
            # Wall velocity at the vertex (rotation)
            v_wall = omega * np.array([-vertex[1], vertex[0]])
            v_rel = ball_vel - v_wall
            if np.dot(v_rel, n) < 0:
                v_rel = v_rel - 2 * np.dot(v_rel, n) * n
                ball_vel = v_rel + v_wall
                ball_pos = vertex + n * ball_radius

    return ball_pos, ball_vel

def update(frame):
    """
    Update function for the animation. Integrates physics, handles collisions,
    updates the rotation of the octagon, and redraws the scene.
    """
    global ball_pos, ball_vel, theta, t_sim

    t_sim += dt
    theta += omega * dt  # Update octagon rotation angle

    # Update ball physics (gravity and motion)
    ball_vel[1] += g * dt
    new_pos = ball_pos + ball_vel * dt

    # Handle collision with the rotated octagon boundaries
    ball_pos, ball_vel = handle_collision(new_pos, ball_vel, theta)

    # Clear and update the plot
    ax.clear()
    
    # Draw the octagon (closing the polygon by repeating the first vertex)
    vertices = get_octagon_vertices(theta)
    octagon = np.vstack([vertices, vertices[0]])
    ax.plot(octagon[:, 0], octagon[:, 1], 'k-', lw=2)

    # Draw the ball as a circle
    circle = plt.Circle(ball_pos, ball_radius, color='blue')
    ax.add_patch(circle)

    # Set plot limits and aesthetics
    ax.set_xlim(-octagon_radius - 1, octagon_radius + 1)
    ax.set_ylim(-octagon_radius - 1, octagon_radius + 1)
    ax.set_aspect('equal')
    ax.set_title("Bouncing Ball in a Rotating Octagon")
    
    return ax.patches

# Set up the figure and axes for the animation
fig, ax = plt.subplots(figsize=(6, 6))
ani = FuncAnimation(fig, update, frames=600, interval=dt * 1000, blit=False)

if __name__ == "__main__":
    plt.show()