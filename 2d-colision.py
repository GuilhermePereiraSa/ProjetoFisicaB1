import numpy as np

class Particle:
    def __init__(self, pos, vel, mass, radius):
        self.pos = np.array(pos, dtype=float)  # Position [x, y]
        self.vel = np.array(vel, dtype=float)  # Velocity [vx, vy]
        self.mass = mass                      # Mass
        self.radius = radius                  # Radius for collision

def update(dt, particles):
    for p in particles:
        # Update positions based on velocity and acceleration (if any)
        p.pos += p.vel * dt

    # Handle collisions between particles
    handle_particle_collisions(particles)

def handle_particle_collisions(particles):
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            p1 = particles[i]
            p2 = particles[j]

            # Check for collision
            dist = np.linalg.norm(p1.pos - p2.pos)
            if dist <= p1.radius + p2.radius:  # Collision detected
                resolve_collision(p1, p2)

def resolve_collision(p1, p2):
    # Relative velocity
    rel_vel = p1.vel - p2.vel
    # Normal vector
    normal = (p1.pos - p2.pos) / np.linalg.norm(p1.pos - p2.pos)
    # Velocity along the normal
    vel_along_normal = np.dot(rel_vel, normal)

    # If velocities are separating, no collision
    if vel_along_normal > 0:
        return

    # Conservation of momentum (elastic collision)
    restitution = 1  # For perfectly elastic collision
    impulse = (-(1 + restitution) * vel_along_normal) / (1 / p1.mass + 1 / p2.mass)

    # Impulse vector
    impulse_vec = impulse * normal

    # Update velocities
    p1.vel += impulse_vec / p1.mass
    p2.vel -= impulse_vec / p2.mass



# Example with two particles
p1 = Particle(pos=[0, 0], vel=[1, 0], mass=1, radius=1)
p2 = Particle(pos=[2, 0], vel=[-1, 0], mass=1, radius=1)

particles = [p1, p2]

dt = 1 / 60  # Assuming 60 FPS
for _ in range(10):  # Simulate 10 frames
    update(dt, particles)
    print(f"Particle 1: pos={p1.pos}, vel={p1.vel}")
    print(f"Particle 2: pos={p2.pos}, vel={p2.vel}")
