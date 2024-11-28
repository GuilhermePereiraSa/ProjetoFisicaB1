import pygame
import numpy as np

# Configurações principais
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
FPS = 60
GRAVITY = 9.81  # m/s^2 (aceleração da gravidade)

# Cores
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

class Particle:
    def __init__(self, pos, vel, mass, radius, color):
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.mass = mass
        self.radius = radius
        self.color = color

    def kinetic_energy(self):
        velocity_magnitude = np.linalg.norm(self.vel)
        return 0.5 * self.mass * velocity_magnitude**2

    def potential_energy(self, ground_level=0):
        height = max(0, self.pos[1] - self.radius - ground_level)
        return self.mass * GRAVITY * height

    def mechanical_energy(self, ground_level=0):
        return self.kinetic_energy() + self.potential_energy(ground_level)

    def update(self, dt, walls):
        self.vel[1] -= GRAVITY * dt
        self.pos += self.vel * dt
        self.handle_wall_collisions(walls)

    def handle_wall_collisions(self, walls):
        restitution = 0.8
        if self.pos[0] - self.radius < walls["left"]:
            self.pos[0] = walls["left"] + self.radius
            self.vel[0] = -self.vel[0] * restitution
        if self.pos[0] + self.radius > walls["right"]:
            self.pos[0] = walls["right"] - self.radius
            self.vel[0] = -self.vel[0] * restitution
        if self.pos[1] - self.radius < walls["bottom"]:
            self.pos[1] = walls["bottom"] + self.radius
            self.vel[1] = -self.vel[1] * restitution

def draw_arrow(screen, start, end, color, arrow_size=10):
    pygame.draw.line(screen, color, start, end, 2)
    direction = np.array(end) - np.array(start)
    magnitude = np.linalg.norm(direction)
    if magnitude == 0:
        return
    direction /= magnitude
    perpendicular = np.array([-direction[1], direction[0]])
    base = np.array(end) - direction * arrow_size
    left = base + perpendicular * (arrow_size / 2)
    right = base - perpendicular * (arrow_size / 2)
    pygame.draw.polygon(screen, color, [end, left, right])

def draw_particles(screen, particles, show_vectors):
    for p in particles:
        pygame.draw.circle(screen, p.color, (int(p.pos[0]), int(SCREEN_HEIGHT - p.pos[1])), int(p.radius))
        if show_vectors:
            velocity_end = p.pos + p.vel * 10
            draw_arrow(screen, (p.pos[0], SCREEN_HEIGHT - p.pos[1]),
                       (velocity_end[0], SCREEN_HEIGHT - velocity_end[1]), p.color)

def draw_hud(screen, particles, font):
    x, y = 10, 10
    line_height = 25
    for p in particles:
        ke = p.kinetic_energy()
        pe = p.potential_energy()
        me = p.mechanical_energy()
        text = f"KE: {ke:.1f} | PE: {pe:.1f} | ME: {me:.1f}"
        label = font.render(text, True, p.color)
        screen.blit(label, (x, y))
        y += line_height

def handle_particle_collisions(particles):
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            p1, p2 = particles[i], particles[j]
            delta = p1.pos - p2.pos
            dist = np.linalg.norm(delta)
            overlap = p1.radius + p2.radius - dist
            if overlap > 0:
                normal = delta / dist
                correction = normal * overlap / 2
                p1.pos += correction
                p2.pos -= correction
                resolve_collision(p1, p2, normal)

def resolve_collision(p1, p2, normal):
    rel_vel = p1.vel - p2.vel
    vel_along_normal = np.dot(rel_vel, normal)
    if vel_along_normal > 0:
        return
    restitution = 0.9
    impulse = (-(1 + restitution) * vel_along_normal) / (1 / p1.mass + 1 / p2.mass)
    impulse_vec = impulse * normal
    p1.vel += impulse_vec / p1.mass
    p2.vel -= impulse_vec / p2.mass

def calculate_center_of_mass(particles):
    total_mass = sum(p.mass for p in particles)
    if total_mass == 0:
        return np.array([0, 0])
    weighted_positions = sum(p.mass * p.pos for p in particles)
    return weighted_positions / total_mass

def draw_center_of_mass(screen, center_of_mass):
    pygame.draw.circle(screen, BLACK, (int(center_of_mass[0]), int(SCREEN_HEIGHT - center_of_mass[1])), 5)
    label = pygame.font.Font(None, 24).render("CM", True, BLACK)
    screen.blit(label, (int(center_of_mass[0]) + 10, int(SCREEN_HEIGHT - center_of_mass[1]) - 10))

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Particle Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    walls = {"left": 0, "right": SCREEN_WIDTH, "bottom": 0}

    particles = [
        Particle(pos=[200, 400], vel=[60, -40], mass=1, radius=10, color=RED),
        Particle(pos=[400, 200], vel=[-50, 30], mass=2, radius=12, color=BLUE),
        Particle(pos=[600, 500], vel=[40, -30], mass=1.5, radius=15, color=GREEN),
    ]

    show_vectors = False
    running = True

    while running:
        dt = clock.tick(FPS) / 1000

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_v:
                    show_vectors = not show_vectors

        for p in particles:
            p.update(dt, walls)

        handle_particle_collisions(particles)

        center_of_mass = calculate_center_of_mass(particles)

        screen.fill(WHITE)
        draw_particles(screen, particles, show_vectors)
        draw_center_of_mass(screen, center_of_mass)
        draw_hud(screen, particles, font)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
