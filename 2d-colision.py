import pygame  # Biblioteca para criação de gráficos e jogos.
import numpy as np  # Biblioteca para cálculos matemáticos e manipulação de vetores.

# Configurações principais
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600  # Dimensões da janela da simulação em pixels.
FPS = 60  # Taxa de quadros por segundo (para animações suaves e consistentes).
GRAVITY = 9.81  # Aceleração gravitacional em m/s².

# Definição de cores em formato RGB.
WHITE = (255, 255, 255)  # Cor branca (fundo).
BLACK = (0, 0, 0)  # Cor preta (usada para textos e centro de massa).
RED = (255, 0, 0)  # Cor vermelha (para partículas).
BLUE = (0, 0, 255)  # Cor azul (para partículas).
GREEN = (0, 255, 0)  # Cor verde (para partículas).

# Classe que representa uma partícula física.
class Particle:
    def __init__(self, pos, vel, mass, radius, color):
        """
        Inicializa a partícula com posição, velocidade, massa, raio e cor.
        :param pos: Posição inicial da partícula (x, y).
        :param vel: Vetor de velocidade inicial da partícula (vx, vy).
        :param mass: Massa da partícula.
        :param radius: Raio da partícula.
        :param color: Cor da partícula.
        """
        self.pos = np.array(pos, dtype=float)  # Posição inicial como vetor numpy (x, y).
        self.vel = np.array(vel, dtype=float)  # Velocidade inicial como vetor numpy (vx, vy).
        self.mass = mass  # Massa da partícula.
        self.radius = radius  # Raio da partícula.
        self.color = color  # Cor da partícula.

    def kinetic_energy(self):
        """
        Calcula a energia cinética da partícula.
        :return: Energia cinética (0.5 * massa * velocidade²).
        """
        velocity_magnitude = np.linalg.norm(self.vel)  # Calcula a magnitude da velocidade (||v||).
        return 0.5 * self.mass * velocity_magnitude**2  # Fórmula da energia cinética.

    def potential_energy(self, ground_level=0):
        """
        Calcula a energia potencial gravitacional da partícula.
        :param ground_level: Nível de referência para o cálculo da altura.
        :return: Energia potencial (massa * gravidade * altura).
        """
        height = max(0, self.pos[1] - self.radius - ground_level)  # Altura acima do "chão".
        return self.mass * GRAVITY * height  # Fórmula da energia potencial.

    def mechanical_energy(self, ground_level=0):
        """
        Calcula a energia mecânica total da partícula.
        :param ground_level: Nível de referência para o cálculo.
        :return: Soma da energia cinética e potencial.
        """
        return self.kinetic_energy() + self.potential_energy(ground_level)

    def update(self, dt, walls):
        """
        Atualiza a posição e a velocidade da partícula considerando a gravidade.
        :param dt: Intervalo de tempo entre atualizações (em segundos).
        :param walls: Limites das paredes da simulação.
        """
        self.vel[1] -= GRAVITY * dt  # Aplica a gravidade na velocidade vertical.
        self.pos += self.vel * dt  # Atualiza a posição com base na velocidade.
        self.handle_wall_collisions(walls)  # Verifica e trata colisões com as paredes.

    def handle_wall_collisions(self, walls):
        """
        Verifica e resolve colisões da partícula com as paredes da simulação.
        :param walls: Dicionário com as coordenadas das paredes.
        """
        restitution = 0.8  # Coeficiente de restituição (elasticidade da colisão).
        if self.pos[0] - self.radius < walls["left"]:  # Colisão com a parede esquerda.
            self.pos[0] = walls["left"] + self.radius  # Reposiciona a partícula.
            self.vel[0] = -self.vel[0] * restitution  # Inverte e reduz a velocidade horizontal.
        if self.pos[0] + self.radius > walls["right"]:  # Colisão com a parede direita.
            self.pos[0] = walls["right"] - self.radius
            self.vel[0] = -self.vel[0] * restitution
        if self.pos[1] - self.radius < walls["bottom"]:  # Colisão com o chão.
            self.pos[1] = walls["bottom"] + self.radius
            self.vel[1] = -self.vel[1] * restitution


# Função para desenhar uma seta representando vetores (como velocidade).
def draw_arrow(screen, start, end, color, arrow_size=10):
    """
    Desenha uma seta no display entre dois pontos.
    :param screen: Superfície do pygame onde a seta será desenhada.
    :param start: Ponto inicial da seta.
    :param end: Ponto final da seta.
    :param color: Cor da seta.
    :param arrow_size: Tamanho do triângulo da ponta da seta.
    """
    pygame.draw.line(screen, color, start, end, 2)  # Desenha a linha principal da seta.
    direction = np.array(end) - np.array(start)  # Direção do vetor.
    magnitude = np.linalg.norm(direction)  # Calcula a magnitude do vetor.
    if magnitude == 0:
        return  # Evita desenhar se a direção for nula.
    direction /= magnitude  # Normaliza a direção.
    perpendicular = np.array([-direction[1], direction[0]])  # Vetor perpendicular à direção.
    base = np.array(end) - direction * arrow_size  # Base do triângulo da ponta.
    left = base + perpendicular * (arrow_size / 2)  # Ponta esquerda.
    right = base - perpendicular * (arrow_size / 2)  # Ponta direita.
    pygame.draw.polygon(screen, color, [end, left, right])  # Desenha o triângulo.

# Função para desenhar partículas na tela.
def draw_particles(screen, particles, show_vectors):
    """
    Desenha todas as partículas na tela.
    :param screen: Superfície do pygame.
    :param particles: Lista de partículas a serem desenhadas.
    :param show_vectors: Booleano para exibir ou não os vetores de velocidade.
    """
    for p in particles:
        # Desenha a partícula como um círculo preenchido.
        pygame.draw.circle(screen, p.color, (int(p.pos[0]), int(SCREEN_HEIGHT - p.pos[1])), int(p.radius))
        if show_vectors:  # Se habilitado, desenha o vetor de velocidade.
            velocity_end = p.pos + p.vel * 10  # Ponto final do vetor de velocidade (amplificado).
            draw_arrow(screen, (p.pos[0], SCREEN_HEIGHT - p.pos[1]),
                       (velocity_end[0], SCREEN_HEIGHT - velocity_end[1]), p.color)

# Função para desenhar informações de HUD (display de informações).
def draw_hud(screen, particles, font):
    """
    Exibe informações de energia no canto da tela.
    :param screen: Superfície do pygame.
    :param particles: Lista de partículas para exibição das energias.
    :param font: Fonte do texto.
    """
    x, y = 10, 10  # Posição inicial do texto.
    line_height = 25  # Distância vertical entre linhas de texto.
    for p in particles:
        ke = p.kinetic_energy()  # Energia cinética.
        pe = p.potential_energy()  # Energia potencial.
        me = p.mechanical_energy()  # Energia mecânica total.
        text = f"KE: {ke:.1f} | PE: {pe:.1f} | ME: {me:.1f}"  # Texto formatado.
        label = font.render(text, True, p.color)  # Renderiza o texto com a cor da partícula.
        screen.blit(label, (x, y))  # Desenha o texto na tela.
        y += line_height  # Move a posição para a próxima linha.


def handle_particle_collisions(particles):
    """
    Detecta e resolve colisões entre todas as partículas na simulação.
    :param particles: Lista de partículas.
    """
    for i in range(len(particles)):  # Itera sobre cada partícula.
        for j in range(i + 1, len(particles)):  # Evita repetições, comparando apenas pares únicos.
            p1, p2 = particles[i], particles[j]  # Seleciona duas partículas para verificar colisão.
            delta = p1.pos - p2.pos  # Vetor de deslocamento entre as partículas.
            dist = np.linalg.norm(delta)  # Distância entre os centros das partículas.
            overlap = p1.radius + p2.radius - dist  # Quantidade de interpenetração (se houver).
            if overlap > 0:  # Se as partículas estão colidindo:
                normal = delta / dist  # Vetor normalizado apontando de p2 para p1.
                correction = normal * overlap / 2  # Corrige a sobreposição igualmente entre as partículas.
                p1.pos += correction  # Move p1 para fora da interpenetração.
                p2.pos -= correction  # Move p2 para fora da interpenetração.
                resolve_collision(p1, p2, normal)  # Resolve a colisão ajustando as velocidades.

def resolve_collision(p1, p2, normal):
    """
    Resolve a colisão entre duas partículas ajustando suas velocidades com base no princípio da conservação do momento.
    :param p1: Primeira partícula.
    :param p2: Segunda partícula.
    :param normal: Vetor unitário que aponta da segunda para a primeira partícula.
    """
    rel_vel = p1.vel - p2.vel  # Calcula a velocidade relativa entre as partículas.
    vel_along_normal = np.dot(rel_vel, normal)  # Projeta a velocidade relativa ao longo do vetor normal.
    if vel_along_normal > 0:  # Se as partículas estão se afastando, nenhuma ação é necessária.
        return
    restitution = 0.9  # Coeficiente de restituição (determina o quão elástica é a colisão).
    # Calcula a magnitude do impulso escalar com base no coeficiente de restituição.
    impulse = (-(1 + restitution) * vel_along_normal) / (1 / p1.mass + 1 / p2.mass)
    impulse_vec = impulse * normal  # Vetor de impulso na direção do vetor normal.
    # Ajusta as velocidades das partículas aplicando o impulso proporcional às suas massas.
    p1.vel += impulse_vec / p1.mass
    p2.vel -= impulse_vec / p2.mass

def calculate_center_of_mass(particles):
    """
    Calcula o centro de massa de um sistema de partículas.
    :param particles: Lista de partículas.
    :return: Vetor (x, y) representando o centro de massa.
    """
    total_mass = sum(p.mass for p in particles)  # Soma a massa de todas as partículas.
    if total_mass == 0:  # Evita divisão por zero.
        return np.array([0, 0])  # Retorna (0, 0) se não houver partículas.
    # Calcula a posição ponderada pela massa de cada partícula.
    weighted_positions = sum(p.mass * p.pos for p in particles)
    return weighted_positions / total_mass  # Divide pelo total de massa para obter o centro de massa.

def draw_center_of_mass(screen, center_of_mass):
    """
    Desenha o centro de massa no display.
    :param screen: Superfície do pygame onde será desenhado.
    :param center_of_mass: Vetor (x, y) representando o centro de massa.
    """
    # Desenha um pequeno círculo preto no ponto do centro de massa.
    pygame.draw.circle(screen, BLACK, (int(center_of_mass[0]), int(SCREEN_HEIGHT - center_of_mass[1])), 5)
    # Adiciona um rótulo "CM" ao lado do centro de massa.
    label = pygame.font.Font(None, 24).render("CM", True, BLACK)
    screen.blit(label, (int(center_of_mass[0]) + 10, int(SCREEN_HEIGHT - center_of_mass[1]) - 10))

def main():
    """
    Função principal para inicializar a simulação, executar o loop principal e exibir partículas.
    """
    pygame.init()  # Inicializa todos os módulos do pygame.
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))  # Cria a janela principal.
    pygame.display.set_caption("Particle Simulation")  # Define o título da janela.
    clock = pygame.time.Clock()  # Cria um relógio para controlar o FPS.
    font = pygame.font.Font(None, 24)  # Define a fonte para o texto.

    walls = {"left": 0, "right": SCREEN_WIDTH, "bottom": 0}  # Define os limites da simulação.

    # Cria uma lista de partículas com posições, velocidades, massas, raios e cores iniciais.
    particles = [
        Particle(pos=[200, 400], vel=[60, -40], mass=1, radius=10, color=RED),
        Particle(pos=[400, 200], vel=[-50, 30], mass=2, radius=12, color=BLUE),
        Particle(pos=[600, 500], vel=[40, -30], mass=1.5, radius=15, color=GREEN),
    ]

    show_vectors = False  # Controle para exibir ou ocultar vetores de velocidade.
    running = True  # Flag para manter o loop principal em execução.

    while running:
        dt = clock.tick(FPS) / 1000  # Calcula o intervalo de tempo entre frames em segundos.

        for event in pygame.event.get():  # Processa os eventos do pygame.
            if event.type == pygame.QUIT:  # Evento para sair do programa.
                running = False
            if event.type == pygame.KEYDOWN:  # Evento de pressionar teclas.
                if event.key == pygame.K_v:  # Alterna a exibição de vetores.
                    show_vectors = not show_vectors

        for p in particles:  # Atualiza cada partícula.
            p.update(dt, walls)

        handle_particle_collisions(particles)  # Verifica e resolve colisões entre partículas.

        center_of_mass = calculate_center_of_mass(particles)  # Calcula o centro de massa.

        screen.fill(WHITE)  # Limpa a tela com a cor branca.
        draw_particles(screen, particles, show_vectors)  # Desenha as partículas na tela.
        draw_center_of_mass(screen, center_of_mass)  # Desenha o centro de massa.
        draw_hud(screen, particles, font)  # Exibe o HUD com informações de energia.
        pygame.display.flip()  # Atualiza a tela com os novos desenhos.

    pygame.quit()  # Finaliza o pygame quando o loop principal termina.

if __name__ == "__main__":
    main()  # Chama a função principal para iniciar a simulação.
