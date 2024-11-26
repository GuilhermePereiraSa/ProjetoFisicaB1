# 🎱 Simulação de Colisões com Partículas e Bola de Sinuca
## 📖 Introdução
Este projeto é uma simulação interativa que combina conceitos de física computacional com a programação de um pequeno jogo. Ele simula as colisões entre partículas (como bolas de sinuca), criando uma interface visual para demonstrar fenômenos físicos como conservação de momento e colisões elásticas.

### Objetivo
- Simular colisões entre partículas e objetos em um ambiente controlado.
- Demonstrar conceitos de física como conservação de energia e momentum.
- Criar uma interface visual interativa utilizando a biblioteca Pygame.
O projeto é inspirado no capítulo 2 do livro Computational Physics e em vídeos educativos como Building Collision Simulations do canal Reducible. A ideia surgiu do desejo de criar uma aplicação que una teoria física e programação em uma experiência prática.

### Como usar
Instalar as dependências necessárias:
```bash
pip install pygame
```
```bash
pip install numpy
```
Ou use o alternativo, se necessário:

```bash
sudo apt install python3-pygame
```

```bash
sudo apt install python3-numpy
```

### Simulação e Interação
A simulação é controlada por uma interface gráfica, onde o usuário pode ver e interagir com as partículas (bolas de sinuca). O comportamento das colisões, as energias cinéticas e potenciais, além do movimento das bolas, são visualizados em tempo real.

## 🛠 Desenvolvimento
### 📦 Estrutura do Projeto
O projeto está dividido em duas partes principais:

### Simulação de Física:

#### Partículas: Implementação de partículas com propriedades como posição, velocidade, massa e raio.
Gerenciamento de colisões: Utilização de vetores e leis de conservação para detectar e processar colisões.
Atualização do estado das partículas: Movimentação das partículas ao longo do tempo, com simulação de força gravitacional e colisões elásticas.
Interface de Sinuca:

#### Mesa de Sinuca: Representação gráfica de uma mesa de sinuca usando Pygame.
- Bolas de Sinuca: Criação de bolas (partículas) que se movem na mesa e colidem entre si.
- Gerenciamento de colisões com bolsões e bordas: Detecção de colisões das bolas com os limites da mesa e os bolsões.
##### Fluxo do Código
O código simula as partículas em um ambiente 2D (mesa de sinuca) onde as bolas colidem entre si e com as bordas. A física das colisões é baseada em conceitos reais de conservação de energia e momentum, com a interação visual sendo feita com Pygame.

###### Partículas são criadas com atributos como massa, posição, velocidade e raio.
- Colisões são detectadas e resolvidas usando a física de colisões elásticas (conservação de energia cinética e momentum).
- Movimento das partículas é calculado a cada quadro com a aplicação de aceleração devido à gravidade e interações de colisão.

```python
import pygame
import numpy as np
```

###### Parte principal do código que trata conceitos físicos


```python
class Particle:
def **init**(self, pos, vel, mass, radius, color):
self.pos = np.array(pos, dtype=float)
self.vel = np.array(vel, dtype=float)
self.mass = mass
self.radius = radius
self.color = color

    def kinetic_energy(self):
        # KE = 0.5 * m * v^2
        velocity_magnitude = np.linalg.norm(self.vel)
        return 0.5 * self.mass * velocity_magnitude**2

    def potential_energy(self, ground_level=0):
        # PE = m * g * h (h é a altura acima do chão)
        height = max(0, self.pos[1] - self.radius - ground_level)
        return self.mass * GRAVITY * height

    def mechanical_energy(self, ground_level=0):
        # ME = KE + PE
        return self.kinetic_energy() + self.potential_energy(ground_level)

```

```python
def calculate_center_of_mass(particles):
    total_mass = sum(p.mass for p in particles)
    if total_mass == 0:
        return np.array([0, 0])
    weighted_positions = sum(p.mass * p.pos for p in particles)
    return weighted_positions / total_mass
```
## Referências

- **Livro**: *Computational Physics* (Capítulo 2)
- **Canal**: *Reducible* – [Building Collision Simulations](https://www.youtube.com/watch?v=eED4bSkYCB8&ab_channel=Reducible)
