
import math
import random
import sys
from collections import deque

import pygame

pygame.init()

# ==========================================
# CONFIGURAÇÕES DA SIMULAÇÃO
# ==========================================

W, H = 1220, 860
TELA = pygame.display.set_mode((W, H))
pygame.display.set_caption("Laboratório de Controle PID")

RELOGIO = pygame.time.Clock()

FONTE = pygame.font.SysFont("dejavusans", 16)
PEQUENA = pygame.font.SysFont("dejavusans", 13)
TITULO = pygame.font.SysFont("dejavusans", 24, bold=True)

FPS = 60
DT = 1 / 240
G = 9.81

FUNDO = (17, 24, 36)
PAINEL = (30, 41, 57)
GRADE = (67, 79, 98)
BRANCO = (235, 240, 247)
CINZA = (155, 169, 187)
AZUL = (75, 169, 244)
LARANJA = (255, 176, 65)
VERDE = (73, 215, 155)
VERMELHO = (240, 105, 105)

ruido_ativo = True
perturbacoes_ativas = True


def limitar(valor, minimo, maximo):
    return max(minimo, min(valor, maximo))


def rad(graus):
    return math.radians(graus)


def texto(msg, x, y, cor=BRANCO, fonte=FONTE):
    img = fonte.render(str(msg), True, cor)
    TELA.blit(img, (int(x), int(y)))


def angulo_normalizado(theta):
    return math.atan2(math.sin(theta), math.cos(theta))


# ==========================================
# SISTEMA BALL AND BEAM
# ==========================================

class BallBeam:
    def __init__(self):
        self.comprimento = 0.80
        self.limite = self.comprimento / 2 - 0.025

        # Ganhos PID
        self.kp = 1.4
        self.ki = 0.06
        self.kd = 0.28

        # Servo
        self.angulo_max = rad(10)
        self.vel_servo_max = rad(85)
        self.tau_servo = 0.18
        self.zona_morta = rad(0.5)

        # Atrito
        self.atrito = 0.16

        # Sensor: 25 Hz
        self.periodo_sensor = 0.04
        self.ruido_sensor = 0.0025
        self.resolucao_sensor = 0.005

        self.pid_ativo = True
        self.reset()

    def reset(self):
        self.x = 0.24
        self.v = 0.0

        self.alpha = 0.0
        self.alpha_dot = 0.0

        self.x_medida = self.x
        self.v_medida = 0.0

        self.integral = 0.0
        self.comando = 0.0

        self.timer_sensor = 0.0
        self.desde_medida = 0.0
        self.tempo = 0.0

    def atualizar(self, dt):
        self.tempo += dt
        self.timer_sensor += dt
        self.desde_medida += dt

        # ----------------------------------
        # SENSOR E CONTROLADOR DISCRETOS
        # ----------------------------------

        if self.timer_sensor >= self.periodo_sensor:
            self.timer_sensor -= self.periodo_sensor

            leitura_valida = True

            if ruido_ativo:
                # 3% de perda de medições
                leitura_valida = random.random() > 0.03

            if leitura_valida:
                medida = self.x

                if ruido_ativo:
                    medida += random.gauss(
                        0, self.ruido_sensor
                    )

                # Quantização do sensor
                medida = (
                    round(medida / self.resolucao_sensor)
                    * self.resolucao_sensor
                )

                medida = limitar(
                    medida, -self.limite, self.limite
                )

                h = max(
                    self.desde_medida,
                    self.periodo_sensor
                )

                velocidade_instantanea = (
                    medida - self.x_medida
                ) / h

                # Filtro da velocidade estimada
                self.v_medida = (
                    0.25 * velocidade_instantanea
                    + 0.75 * self.v_medida
                )

                self.x_medida = medida
                self.desde_medida = 0.0

            # ----------------------------------
            # CONTROLADOR PID
            # ----------------------------------

            if self.pid_ativo:
                erro = self.x_medida

                integral_teste = limitar(
                    self.integral
                    + erro * self.periodo_sensor,
                    -0.20, 0.20
                )

                u_teste = (
                    -self.kp * erro
                    -self.ki * integral_teste
                    -self.kd * self.v_medida
                )

                # Anti-windup
                if abs(u_teste) < self.angulo_max:
                    self.integral = integral_teste

                self.comando = (
                    -self.kp * erro
                    -self.ki * self.integral
                    -self.kd * self.v_medida
                )

                self.comando = limitar(
                    self.comando,
                    -self.angulo_max,
                    self.angulo_max
                )

            else:
                self.comando = 0.0
                self.integral = 0.0

        if not self.pid_ativo:
            self.comando = 0.0

        # ----------------------------------
        # DINÂMICA DO SERVOMOTOR
        # ----------------------------------

        diferenca = self.comando - self.alpha

        if abs(diferenca) < self.zona_morta:
            self.alpha_dot = 0.0
        else:
            self.alpha_dot = limitar(
                diferenca / self.tau_servo,
                -self.vel_servo_max,
                self.vel_servo_max
            )

        self.alpha += self.alpha_dot * dt

        self.alpha = limitar(
            self.alpha,
            -self.angulo_max,
            self.angulo_max
        )

        # ----------------------------------
        # DINÂMICA DA BOLINHA
        # ----------------------------------

        aceleracao = (5.0 / 7.0) * (
            G * math.sin(self.alpha)
            + self.x * self.alpha_dot**2
        )

        aceleracao -= self.atrito * self.v

        if perturbacoes_ativas:
            aceleracao += (
                0.06 * math.sin(2.2 * self.tempo)
            )

        # Integração semi-implícita
        self.v += aceleracao * dt
        self.x += self.v * dt

        # Colisões nas extremidades
        if self.x >= self.limite:
            self.x = self.limite
            self.v = -abs(self.v) * 0.15

        elif self.x <= -self.limite:
            self.x = -self.limite
            self.v = abs(self.v) * 0.15


# ==========================================
# PÊNDULO INVERTIDO COM MOTOR NO EIXO
# ==========================================

class Pendulo:
    def __init__(self):
        self.m = 0.45
        self.L = 0.70
        self.I = self.m * self.L**2

        # Atritos
        self.atrito = 0.07
        self.atrito_seco = 0.035

        # Motor
        self.torque_max = 2.5
        self.tau_motor = 0.10
        self.taxa_torque_max = 15.0
        self.zona_morta = 0.045

        # Ganhos PID
        self.kp = 6.5
        self.ki = 0.08
        self.kd = 0.45

        # Sensor: 50 Hz
        self.periodo_sensor = 0.02
        self.ruido_angulo = rad(0.35)
        self.ruido_velocidade = rad(1.2)

        self.pid_ativo = True
        self.reset()

    def reset(self):
        self.theta = rad(35)
        self.omega = 0.0

        self.theta_medida = self.theta
        self.omega_medida = 0.0

        self.integral = 0.0
        self.comando = 0.0
        self.torque = 0.0

        self.timer_sensor = 0.0
        self.tempo = 0.0

    def atualizar(self, dt):
        self.tempo += dt
        self.timer_sensor += dt

        # ----------------------------------
        # SENSOR E PID
        # ----------------------------------

        if self.timer_sensor >= self.periodo_sensor:
            self.timer_sensor -= self.periodo_sensor

            self.theta_medida = self.theta
            self.omega_medida = self.omega

            if ruido_ativo:
                self.theta_medida += random.gauss(
                    0, self.ruido_angulo
                )

                self.omega_medida += random.gauss(
                    0, self.ruido_velocidade
                )

            erro = angulo_normalizado(
                self.theta_medida
            )

            if self.pid_ativo:
                integral_teste = limitar(
                    self.integral
                    + erro * self.periodo_sensor,
                    -1.0, 1.0
                )

                u_teste = (
                    -self.kp * erro
                    -self.ki * integral_teste
                    -self.kd * self.omega_medida
                )

                # Anti-windup
                if abs(u_teste) < self.torque_max:
                    self.integral = integral_teste

                self.comando = (
                    -self.kp * erro
                    -self.ki * self.integral
                    -self.kd * self.omega_medida
                )

                self.comando = limitar(
                    self.comando,
                    -self.torque_max,
                    self.torque_max
                )

                # Zona morta do motor
                if abs(self.comando) < self.zona_morta:
                    self.comando = 0.0

            else:
                self.comando = 0.0
                self.integral = 0.0

        if not self.pid_ativo:
            self.comando = 0.0

        # ----------------------------------
        # DINÂMICA DO MOTOR
        # ----------------------------------

        taxa = (
            self.comando - self.torque
        ) / self.tau_motor

        taxa = limitar(
            taxa,
            -self.taxa_torque_max,
            self.taxa_torque_max
        )

        self.torque += taxa * dt

        # ----------------------------------
        # PERTURBAÇÕES EXTERNAS
        # ----------------------------------

        perturbacao = 0.0

        if perturbacoes_ativas:
            perturbacao = (
                0.055 * math.sin(2.4 * self.tempo)
                + 0.025 * math.sin(5.3 * self.tempo)
            )

        # Atrito seco aproximado
        atrito_seco = (
            self.atrito_seco
            * math.tanh(self.omega / 0.025)
        )

        # ----------------------------------
        # DINÂMICA FÍSICA
        # ----------------------------------

        aceleracao = (
            self.m * G * self.L * math.sin(self.theta)
            - self.atrito * self.omega
            - atrito_seco
            + self.torque
            + perturbacao
        ) / self.I

        self.omega += aceleracao * dt
        self.theta += self.omega * dt


bola = BallBeam()
pendulo = Pendulo()


# ==========================================
# SLIDERS INTERATIVOS
# ==========================================

class Slider:
    def __init__(
        self, objeto, atributo,
        rotulo, minimo, maximo,
        x, y, cor
    ):
        self.objeto = objeto
        self.atributo = atributo
        self.rotulo = rotulo

        self.minimo = minimo
        self.maximo = maximo

        self.x = x
        self.y = y
        self.largura = 310
        self.cor = cor

    def valor(self):
        return getattr(self.objeto, self.atributo)

    def alterar(self, mouse_x):
        proporcao = limitar(
            (mouse_x - self.x) / self.largura,
            0.0, 1.0
        )

        valor = (
            self.minimo
            + proporcao * (self.maximo - self.minimo)
        )

        setattr(self.objeto, self.atributo, valor)

    def clicou(self, pos):
        mx, my = pos

        return (
            self.x - 12 <= mx <= self.x + self.largura + 12
            and abs(my - self.y) <= 15
        )

    def desenhar(self):
        valor = self.valor()

        proporcao = limitar(
            (valor - self.minimo)
            / (self.maximo - self.minimo),
            0, 1
        )

        px = self.x + proporcao * self.largura

        texto(
            self.rotulo,
            self.x - 67,
            self.y - 11
        )

        pygame.draw.line(
            TELA, GRADE,
            (self.x, self.y),
            (self.x + self.largura, self.y),
            7
        )

        pygame.draw.line(
            TELA, self.cor,
            (self.x, self.y),
            (int(px), self.y),
            7
        )

        pygame.draw.circle(
            TELA, BRANCO,
            (int(px), self.y), 10
        )

        pygame.draw.circle(
            TELA, self.cor,
            (int(px), self.y), 6
        )

        texto(
            f"{valor:.2f}",
            self.x + self.largura + 15,
            self.y - 11,
            BRANCO
        )


sliders = []

for objeto, inicio, cor, faixas in [
    (
        bola, 135, AZUL,
        [
            ("kp", "Kp", 0, 4),
            ("ki", "Ki", 0, 0.8),
            ("kd", "Kd", 0, 2)
        ]
    ),
    (
        pendulo, 735, VERDE,
        [
            ("kp", "Kp", 0, 15),
            ("ki", "Ki", 0, 3),
            ("kd", "Kd", 0, 3)
        ]
    )
]:
    for i, (atributo, rotulo, minimo, maximo) \
            in enumerate(faixas):

        sliders.append(
            Slider(
                objeto, atributo, rotulo,
                minimo, maximo,
                inicio, 636 + i * 44, cor
            )
        )


# ==========================================
# GRÁFICOS
# ==========================================

MAX_AMOSTRAS = 360

historico_bola = deque(maxlen=MAX_AMOSTRAS)
historico_pend = deque(maxlen=MAX_AMOSTRAS)


def desenhar_grafico(
    historico, x, y,
    largura, altura,
    escala, cor_real
):
    pygame.draw.rect(
        TELA, (22, 31, 45),
        (x, y, largura, altura),
        border_radius=6
    )

    meio = y + altura / 2

    # Referência = zero
    pygame.draw.line(
        TELA, BRANCO,
        (x, int(meio)),
        (x + largura, int(meio)), 1
    )

    if len(historico) < 2:
        return

    pontos_reais = []
    pontos_medidos = []

    for i, (real, medido) in enumerate(historico):
        px = x + i * (
            largura - 1
        ) / (MAX_AMOSTRAS - 1)

        real_normalizado = limitar(
            real / escala, -1, 1
        )

        medido_normalizado = limitar(
            medido / escala, -1, 1
        )

        py_real = meio - (
            real_normalizado * altura * 0.44
        )

        py_medido = meio - (
            medido_normalizado * altura * 0.44
        )

        pontos_reais.append(
            (int(px), int(py_real))
        )

        pontos_medidos.append(
            (int(px), int(py_medido))
        )

    pygame.draw.lines(
        TELA, cor_real,
        False, pontos_reais, 2
    )

    pygame.draw.lines(
        TELA, LARANJA,
        False, pontos_medidos, 2
    )


def desenhar_legenda(x, y, cor_real):
    itens = [
        ("Real", cor_real),
        ("Sensor", LARANJA),
        ("Referência", BRANCO)
    ]

    for i, (nome, cor) in enumerate(itens):
        xx = x + i * 118

        pygame.draw.line(
            TELA, cor,
            (xx, y + 8), (xx + 19, y + 8),
            3
        )

        texto(nome, xx + 25, y, CINZA, PEQUENA)


# ==========================================
# POSIÇÕES DOS OBJETOS
# ==========================================

BX, BY = 310, 325
PX, PY = 910, 292

ESCALA_BOLA = 510
ESCALA_PEND = 195


def posicao_bola():
    a = bola.alpha
    d = bola.x * ESCALA_BOLA
    raio = 14

    return (
        BX + d * math.cos(a) + raio * math.sin(a),
        BY + d * math.sin(a) - raio * math.cos(a)
    )


def posicao_pendulo():
    return (
        PX + pendulo.L * ESCALA_PEND
        * math.sin(pendulo.theta),

        PY - pendulo.L * ESCALA_PEND
        * math.cos(pendulo.theta)
    )


# ==========================================
# DESENHO DO BALL AND BEAM
# ==========================================

def desenhar_bola():
    pygame.draw.rect(
        TELA, PAINEL,
        (20, 75, 580, 710),
        border_radius=12
    )

    texto(
        "BALL AND BEAM",
        43, 94, BRANCO, TITULO
    )

    estado = (
        "PID LIGADO"
        if bola.pid_ativo
        else "SEM CONTROLE"
    )

    texto(
        estado, 43, 130,
        VERDE if bola.pid_ativo else VERMELHO
    )

    # Centro desejado
    pygame.draw.line(
        TELA, GRADE,
        (BX, 180), (BX, 414), 1
    )

    # Apoio
    pygame.draw.polygon(
        TELA, (105, 119, 139),
        [
            (BX, BY + 5),
            (BX - 30, BY + 85),
            (BX + 30, BY + 85)
        ]
    )

    a = bola.alpha
    metade = (
        bola.comprimento / 2 * ESCALA_BOLA
    )

    dx = metade * math.cos(a)
    dy = metade * math.sin(a)

    inicio = (
        int(BX - dx), int(BY - dy)
    )

    fim = (
        int(BX + dx), int(BY + dy)
    )

    pygame.draw.line(
        TELA, AZUL, inicio, fim, 12
    )

    # Batentes
    for sinal in [-1, 1]:
        cx = BX + sinal * dx
        cy = BY + sinal * dy

        nx = math.sin(a) * 22
        ny = -math.cos(a) * 22

        pygame.draw.line(
            TELA, AZUL,
            (cx, cy),
            (cx + nx, cy + ny), 5
        )

    # Bolinha
    x, y = posicao_bola()

    pygame.draw.circle(
        TELA, LARANJA,
        (int(x), int(y)), 14
    )

    pygame.draw.circle(
        TELA, (255, 220, 150),
        (int(x - 4), int(y - 4)), 4
    )

    texto(
        f"Real: {bola.x * 100:+.1f} cm",
        43, 424
    )

    texto(
        f"Sensor: {bola.x_medida * 100:+.1f} cm",
        260, 424
    )

    texto(
        f"Inclinação: {math.degrees(bola.alpha):+.1f}°",
        43, 449, CINZA
    )

    texto(
        "Posição da bolinha (m)",
        43, 471, CINZA, PEQUENA
    )

    desenhar_grafico(
        historico_bola,
        43, 490, 535, 96,
        0.4, AZUL
    )

    desenhar_legenda(43, 591, AZUL)


# ==========================================
# DESENHO DO PÊNDULO
# ==========================================

def desenhar_pendulo():
    pygame.draw.rect(
        TELA, PAINEL,
        (620, 75, 580, 710),
        border_radius=12
    )

    texto(
        "PÊNDULO INVERTIDO",
        643, 94, BRANCO, TITULO
    )

    estado = (
        "PID LIGADO"
        if pendulo.pid_ativo
        else "SEM CONTROLE"
    )

    texto(
        estado, 643, 130,
        VERDE if pendulo.pid_ativo else VERMELHO
    )

    pygame.draw.line(
        TELA, GRADE,
        (PX, 150), (PX, 430), 1
    )

    x, y = posicao_pendulo()

    pygame.draw.line(
        TELA, VERDE,
        (PX, PY),
        (int(x), int(y)), 6
    )

    pygame.draw.circle(
        TELA, (190, 205, 220),
        (PX, PY), 9
    )

    pygame.draw.circle(
        TELA, LARANJA,
        (int(x), int(y)), 19
    )

    angulo_real = math.degrees(
        angulo_normalizado(pendulo.theta)
    )

    angulo_medido = math.degrees(
        angulo_normalizado(pendulo.theta_medida)
    )

    texto(
        f"Real: {angulo_real:+.1f}°",
        643, 424
    )

    texto(
        f"Sensor: {angulo_medido:+.1f}°",
        850, 424
    )

    texto(
        f"Torque aplicado: {pendulo.torque:+.2f} N.m",
        643, 449, CINZA
    )

    texto(
        "Ângulo do pêndulo (graus)",
        643, 471, CINZA, PEQUENA
    )

    desenhar_grafico(
        historico_pend,
        643, 490, 535, 96,
        180, VERDE
    )

    desenhar_legenda(643, 591, VERDE)


# ==========================================
# INTERAÇÃO COM O MOUSE
# ==========================================

arrastando = None
slider_ativo = None
pausado = False


def iniciar_arraste(pos):
    global arrastando

    mx, my = pos

    xb, yb = posicao_bola()
    xp, yp = posicao_pendulo()

    if math.hypot(mx - xb, my - yb) < 35:
        arrastando = "bola"

        bola.v = 0.0
        bola.v_medida = 0.0
        bola.integral = 0.0

    elif math.hypot(mx - xp, my - yp) < 42:
        arrastando = "pendulo"

        pendulo.omega = 0.0
        pendulo.omega_medida = 0.0
        pendulo.integral = 0.0


def mover_arraste(pos):
    mx, my = pos

    if arrastando == "bola":
        dx = mx - BX
        dy = my - BY

        deslocamento = (
            dx * math.cos(bola.alpha)
            + dy * math.sin(bola.alpha)
        )

        bola.x = limitar(
            deslocamento / ESCALA_BOLA,
            -bola.limite, bola.limite
        )

        bola.v = 0.0
        bola.x_medida = bola.x
        bola.v_medida = 0.0

    elif arrastando == "pendulo":
        dx = mx - PX
        dy = PY - my

        if math.hypot(dx, dy) > 15:
            pendulo.theta = math.atan2(dx, dy)
            pendulo.omega = 0.0

            pendulo.theta_medida = pendulo.theta
            pendulo.omega_medida = 0.0


# ==========================================
# LOOP PRINCIPAL
# ==========================================

acumulador = 0.0
contador_passos = 0
rodando = True

while rodando:
    tempo_frame = min(
        RELOGIO.tick(FPS) / 1000.0, 0.05
    )

    for evento in pygame.event.get():

        if evento.type == pygame.QUIT:
            rodando = False

        elif evento.type == pygame.KEYDOWN:

            if evento.key == pygame.K_ESCAPE:
                rodando = False

            elif evento.key == pygame.K_1:
                bola.pid_ativo = not bola.pid_ativo
                bola.integral = 0.0

            elif evento.key == pygame.K_2:
                pendulo.pid_ativo = not pendulo.pid_ativo
                pendulo.integral = 0.0

            elif evento.key == pygame.K_SPACE:
                pausado = not pausado

            elif evento.key == pygame.K_n:
                ruido_ativo = not ruido_ativo

            elif evento.key == pygame.K_d:
                perturbacoes_ativas = (
                    not perturbacoes_ativas
                )

            elif evento.key == pygame.K_r:
                bola.reset()
                pendulo.reset()

                historico_bola.clear()
                historico_pend.clear()

                arrastando = None
                slider_ativo = None
                acumulador = 0.0

        elif evento.type == pygame.MOUSEBUTTONDOWN:
            if evento.button == 1:

                slider_ativo = None

                for slider in sliders:
                    if slider.clicou(evento.pos):
                        slider_ativo = slider
                        slider.alterar(evento.pos[0])
                        break

                if slider_ativo is None:
                    iniciar_arraste(evento.pos)

        elif evento.type == pygame.MOUSEMOTION:

            if slider_ativo is not None:
                slider_ativo.alterar(evento.pos[0])

            elif arrastando is not None:
                mover_arraste(evento.pos)

        elif evento.type == pygame.MOUSEBUTTONUP:
            if evento.button == 1:

                if arrastando == "bola":
                    bola.x_medida = bola.x
                    bola.v_medida = 0.0
                    bola.v = 0.0
                    bola.desde_medida = 0.0

                elif arrastando == "pendulo":
                    pendulo.theta_medida = pendulo.theta
                    pendulo.omega_medida = 0.0
                    pendulo.omega = 0.0

                arrastando = None
                slider_ativo = None

    # --------------------------------------
    # INTEGRAÇÃO FÍSICA
    # --------------------------------------

    if not pausado:
        acumulador += tempo_frame

        while acumulador >= DT:

            if arrastando != "bola":
                bola.atualizar(DT)

            if arrastando != "pendulo":
                pendulo.atualizar(DT)

            contador_passos += 1

            if contador_passos % 4 == 0:

                historico_bola.append((
                    bola.x,
                    bola.x_medida
                ))

                historico_pend.append((
                    math.degrees(
                        angulo_normalizado(pendulo.theta)
                    ),
                    math.degrees(
                        angulo_normalizado(
                            pendulo.theta_medida
                        )
                    )
                ))

            acumulador -= DT

    # --------------------------------------
    # DESENHO DA INTERFACE
    # --------------------------------------

    TELA.fill(FUNDO)

    texto(
        "LABORATÓRIO DE CONTROLE PID",
        24, 23, BRANCO, TITULO
    )

    desenhar_bola()
    desenhar_pendulo()

    for slider in sliders:
        slider.desenhar()

    estado_ruido = "ON" if ruido_ativo else "OFF"
    estado_pert = (
        "ON" if perturbacoes_ativas else "OFF"
    )

    texto(
        f"Ruído [N]: {estado_ruido}    "
        f"Perturbações [D]: {estado_pert}",
        670, 35, CINZA, PEQUENA
    )

    texto(
        "Arraste os objetos  |  "
        "1: PID bola  |  2: PID pêndulo  |  "
        "N: ruído  |  D: perturbações  |  "
        "R: reiniciar  |  Espaço: pausar",
        30, 815, CINZA, PEQUENA
    )

    pygame.display.flip()

pygame.quit()
sys.exit()
