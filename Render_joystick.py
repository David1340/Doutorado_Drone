import coppeliasim_zmqremoteapi_client as zmq
import os
import numpy as np
import re
import cv2
import render
import pygame
import matplotlib.pyplot as plt
import time
def get_robot_position(sim, robot_name):
    handle = sim.getObjectHandle(robot_name)
    return sim.getObjectPosition(handle, -1)  # Posição global do robô

# Conectar ao servidor
client = zmq.RemoteAPIClient()
sim = client.require('sim')

# Inicia a simulação
sim.startSimulation()

# Nome do robô no CoppeliaSim
robot_name = "/Quadcopter"

# Caminho da textura
texture_path = r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\scenes\Drone\tiles"

# Definir raio de renderização (exemplo: 2 tiles de distância)
render_radius = 2 * 12  # 2 tiles * 12 unidades por tile

# Iniciando a renderização
render = render.Render(sim,texture_path,10)
render.create_planes(texture_path)

robot_pos = get_robot_position(sim, robot_name)
robot_x, robot_y = robot_pos[0], robot_pos[1]
render.update_scene(robot_x,robot_y,texture_path)

# Inicializa o pygame e o módulo de joystick
pygame.init()
pygame.joystick.init()

# Seleciona o primeiro joystick disponível
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Obtém o identificador do sensor de visão
camera = sim.getObject('/Quadcopter/visionSensor')

# Obtém o identificador do drone
drone = sim.getObject('/Quadcopter')

motor1 = sim.getObject('/Quadcopter/propeller[0]')
motor2 = sim.getObject('/Quadcopter/propeller[1]')
motor3 = sim.getObject('/Quadcopter/propeller[2]')
motor4 = sim.getObject('/Quadcopter/propeller[3]')

plt.ion()  # Ativa o modo interativo do Matplotlib
fig, ax = plt.subplots()
img_display = None
pygame.event.pump()
ex_const = joystick.get_axis(0)
ey_const = joystick.get_axis(1)
dx_const = joystick.get_axis(2)
dy_const = joystick.get_axis(3)

while True:#sim.getSimulationTime() < 10*60:
    sim.step()  # Atualizar a simulação
    inicio = sim.getSimulationTime()
    pygame.event.pump()  # Atualiza os eventos do pygame    
    image = []
    image, resolution = sim.getVisionSensorImg(camera)
    
    if image:
        # Converte a imagem para um formato adequado para exibição
        img_array = np.frombuffer(image, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
        img_array = np.flipud(img_array)  # Corrige a orientação da imagem

        if img_display is None:
            img_display = ax.imshow(img_array)
        else:
            img_display.set_data(img_array)

    # Lê os valores dos analógicos (geralmente eixo 0 e 1 são o analógico esquerdo, e 2 e 3 o direito)
    eixo_esquerdo_x = round(joystick.get_axis(0),2)  # Esquerda (-1) / Direita (+1)
    eixo_esquerdo_y = round(joystick.get_axis(1),2)  # Cima (-1) / Baixo (+1)
    eixo_direito_x = round(joystick.get_axis(2),2) # Esquerda (-1) / Direita (+1)
    eixo_direito_y = round(joystick.get_axis(3),2)  # Cima (-1) / Baixo (+1)
    print(eixo_esquerdo_x, eixo_esquerdo_y, eixo_direito_x, eixo_direito_y)
    position = np.array(sim.getObjectPosition(drone, -1)) # Obtém a posição do drone no referencial global (-1)
    position[2] += -eixo_esquerdo_y * 0.5
    position[0] += -eixo_direito_y * 0.5
    position[1] += -eixo_direito_x * 0.5

    # Obter posição do robô
    robot_pos = get_robot_position(sim, robot_name)
    robot_x, robot_y = robot_pos[0], robot_pos[1]
    render.update_scene(position[0],position[1],texture_path)
    handleScript = sim.getObject('/Quadcopter/Script')
    try:
        sim.callScriptFunction('cmd_vel',handleScript,-eixo_direito_y,
                               eixo_direito_x,-eixo_esquerdo_y,eixo_esquerdo_x)
    except Exception as e:
        print(f"[AVISO] Erro ao chamar 'cmd_vel': {e}")
        time.sleep(1.0)
    print(sim.getSimulationTime())