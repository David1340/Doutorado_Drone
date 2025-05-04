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

# Definir raio de renderização (exemplo: 2 tiles de distância)
render_radius = 2 * 12  # 2 tiles * 12 unidades por tile

robot_pos = get_robot_position(sim, robot_name)
robot_x, robot_y = robot_pos[0], robot_pos[1]

# Inicializa o pygame e o módulo de joystick
pygame.init()
pygame.joystick.init()

# Seleciona o primeiro joystick disponível
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Obtém o identificador do drone
drone = sim.getObject('/Quadcopter')

pygame.event.pump()
ex_const = joystick.get_axis(0)
ey_const = joystick.get_axis(1)
dx_const = joystick.get_axis(2)
dy_const = joystick.get_axis(3)

while True:#sim.getSimulationTime() < 10*60:
    sim.step()  # Atualizar a simulação
    inicio = sim.getSimulationTime()
    pygame.event.pump()  # Atualiza os eventos do pygame    

    # Lê os valores dos analógicos (geralmente eixo 0 e 1 são o analógico esquerdo, e 2 e 3 o direito)
    eixo_esquerdo_x = round(joystick.get_axis(0),2)  # Esquerda (-1) / Direita (+1)
    eixo_esquerdo_y = round(joystick.get_axis(1),2)  # Cima (-1) / Baixo (+1)
    eixo_direito_x = round(joystick.get_axis(2),2) # Esquerda (-1) / Direita (+1)
    eixo_direito_y = round(joystick.get_axis(3),2)  # Cima (-1) / Baixo (+1)
    print(eixo_esquerdo_x, eixo_esquerdo_y, eixo_direito_x, eixo_direito_y)
    position = np.array(sim.getObjectPosition(drone, -1)) # Obtém a posição do drone no referencial global (-1)

    handleScript = sim.getObject('/Quadcopter/Script')
    try:
        sim.callScriptFunction('cmd_vel',handleScript,-eixo_direito_y,
                               eixo_direito_x,-eixo_esquerdo_y,eixo_esquerdo_x)
    except Exception as e:
        print(f"[AVISO] Erro ao chamar 'cmd_vel': {e}")
        time.sleep(1.0)