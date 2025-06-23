from shapely.geometry import Polygon
import numpy as np
import matplotlib.pyplot as plt
from utils import lawmowerPath, Proportional_Controller, fromPiToPi
import coppeliasim_zmqremoteapi_client as zmq
import sys
import time

# Horizontal coverage at ground leve 17.32m
# Vertical coverage at ground level 12.99m

# Polígono de teste
poly = Polygon([(-50, -50), (50, -50), (50, 50), (-50, 50)])

# Parâmetros
width = 17.32 # largura da faixa do robô
heigh = 12.99
angle_deg = 0  # ângulo da varredura em graus (0 = horizontal, 90 = vertical)
_waypoints = lawmowerPath(poly, heigh, width, angle_deg)
waypoints = np.array(_waypoints)

# Plot final
x, y = poly.exterior.xy
plt.plot(x, y, color='black')
if poly.interiors:
    for interior in poly.interiors:
        x, y = interior.xy
        plt.plot(x, y, color='black')

plt.plot(waypoints[:,0], waypoints[:,1], '-o', color='blue', markersize=2)
plt.axis('equal')
plt.title(f"Trajetória com varredura a {angle_deg}°")
#plt.show()

# Conectar ao servidor
client = zmq.RemoteAPIClient()
sim = client.require('sim')
sim.loadScene('./scenes/Drone/base.ttt')
time.sleep(1.0)
sim.startSimulation()
# Nome do robô no CoppeliaSim
drone = sim.getObject('/Quadcopter')
camera = sim.getObject('/Quadcopter/visionSensor')
controlScript = sim.getObject('/Quadcopter/Script')

#parãmetros do controlador
tamos = 0.15
erro_min = 0.1
zD = 12
cz = Proportional_Controller(max=0.3,k=1)
cx = Proportional_Controller(max=0.2,k=0.5)
cw = Proportional_Controller(max=0.25,k=3)
xD = waypoints[0,0]
yD = waypoints[0,1]

#Obtém a altitude desejada e gira em em direção do primeio waypoint
while True:
    sim.step()
    inicio = sim.getSimulationTime()
    position = np.array(sim.getObjectPosition(drone, -1)) # Obtém a posição do drone no referencial global (-1)
    xR,yR = position[0], position[1]
    thD = np.arctan2(yD - yR, xD - xR)
    orientation = sim.getObjectOrientation(drone)
    yaw,pitch,roll = sim.alphaBetaGammaToYawPitchRoll(orientation[0],orientation[1],orientation[2])
    thR = yaw
    thR = fromPiToPi(thR)
    th = thD - thR
    th = fromPiToPi(th)
    erro_z = zD - position[2]
    ux = 0.0
    uy = 0.0
    uz = cz.action(erro_z)
    up = cw.action(th)

    print(f"\033[2A\033[KErro em z: {erro_z:.3f}\nuz: {uz:.3f}")
    if(np.abs(erro_z) < erro_min):
        break

    try:
        sim.callScriptFunction('cmd_vel',controlScript,ux,
                                uy,uz,-up)
    except Exception as e:
        print(f"[AVISO] Erro ao chamar 'cmd_vel': {e}")

    while(sim.getSimulationTime() - inicio < tamos):
        pass

uz = 0.0
sim.callScriptFunction('cmd_vel',controlScript,0.0,0.0,0.0,0.0)

tempo_max = 60.0  # tempo máximo por waypoint, em segundos
trajetoria_x = []
trajetoria_y = []
parar_tudo = False
for i, waypoint in enumerate(waypoints):
    if(i == 0):
        tempo_inicial = sim.getSimulationTime()
    if parar_tudo:
        break
    # Controle de posição planar (xD,yD)
    xD = waypoint[0]
    yD = waypoint[1]

    inicio_global = sim.getSimulationTime()  # tempo de início global desse waypoint

    while(True):
        sim.step()
        tempo_atual = sim.getSimulationTime()
        
        # Obter a posição
        position = np.array(sim.getObjectPosition(drone, -1))
        xR, yR = position[0], position[1]
        trajetoria_x.append(xR)
        trajetoria_y.append(yR)
        d = np.sqrt((xD - xR)**2 + (yD - yR)**2)
        thD = np.arctan2(yD - yR, xD - xR)
        orientation = sim.getObjectOrientation(drone)

        yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(orientation[0], orientation[1], orientation[2])
        thR = fromPiToPi(yaw)
        th = fromPiToPi(thD - thR)

        uz = 0.0
        uy = 0.0    
        up = round(cw.action(th),3)
        ux = round(0.2 * np.power(np.cos(th / 2), 100),2) #round(cx.action(d) * np.power(np.cos(th / 2), 100),2)

        print(f"\033[3A\033[Kd: {d:.2f},th: {(180/np.pi)*th:.1f}\n\033[Kux: {ux},up: {up}\n\033[Kwaypoint[{i}]: {waypoint} ")

        try:
            sim.callScriptFunction('cmd_vel', controlScript, ux, uy, uz, -up)
        except Exception as e:
            print(f"[AVISO] Erro ao chamar 'cmd_vel': {e}")

        if np.abs(d) < 10 * erro_min:
            tempo_final = sim.getSimulationTime()
            minutos = (tempo_final - tempo_inicial) / 60
            print(f"Concluído em {minutos:.2f} minutos",)
            break

        # Critério de tempo: se exceder tempo_max, para
        if tempo_atual - inicio_global > tempo_max:
            print(f"[AVISO] Tempo máximo ({tempo_max}s) excedido para o waypoint {i}.")
            parar_tudo = True
            break

        while sim.getSimulationTime() - tempo_atual < tamos:
            pass

try:
    sim.callScriptFunction('cmd_vel', controlScript, 0.0, 0.0, 0.0, 0.0)
except Exception as e:
    print(f"[AVISO] Erro ao chamar 'cmd_vel': {e}")

plt.figure(figsize=(8, 6))
plt.plot(trajetoria_x, trajetoria_y, label='Trajetória do drone', color='blue')
waypoints_x = [wp[0] for wp in waypoints]
waypoints_y = [wp[1] for wp in waypoints]
plt.scatter(waypoints_x, waypoints_y, color='red', label='Waypoints', marker='x')

plt.title('Trajetória do Drone')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()