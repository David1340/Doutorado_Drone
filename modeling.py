from DJIControlCoppeliaSim import DJIControlClient
import time
from utils import Proportional_Controller
import numpy as np
import csv
import matplotlib.pyplot as plt
#Initializes the drone client 
drone_client = DJIControlClient(ip='10.25.2.17', port=8080)

#Take off and enable virtual stick mode
response = drone_client.takeOff()
print("Takeoff response:", response)
time.sleep(0.5) #wait 5s for the drone finish take off
response = drone_client.enableVirtualStick()
print("Enable Virtual Stick response:", response)

controllerAltitude = Proportional_Controller(max=0.5,k=1.5) #Controller for Altitude (axis z)
desirableAltitude = 5 #Desired altitude [meters]
minimumErrorAltitude = 0.2 #Minimum error for altitude [meters]

#Controle no eixo z
while(True):
    break
    initialTime = time.time()

    # Get drone position
    position = drone_client.getDronePosition()
    gpsAltitude = position['altitude']
    errorAltitude = desirableAltitude - gpsAltitude
    print("errorAltitude: ", errorAltitude," m")
    if(np.abs(errorAltitude) < minimumErrorAltitude):
        break

    # Calculate control command for altitude
    uz = controllerAltitude.action(errorAltitude)
    print("Control command uz:", uz)

    # Send virtual stick control command for altitude
    response = drone_client.setLeftPosition(0.0, uz)

    # Wait for the next iteration (to maintain the desired sampling rate)
    durationTime = time.time() - initialTime
    time.sleep(max(0, 0.1 - durationTime))  # 0.1 seconds sampling rate

response = drone_client.setLeftPosition(0.0, 0.0)
print("Send Virtual Stick Control response:", response)

#Obtaining date for vx velocity modeling
fullTime = 10 #[s]
samplingTime = 0.25 #[s]
dataVx = []
dataVy = []
dataVz = []
dataTime = []
dataUx = []
initialExperimentTimeGlobal = time.time()
ux_previous = 0.0 #Initial value for ux
values =  [0.1,0.0,0.2,0.0,0.3,0.0, 0.4,0.0,0.5,0.0] # #Values for ux to be tested
for ux in values:
    initialExperimentTime = time.time()
    while (time.time() - initialExperimentTime) < fullTime:
        initialTime = time.time()
        dataTime.append(initialTime - initialExperimentTimeGlobal)
        linearVelocity = drone_client.getDroneVelocity()
        vx = linearVelocity.get('x', 0)
        vy = linearVelocity.get('y', 0)
        vz = linearVelocity.get('z', 0)
        dataVx.append(vx)
        dataVy.append(vy)
        dataVz.append(vz)
        dataUx.append(ux_previous)
        drone_client.setRightPosition(ux,0.0)
        durationTime = time.time() - initialTime
        ux_previous = ux
        time.sleep(max(0, samplingTime - durationTime))  # samplingTime in seconds
    
    #drone_client.setRightPosition(0, 0)
    #time.sleep(2.0)  # wait for the drone to stabilize before the next iteration

response = drone_client.setRightPosition(0, 0)
drone_client.disableVirtualStick()


nameFile = 'dados.csv'
with open(nameFile, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['vx','vy','vz','ux', 'time'])  # cabeçalhos (opcional)
    for vx,vy,vz, ux, t in zip(dataVx,dataVy,dataVz, dataUx, dataTime):
        writer.writerow([vx,vy,vz, ux, t])

print("Data saved in ", nameFile)

#Plotting data
fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Plot 1 - Vx
axs[0].plot(dataTime, dataVy, color='blue', marker='o')
axs[0].set_ylabel('Vy')
axs[0].set_title('Vy ao longo do tempo')
axs[0].grid(True)

# Plot 2 - Ux
axs[1].plot(dataTime, dataUx, color='green', marker='s')
axs[1].set_ylabel('Ux')
axs[1].set_title('Ux ao longo do tempo')
axs[1].grid(True)

plt.tight_layout()
plt.show()

