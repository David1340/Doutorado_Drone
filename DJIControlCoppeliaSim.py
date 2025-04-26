import coppeliasim_zmqremoteapi_client as zmq
import numpy as np

class DJIControlClient:
    def __init__(self,ip: str, port: int):
        # Conectar ao servidor
        self.client = zmq.RemoteAPIClient()
        self.sim = self.client.require('sim')

        # Nome do robô no CoppeliaSim
        self.drone = self.sim.getObject('/Quadcopter')
        self.camera = self.sim.getObject('/Quadcopter/visionSensor')
        self.controlScript = self.sim.getObject('/Quadcopter/Script')

    def takeOff(self):
        pass

    def enableVirtualStick(self):
        self.sim.startSimulation()
        self.clearVirtualStick()

    def disableVirtualStick(self):
        self.clearVirtualStick()

    def clearVirtualStick(self):
        self.sim.callScriptFunction('cmd_vel',self.controlScript,0.0,0.0,0.0,0.0)

    def setLeftPosition(self, LeftPx: float, LeftPy: float):
        self.sim.callScriptFunction('cmd_vel',self.controlScript,0.0,0.0,LeftPy,LeftPx)

    def setRightPosition(self, RightPx: float, RightPy: float):
        self.sim.callScriptFunction('cmd_vel',self.controlScript,RightPy,RightPx,0.0,0.0)

    def getDronePosition(self):
        position = self.sim.getObjectPosition(self.drone, -1)
        return {'x': position[0], 'y': position[1], 'altitude': position[2]}
    
    def getDroneVelocity(self):
        linear,angular = self.sim.getObjectVelocity(self.drone)
        H = self.sim.getObjectMatrix(self.drone,self.sim.handle_inverse)
        norm_x = np.sqrt(H[0]*H[0] + H[4]*H[4])
        norm_y = np.sqrt(H[1]*H[1] + H[5]*H[5]) 
        H = [H[0]/norm_x, H[1]/norm_y, 0, 0,
                        H[4]/norm_x, H[5]/norm_y, 0, 0,
                        0, 0, 1, 0]
        linear = self.sim.multiplyVector(H, linear)

        return {'x': linear[0], 'y': linear[1], 'z': linear[2]}