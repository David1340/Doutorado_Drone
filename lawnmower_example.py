from shapely.geometry import Polygon
import numpy as np
import matplotlib.pyplot as plt
from utils import lawmowerPath


# Polígono de teste
poly = Polygon([(0, 0), (10, 0), (10, 5), (5, 8), (3, 8), (0, 5)])


# Parâmetros
width = 1.0 # largura da faixa do robô
heigh = 0.5
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
plt.show()
