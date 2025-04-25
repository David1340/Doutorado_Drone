from shapely.geometry import Polygon, LineString, Point
from shapely import affinity
import numpy as np
import matplotlib.pyplot as plt

def lawmowerPath(polygon,length,width,angle):

    # Polígono original
    if isinstance(polygon, Polygon):
        poly = polygon
    elif isinstance(polygon, list):
        poly = Polygon(polygon)

    # Parâmetros
    step_size = width # largura da faixa do robô
    inter_point_dist = length
    angle_deg = angle  # ângulo da varredura em graus (0 = horizontal, 90 = vertical)

    # Centraliza o polígono na origem antes de rotacionar
    centroid = poly.centroid
    poly_centered = affinity.translate(poly, xoff=-centroid.x, yoff=-centroid.y)

    # Rotaciona o polígono para alinhar com o eixo x
    rotated_poly = affinity.rotate(poly_centered, -angle_deg, origin=(0, 0), use_radians=False)

    # Bounding box do polígono rotacionado
    minx, miny, maxx, maxy = rotated_poly.bounds

    # Geração das linhas de varredura horizontais
    lines = []
    y = miny + step_size / 2
    while y <= maxy:
        line = LineString([(minx - 1, y), (maxx + 1, y)])
        lines.append(line)
        y += step_size

    # Interseção e geração dos waypoints
    paths = []
    for i, line in enumerate(lines):
        intersection = rotated_poly.intersection(line)
        if intersection.is_empty:
            continue
        segs = list(intersection) if intersection.geom_type == 'MultiLineString' else [intersection]

        for seg in segs:
            if(len(seg.coords) > 1):
                x0, y0 = seg.coords[0]
                x1, y1 = seg.coords[1]
                if(x1 < x0):
                    x0, y0 = seg.coords[1]
                    x1, y1 = seg.coords[0]
                x0 = x0 + inter_point_dist/2
                x1 = x1 - inter_point_dist/2
                length = np.hypot(x1 - x0, y1 - y0)
                n_points = int(np.floor(length / inter_point_dist)) + 1

                x_vals = np.linspace(x0, x1, n_points)
                y_vals = np.linspace(y0, y1, n_points)
                coords = list(zip(x_vals, y_vals))
            else:
                coords = list(seg.coords)
            if i % 2 == 1:
                coords.reverse()
            paths.extend(coords)

    # Desrotaciona e descentraliza os waypoints para a orientação original
    waypoints = [affinity.rotate(Point(p), angle_deg, origin=(0, 0), use_radians=False) for p in paths]
    waypoints = [affinity.translate(p, xoff=centroid.x, yoff=centroid.y) for p in waypoints]

    # Converte para lista de listas: [[x1, y1], [x2, y2], ...]
    waypoints_list = [[p.x, p.y] for p in waypoints]

    return waypoints_list

class Proportional_Controller:

    def __init__(self,max,k):
        self.max = max
        self.k = k
    
    def action(self,x):
        return self.max*np.tanh(self.k*x)
    
def fromPiToPi(th):
    if(th > np.pi):
        th = th - 2*np.pi
    if(th < -np.pi): 
       th = th + 2*np.pi
    return th