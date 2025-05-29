from shapely.geometry import Polygon, LineString, Point
from shapely import affinity
import numpy as np
import matplotlib.pyplot as plt

class SensorCamera():
    def __init__(self, width = 9.6, height = 7.2, pixels_width = 4032, pixels_height = 3024, focal = 6.65):
        # width [mm], height [mm], length focal [mm]
        self.width = width
        self.height = height
        self.pixels_width = pixels_width
        self.pixels_height = pixels_height
        self.focal = focal


    def l(self,h): #dado um valor de altitude, calcula as dimensões em metro da área vista
        GSD = self.GSD(h)
        return GSD*self.pixels_width, GSD*self.pixels_height

    def l_inv(self,width, height):
        GSDw = width / self.pixels_width
        GSDh = height / self.pixels_height
        return max(self.h_desirable(GSDw),self.h_desirable(GSDh))

    def h_desirable(self,GSD = 0.43): #cm/px
        iw,ih = self.pixels_width,self.pixels_height 
        sw,sh = self.width,self.height  #mm
        f = self.focal #mm
        return min(GSD*f*iw/sw, GSD*f*ih/sh) #[cm]

    def GSD(self,h): # a dimensão da saída é [h]/px
        return h*self.width/(self.focal*self.pixels_width) #[h]/px

class Tree:
    def __init__(self,sensorCamera:SensorCamera):
        self.nodes = []
        self.sensorCamera = sensorCamera
    
    def add_fistLevel_nodes(self, points, Altitude,width,height):
        for x,y in points:
            node = CoverageNode(x, y, Altitude,width,height)
            self.nodes.append(node)

class CoverageNode:
    def __init__(self, x, y, Altitude, width, height, level=0, parent=None):
        self.x = x
        self.y = y
        self.point = Point([x,y])
        self.Altitude = Altitude  # altitude do drone
        self.width = width
        self.height = height
        self.level = level
        self.parent = parent
        self.children = []

        self.polygon = Polygon([(x - height / 2, y - width / 2),
                                (x - height / 2, y + width / 2),
                                (x + height / 2, y + width / 2),
                                (x + height / 2, y - width / 2)])

    def __repr__(self):
        return f"Node(level={self.level}, pos=({self.x:.1f},{self.y:.1f}), Altitude={self.Altitude:.1f}, children={len(self.children)})"
    
    def found_children_grid(self,b,sensorCamera,sentido):
        cell_w = self.width / b
        cell_h = self.height / b

        hc = sensorCamera.l_inv(cell_w,cell_h)

        offset_x = cell_h / 2
        offset_y = cell_w / 2
        children = []
        '''
        for j in range(b):
            for i in range(b):
                cx = self.x - offset_x + (i + 0.5) * cell_w
                cy = self.y - offset_y + (j + 0.5) * cell_h
                child = CoverageNode(cx, cy, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)'''
        child1 = CoverageNode(self.x - sentido*offset_x, self.y - offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        child2 = CoverageNode(self.x - sentido*offset_x, self.y + offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        child3 = CoverageNode(self.x + sentido*offset_x, self.y + offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        child4 = CoverageNode(self.x + sentido*offset_x, self.y - offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        children.append(child1)
        children.append(child2)
        children.append(child3)
        children.append(child4)
        return children
    
    def add_children(self, node):
        self.children.append(node)

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
    while y <= maxy + step_size / 2:
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
                length = np.ceil(length / inter_point_dist) * inter_point_dist
                n_points = int(np.floor(length / inter_point_dist)) + 1

                '''
                x_vals = np.arange(x0, x1 + inter_point_dist, inter_point_dist)
                y_vals = np.arange(y0, y1 + inter_point_dist, inter_point_dist)
                if len(x_vals) != len(y_vals):
                    # Ajusta o tamanho dos arrays para garantir que tenham o mesmo comprimento
                    if len(x_vals) > len(y_vals):
                        x_vals = x_vals[:len(y_vals)]
                    else:
                        y_vals = y_vals[:len(x_vals)]
                        '''
                x_vals = np.linspace(x0, x0 + length, n_points)
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

