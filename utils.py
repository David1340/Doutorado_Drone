from shapely.geometry import Polygon, LineString, Point
from shapely import affinity
import numpy as np
import matplotlib.pyplot as plt
from typing import List
import itertools

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
        self.nodes: List[CoverageNode] = []
        self.sensorCamera = sensorCamera
    
    def add_fistLevel_nodes(self, points, Altitude,width,height):
        for x,y in points:
            node = CoverageNode(x, y, Altitude,width,height)
            self.nodes.append(node)

class Shortcut(Tree):
    def __init__(self,sensorCamera: SensorCamera, areas_de_interesse: list):
        super().__init__(sensorCamera)
        self.areas_de_interesse = areas_de_interesse

    def __iter__(self):
        queue = list(self.nodes)
        last_node = None
        while(queue):
            print("Tamando do queue: ", len(queue))
            if(last_node == None or last_node.level == 0):
                print("last_node.level == 0")
                node = queue.pop(0)
                if(queue):
                    nextNode = queue[0]
                else:
                    nextNode = None
                yield node
                last_node = node
                if(node.check_interesse(self.areas_de_interesse,0.01)): # first 
                    candidates = node.found_children_grid(self.sensorCamera)
                    children = []
                    for candidate in candidates:
                        if(candidate.check_interesse(self.areas_de_interesse,0.01)):
                            children.append(candidate)
                    if(children):
                        children = ordem_otima_dos_filhos(last_node,children,nextNode)
                        queue_children = children.copy()
                        while(queue_children):
                            child = queue_children[0]
                            if(len(queue_children) > 1 ):
                                nextNode = queue_children[1]
                            elif(queue):
                                nextNode = queue[0]
                            else:
                                nextNode = None
                            if(last_node.level == 0 or last_node.level == 1):
                                queue_children.pop(0)
                                yield child
                                last_node = child
                                candidates = child.found_children_grid(self.sensorCamera)
                                grandchildren = []
                                for candidate in candidates:
                                    if(candidate.check_interesse(self.areas_de_interesse,0.01)):
                                        grandchildren.append(candidate)
                                if(grandchildren):
                                    grandchildren = ordem_otima_dos_filhos(child,grandchildren,nextNode)
                                    for grandchild in grandchildren:
                                        yield grandchild
                                        last_node = grandchild

                            elif(last_node.level == 2):
                                nextNode = child
                                candidates = child.found_children_grid(self.sensorCamera)
                                distancias = [distancia(last_node,candidate) + distancia(candidate,nextNode) for candidate in candidates]
                                grandchild = candidates[np.argmin(distancias)]
                                candidates.remove(grandchild)
                                yield grandchild
                                last_node = grandchild
                                queue_children.pop(0)
                                if(grandchild.check_interesse(self.areas_de_interesse,0.01)): 
                                    if(queue_children):
                                        nextNode = queue_children[0]
                                    elif(queue):
                                        nextNode = queue[0]
                                        nextChildren = queue[0].found_children_grid(self.sensorCamera)
                                        distancias = [distancia(last_node,candidate) + distancia(candidate,nextNode) for candidate in nextChildren]
                                        nextNode = nextChildren[np.argmin(distancias)]
                                    else:
                                        nextNode = None
                                    grandchildren = ordem_otima_dos_filhos(last_node,candidates,nextNode)
                                    for grandchild in grandchildren:
                                        yield grandchild
                                        last_node = grandchild
                                        
                                else:                             
                                    yield child
                                    last_node = child
                                    
                                    if(queue_children):
                                        nextNode = queue_children[0]
                                    elif(queue):
                                        nextNode = queue[0]
                                    else:
                                        nextNode = None
                                    candidates = child.found_children_grid(self.sensorCamera)
                                    grandchildren = []
                                    for grandchild in candidates:
                                        if(grandchild.check_interesse(self.areas_de_interesse,0.01)):
                                          grandchildren.append(grandchild) 
                                    if(grandchildren): 
                                        
                                        grandchildren = ordem_otima_dos_filhos(last_node,grandchildren,nextNode)
                                        for grandchild in grandchildren:
                                            yield grandchild
                                            last_node = grandchild
            elif(last_node.level == 1):
                print("last_node.level == 1")
                node = queue[0]
                if(len(queue)>1):
                    nextNode = queue[1]
                else:
                    nextNode = None
                children = node.found_children_grid(self.sensorCamera)
                distancias = [distancia(last_node,candidate) + distancia(candidate,node) for candidate in children]
                child = children[np.argmin(distancias)]
                children.remove(child)
                yield child
                last_node = child
                children = ordem_otima_dos_filhos(last_node,children,nextNode)
                if(child.check_interesse(self.areas_de_interesse,0.01)):
                    candidates = child.found_children_grid(self.sensorCamera)
                    grandchildren = []
                    for grandchild in candidates:
                        if(grandchild.check_interesse(self.areas_de_interesse,0.01)):
                            grandchildren.append(grandchild)
                    if(grandchildren):
                        if(children):
                            grandchildren = ordem_otima_dos_filhos(last_node,grandchildren,children[0])
                        else:
                            grandchildren = ordem_otima_dos_filhos(last_node,grandchildren,nextNode)
                    queue.pop(0)
                    for grandchild in grandchildren:
                        yield grandchild
                        last_node = grandchild
                    queue_children = children.copy()
                else:
                    yield node
                    last_node = node
                    queue.pop(0)
                    candidates = node.found_children_grid(self.sensorCamera)
                    queue_children = []
                    for child in candidates:
                        if(child.check_interesse(self.areas_de_interesse,0.01)):
                            queue_children.append(child)
                    
                    if(queue_children):
                        queue_children = ordem_otima_dos_filhos(last_node,queue_children,nextNode) 

                while(queue_children):
                    child = queue_children[0]

                    if(len(queue_children) > 1):
                        nextNode = queue_children[1]
                    elif(queue):
                        nextNode = queue[0]
                    else:
                        nextNode = None
                    if(last_node.level == 0 or last_node.level == 1):
                        queue_children.pop(0)
                        yield child
                        last_node = child
                        candidates = child.found_children_grid(self.sensorCamera)
                        grandchildren = []
                        for candidate in candidates:
                            if(candidate.check_interesse(self.areas_de_interesse,0.01)):
                                grandchildren.append(candidate)
                        if(grandchildren):
                            grandchildren = ordem_otima_dos_filhos(child,grandchildren,nextNode)
                            for grandchild in grandchildren:
                                yield grandchild
                                last_node = grandchild

                    elif(last_node.level == 2):
                        candidates = child.found_children_grid(self.sensorCamera)
                        distancias = [distancia(last_node,candidate) + distancia(candidate,child) for candidate in candidates]
                        grandchild = candidates[np.argmin(distancias)]
                        candidates.remove(grandchild)
                        yield grandchild
                        last_node = grandchild
                        queue_children.pop(0)
                        #nextNode = child
                        if(grandchild.check_interesse(self.areas_de_interesse,0.01)): 
                            grandchildren = ordem_otima_dos_filhos(last_node,candidates,nextNode)
                            for grandchild in grandchildren:
                                yield grandchild
                                last_node = grandchild
                        else:
                            
                            yield child
                            last_node = child
                            if(queue_children):
                                nextNode = queue_children[0]
                            elif(len(queue) > 1):
                                nextNode = queue[1]
                            else:
                                nextNode = None
                            grandchildren = []
                            candidates = child.found_children_grid(self.sensorCamera)
                            for candidate in candidates:
                                if(candidate.check_interesse(self.areas_de_interesse,0.01)):
                                    grandchildren.append(candidate)
                            if(grandchildren):
                                grandchildren = ordem_otima_dos_filhos(last_node,candidates,nextNode)
                                for grandchild in grandchildren:
                                    yield grandchild
                                    last_node = grandchild
            elif(last_node.level == 2):
                node = queue[0]
                if(len(queue) > 1):
                    nextNode = queue[1]
                else:
                    nextNode = None
                print("last_node.level == 2")
                children = node.found_children_grid(self.sensorCamera)
                distancias = [distancia(last_node,child) + distancia(child,node) for child in children] 
                child = children[np.argmin(distancias)]
                children.remove(child)
                grandchildren = child.found_children_grid(self.sensorCamera)
                distancias = [distancia(last_node,grandchild) + distancia(grandchild,child) for grandchild in grandchildren] 
                grandchild = grandchildren[np.argmin(distancias)]
                grandchildren.remove(grandchild)

                yield grandchild
                last_node = grandchild
                
                if(grandchild.check_interesse(self.areas_de_interesse,0.01)):
                    children = ordem_otima_dos_filhos(last_node,children,nextNode)
                    nextNode = children[0]
                    grandchildren = ordem_otima_dos_filhos(last_node,grandchildren,nextNode)
                    for grandchild in grandchildren:
                        yield grandchild
                        last_node = grandchild
                else:
                    yield child
                    last_node = child
                    if(children):
                        nextNode = children[0]
                    if(child.check_interesse(self.areas_de_interesse,0.01)):   
                        candidates = child.found_children_grid(self.sensorCamera)
                        grandchildren = []
                        for grandchild in candidates:
                            if(grandchild.check_interesse(self.areas_de_interesse,0.01)):
                                grandchildren.append(grandchild)
                        if(grandchildren):
                            grandchildren = ordem_otima_dos_filhos(last_node,grandchildren,nextNode)
                        for grandchild in grandchildren:
                            yield grandchild
                            last_node = grandchild
                        
                    else:
                        yield node
                        last_node = node
                        candidates = node.found_children_grid(self.sensorCamera)
                        children = []
                        for child in candidates:
                            if(child.check_interesse(self.areas_de_interesse,0.01)):
                                children.append(child)
                        if(children):
                            children = ordem_otima_dos_filhos(child,children,nextNode)
                            
                        else:
                            continue
                
                queue.pop(0)
                queue_children = []
                if(last_node.level == 0):
                    queue_children = children.copy()
                elif(last_node.level == 1):
                    for child in children:
                        if(child.check_interesse(self.areas_de_interesse,0.01)):
                            queue_children.append(child)
                else:
                    queue_children = children.copy()

                if(queue):
                    nextNode = queue[0]
                else:
                    nextNode = None
                if(queue_children):
                    queue_children = ordem_otima_dos_filhos(last_node,queue_children,nextNode)                

                while(queue_children):
                    child = queue_children.pop(0)
                    if(queue_children):
                        nextNode = queue_children[0]
                    elif(queue):
                        nextNode = queue[0]
                        nextChildren = nextNode.found_children_grid(self.sensorCamera)
                        nextChildren = ordem_otima_dos_filhos(child,nextChildren,nextNode)
                        nextNode = nextChildren[0]
                    else:
                        nextNode = None
                    if(last_node.level == 1 or last_node.level == 0):
                        yield child
                        last_node = child
                        candidates = child.found_children_grid(self.sensorCamera)
                        grandchildren = []
                        for candidate in candidates:
                            if(candidate.check_interesse(self.areas_de_interesse,0.01)):
                                grandchildren.append(candidate)
                        if(grandchildren):
                            grandchildren = ordem_otima_dos_filhos(last_node,grandchildren,nextNode)
                            for grandchild in grandchildren:
                                yield grandchild
                                last_node = grandchild
                    elif(last_node.level == 2):
                        candidates = child.found_children_grid(self.sensorCamera)
                        distancias = [distancia(last_node,grandchild) + distancia(grandchild,child) for grandchild in candidates]                       
                        grandchild = candidates[np.argmin(distancias)]
                        candidates.remove(grandchild)
                        yield grandchild
                        last_node = grandchild

                        if(grandchild.check_interesse(self.areas_de_interesse,0.01)):
                            grandchildren = ordem_otima_dos_filhos(last_node,candidates,nextNode)
                            for grandchild in grandchildren:
  
                                yield grandchild
                                last_node = grandchild
                        else:
                            yield child
                            last_node = child
                            candidates = child.found_children_grid(self.sensorCamera)
                            grandchildren = []
                            for grandchild in candidates:
                                if(grandchild.check_interesse(self.areas_de_interesse,0.01)):
                                    grandchildren.append(grandchild)   
                            if(grandchildren):
                                grandchildren = ordem_otima_dos_filhos(last_node,grandchildren,nextNode)
                                for grandchild in grandchildren:                        
                                    yield grandchild
                                    last_node = grandchild
            else:
                print("problem last_node.level == ", last_node.level)

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
        self.visited = False

        self.polygon = Polygon([(x - height / 2, y - width / 2),
                                (x - height / 2, y + width / 2),
                                (x + height / 2, y + width / 2),
                                (x + height / 2, y - width / 2)])

    def __repr__(self):
        return f"Node(level={self.level}, pos=({self.x:.1f},{self.y:.1f}), Altitude={self.Altitude:.1f}, children={len(self.children)})"
    
    def found_children_grid(self,sensorCamera) -> List['CoverageNode']:
        cell_w = self.width/2
        cell_h = self.height/2

        hc = sensorCamera.l_inv(cell_w,cell_h)

        offset_x = cell_h / 2
        offset_y = cell_w / 2
        children = []

        
        child2 = CoverageNode(self.x - offset_x, self.y + offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        child1 = CoverageNode(self.x + offset_x, self.y + offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        child3= CoverageNode(self.x - offset_x, self.y - offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        child4 = CoverageNode(self.x + offset_x, self.y - offset_y, hc, cell_w, cell_h,
                                     level=self.level + 1, parent=self)
        children.extend([child1,child2,child3,child4])
        return children
    
    def add_children(self, node):
        node.parent = self
        node.level = self.level + 1
        self.children.append(node)
    
    def check_interesse(self, poligonos, min_area=0.01):
        return any(p.intersection(self.polygon).area / self.polygon.area >= min_area for p in poligonos)

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

def distancia(a, b):
    return np.hypot(a.x - b.x, a.y - b.y)

def ordem_otima_dos_filhos(pai, filhos, proximo_pai):
    melhor_ordem = None
    menor_distancia = float('inf')

    for perm in itertools.permutations(filhos):
        # Distância de pai → primeiro filho
        dist = distancia(pai, perm[0])
        # Distâncias entre filhos
        for i in range(len(perm)-1):
            dist += distancia(perm[i], perm[i+1])
        # Último filho → proximo_pai
        if(proximo_pai):
            dist += distancia(perm[-1], proximo_pai)

        if dist < menor_distancia:
            menor_distancia = dist
            melhor_ordem = perm

    return list(melhor_ordem)