from shapely.geometry import Polygon, LineString, Point
from shapely import affinity
import numpy as np
import matplotlib.pyplot as plt
from typing import List
import itertools
from hilbertcurve.hilbertcurve import HilbertCurve

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
    
    def toHilbertCurve(self,p):
        # 1. Definir o grid
        #p = 3  # Número de iterações → grid 2^3 = 8x8
        N = 2  # Dimensão (2D)
        hilbert_curve = HilbertCurve(p, N)
        # 2. Encontrar os limites dos waypoints
        x_list = [n.x for n in self.nodes]
        y_list = [n.y for n in self.nodes]
        xmin, xmax = min(x_list), max(x_list)
        ymin, ymax = min(y_list), max(y_list)

        hilbert_keys = []
        for point in self.nodes:
            grid_point = Tree.xy_to_grid(point.x, point.y,xmin,xmax,ymin,ymax,p)
            key = hilbert_curve.distance_from_point(grid_point)
            hilbert_keys.append(key)
        self.nodes = [point for _, point in sorted(zip(hilbert_keys, self.nodes))]


    def xy_to_grid(x,y,xmin,xmax,ymin,ymax,p):
        gx = int(((x - xmin) / (xmax - xmin)) * (2**p - 1))
        gy = int(((y - ymin) / (ymax - ymin)) * (2**p - 1))
        return [gx, gy]

class BreadthFirst(Tree):
    def __init__(self,sensorCamera: SensorCamera, areas_de_interesse: list):
        super().__init__(sensorCamera)
        self.areas_de_interesse = areas_de_interesse
        self.horizonte = 5

    def generate_path(self):
        waypoints = []
        children = []
        grandchildren = []
        for node in self.nodes:
            
            waypoints.append([node.x, node.y, node.Altitude])

            if node.check_interesse(self.areas_de_interesse):
                candidatos = node.found_children_grid(self.sensorCamera)
                for child in candidatos:
                    if(child.check_interesse(self.areas_de_interesse,0.01)):
                        children.append(child)
                        candidatos2 = child.found_children_grid(self.sensorCamera)
                        for grandchild in candidatos2:
                            if(grandchild.check_interesse(self.areas_de_interesse)):
                                grandchildren.append(grandchild)
        children.reverse()
        children_remaining = children.copy()
        last_node = self.nodes[-1]
        while children_remaining:
            janela = children_remaining[:self.horizonte]
            next_node = children_remaining[self.horizonte] if len(children_remaining) > self.horizonte else None

            otima_ordem = ordem_otima_dos_filhos(last_node, janela, next_node)

            chosen = otima_ordem[0]
            waypoints.append([chosen.x, chosen.y, chosen.Altitude])
            children_remaining.remove(chosen)
            last_node = chosen

        grandchildren_remaining = grandchildren.copy()
        while grandchildren_remaining:
            janela = grandchildren_remaining[:self.horizonte]
            next_node = grandchildren_remaining[self.horizonte] if len(grandchildren_remaining) > self.horizonte else None

            otima_ordem = ordem_otima_dos_filhos(last_node, janela, next_node)

            chosen = otima_ordem[0]
            waypoints.append([chosen.x, chosen.y, chosen.Altitude])
            grandchildren_remaining.remove(chosen)
            last_node = chosen

        return waypoints

class DeepFirst(Tree):
    def __init__(self,sensorCamera: SensorCamera, areas_de_interesse: list):
        super().__init__(sensorCamera)
        self.areas_de_interesse = areas_de_interesse
    def generate_path(self):
        queue = list(self.nodes)
        for i, node in enumerate(queue):
            if(i < len(queue) -1):
                self.deepFirst(node, self.sensorCamera,0.01,queue[i+1])
            else:
                self.deepFirst(node, self.sensorCamera,0.01,None)
        waypoints = []
        for node in self.nodes:
            waypoints.append([node.x, node.y, node.Altitude])
            for child in node.children:
                waypoints.append([child.x, child.y, child.Altitude])
                for child2 in child.children:
                    waypoints.append([child2.x, child2.y, child2.Altitude])
        return waypoints

    def deepFirst(self, node, camera, min_area, nextNode = None):
        if node.check_interesse(self.areas_de_interesse,min_area):
            candidatos = node.found_children_grid(camera)
            filhos = []
            for filho in candidatos:
                if(filho.check_interesse(self.areas_de_interesse,min_area)):
                    filhos.append(filho)
            if filhos:
                filhos = ordem_otima_dos_filhos(node,filhos,nextNode)
            for i, filho in enumerate(filhos):
                node.add_children(filho)
                if(node.level < 1):
                    if(i < len(filhos) - 1):
                        self.deepFirst(filho, camera, min_area, filhos[i+1])
                    else:
                        self.deepFirst(filho, camera,min_area, nextNode)

class Shortcut(Tree):
    def __init__(self,sensorCamera: SensorCamera, areas_de_interesse: list):
        super().__init__(sensorCamera)
        self.areas_de_interesse = areas_de_interesse

    def generate_path(self):
        waypoints = []
        for i,waypoint in enumerate(self):
            waypoints.append([waypoint.x,waypoint.y,waypoint.Altitude])
            if(i == 91):
                pass
        return waypoints

    def __iter__(self):
        queue = list(self.nodes)
        last_node = None
        while(queue):
            #print("Tamando do queue: ", len(queue))
            if(last_node == None or last_node.level == 0):
                #print("last_node.level == 0")
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
                #print("last_node.level == 1")
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
                #print("last_node.level == 2")
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
        return f"Node(level={self.level}, pos=({self.x:.1f},{self.y:.1f}), Altitude={self.Altitude:.1f}, children={len(self.children)}, Visited = {self.visited})"
    
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
    
    def add_children(self, node: 'CoverageNode', visited = True):
        node.parent = self
        node.level = self.level + 1
        node.visited = visited
        self.children.append(node)
    
    def check_interesse(self, poligonos, min_area=0.01):
        return any(p.intersection(self.polygon).area / self.polygon.area >= min_area for p in poligonos)

class HilbertTree(Tree):
    def __init__(self,sensorCamera: SensorCamera, areas_de_interesse: list):
        super().__init__(sensorCamera)
        self.areas_de_interesse = areas_de_interesse
        self.HilbertMap1 = None
        self.HilbertMap2 = None
        self.HilbertMap3 = None
    
    def add_another_levels(self):
        self.HilbertMap1 = HilbertMapping(2,self.nodes)
        all_children = []
        all_grandchildren = []
        for node in self.nodes:
            children = node.found_children_grid(self.sensorCamera)
            for child in children:
                all_children.append(child)
                node.add_children(child, False)
                grandchildren = child.found_children_grid(self.sensorCamera)
                for grandchild in grandchildren:
                    child.add_children(grandchild,False)
                    all_grandchildren.append(grandchild)
        self.HilbertMap2 = HilbertMapping(2,all_children)
        self.HilbertMap3 = HilbertMapping(2,all_grandchildren)
 
        self.nodes = self.HilbertMap1.sortNodes(self.nodes)
        for node in self.nodes:
            node.children = self.HilbertMap2.sortNodes(node.children)
            for child in node.children:
                child.children = self.HilbertMap3.sortNodes(child.children)

    def __iter__(self):
        last_node = None
        while(True):
            last_node = self.Hilbert_coverage(last_node)
            
            if(last_node != None):
                last_node.visited = True
                if(last_node.parent != None):
                        last_node.parent.visited = True
                yield last_node
            else:
                break

    def Hilbert_coverage(self, last_node: CoverageNode):
        if(last_node == None):
            return self.nodes[0]
        if(last_node.check_interesse(self.areas_de_interesse)):   
            for child in last_node.children:
                if(not child.check_interesse(self.areas_de_interesse)):
                    child.visited = True
                    if(child.level == 1):
                        for grandchild in child.children:
                            grandchild.visited = True
            if(self.NeedVisit(last_node.children)):
                if(last_node.children[0].visited == True):
                    return self.NextNotVisited(last_node.children[0])
                else:
                    return last_node.children[0]
            else:
                return self.NextNotVisited(last_node)
                
        else:
            if(not self.islast_node(last_node)):
                
                if(last_node.parent == None):
                    return self.NextNotVisited(last_node)
                else:
                    return last_node.parent
            else:
                while(last_node.parent != None):            
                    if(self.islast_node(last_node)):
                        last_node = last_node.parent
                    else:
                        return self.NextNotVisited(last_node)
                return self.NextNotVisited(last_node)

    def islast_node(self,last_node: CoverageNode):
        if(last_node.parent == None):
            return last_node == self.nodes[-1]
        else:
            return last_node == last_node.parent.children[-1]

    def Next(self,last_node: CoverageNode):

        if(last_node.level == 0):
            n = self.nodes.index(last_node)
            n += 1
            if(last_node != self.nodes[-1]):
                return self.nodes[n]
            else:
                return None
        elif(last_node.level == 1):
            n = last_node.parent.children.index(last_node) 
            n += 1 
            if(last_node != last_node.parent.children[-1]):  
                return last_node.parent.children[n]
            else:
                n = self.nodes.index(last_node.parent)
                n += 1
                if(last_node.parent != self.nodes[-1]):
                    return self.nodes[n].children[0]
                else:
                    return None
        elif(last_node.level == 2):
            n = last_node.parent.children.index(last_node) 
            n += 1 
            if(last_node != last_node.parent.children[-1]):  
                return last_node.parent.children[n]
            
            n = last_node.parent.parent.children.index(last_node.parent)
            n += 1
            if(last_node.parent != last_node.parent.parent.children[-1]):
                return  last_node.parent.parent.children[n].children[0]
            else:
                n = self.nodes.index(last_node.parent.parent)
                n += 1
                if(n < len(self.nodes)):
                    return self.nodes[n].children[0].children[0]
                else:
                    return None

    def NextNotVisited(self,last_node: CoverageNode):
        if(last_node == None):
            return None
        next = self.Next(last_node)
        if(next == None):
            return None
        while(next.visited == True):
            next = self.Next(next)
            if(next == None):
                return None
        return next
    
    def NeedVisit(self,children):
        if(isinstance(children, list)):
            return any(child.visited == False for child in children)
        elif(isinstance(children, CoverageNode)):
            return children.visited == False
        elif(children == None):
            return False
        else:
            print(children)
            print("Erro em NeedVisit")

class HilbertMapping():
    def __init__(self,N,nodes):
        self.p = int(np.ceil(np.log2(len(nodes)) / 2)) + 1
        self.hilbert_curve = HilbertCurve(self.p, N)

        x_list = [n.x for n in nodes]
        y_list = [n.y for n in nodes]
        self.xmin, self.xmax = min(x_list), max(x_list)
        self.ymin, self.ymax = min(y_list), max(y_list)
    
    def sortNodes(self,nodes: List[CoverageNode]):
        hilbert_keys = []
        for point in nodes:
            grid_point = self.xy_to_grid(point.x, point.y)
            key = self.hilbert_curve.distance_from_point(grid_point)
            hilbert_keys.append(key)
        nodes = [point for _, point in sorted(zip(hilbert_keys, nodes), key=lambda x: x[0])]
        return nodes

    def xy_to_grid(self,x,y):
        gx = int(((x - self.xmin) / (self.xmax - self.xmin)) * (2**self.p - 1))
        gy = int(((y - self.ymin) / (self.ymax - self.ymin)) * (2**self.p - 1))
        return [gx, gy]

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