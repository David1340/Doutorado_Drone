from shapely.geometry import Polygon  
from shapely import affinity    
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from utils import SensorCamera,Tree
from utils import lawmowerPath
import random
from utils import Shortcut, DeepFirst, BreadthFirst
from utils import HilbertTree
from shapely.ops import unary_union



##################################### Setup Experimental #########################################
#random.seed(42)  # For reproducibility
#DJImini3Camera = SensorCamera(9.6,9.6,4032,4032)
DJImini3Camera = SensorCamera()
desirableGSD = 1*0.43 #cm/px
altitude = DJImini3Camera.h_desirable(desirableGSD)/100
width,height = DJImini3Camera.l(altitude) # dimensão horizontal do sensor para a altitude desejada
L = 8
poly = Polygon([(-L*height, -L*width), (L*height, -L*width), (L*height, L*width), (-L*height, L*width)])
angle_deg = 0  # ângulo da varredura em graus (0 = horizontal, 90 = vertical)
waypointsLawMower = lawmowerPath(poly, height, width, angle_deg)

base = Polygon([(-height/2, -width/2), (height/2,-width/2), (height/2,width/2), (-height/2,width/2)])

p = 30 # porcentagem de área de interesse
################################## Inicio dos Experimentos ###############################################
quantidade = 100
data_Hilbert, data_Shortcut, data_DepthFirst, data_BreadthFirst, = [],[],[],[]
for _ in range(quantidade):
    areas_de_interesse = []
    print(_)
    interesting_area = 0
    while interesting_area < p/100:

        i = random.normalvariate(0.0,0.3) #random.uniform(-0.95,0.95)
        j = random.normalvariate(0.0,0.3) #random.uniform(-0.95,0.95)     
        poly_rand = affinity.translate(base, i*L*height, j*L*width)
        areas_de_interesse.append(poly_rand)
        areas_unidas = unary_union(areas_de_interesse)
        interesting_area = poly.intersection(areas_unidas).area
        interesting_area = interesting_area /poly.area
    areas_de_interesse2 = [areas_unidas]
    altitude2 = DJImini3Camera.h_desirable(4*desirableGSD)/100
    width2,height2 = DJImini3Camera.l(altitude2) # dimensão horizontal do sensor para a altitude desejada
    waypointsInitial = lawmowerPath(poly, height2, width2, angle_deg)

    LawnmowerTree = Tree(sensorCamera=DJImini3Camera)
    LawnmowerTree.add_fistLevel_nodes(waypointsLawMower, altitude,width,height)

    BFTree = BreadthFirst(DJImini3Camera,areas_de_interesse2)
    BFTree.add_fistLevel_nodes(waypointsInitial, altitude2,width2,height2)

    ShortTree = Shortcut(DJImini3Camera,areas_de_interesse2)
    ShortTree.add_fistLevel_nodes(waypointsInitial, altitude2,width2,height2)

    DFTree = DeepFirst(DJImini3Camera,areas_de_interesse2)
    DFTree.add_fistLevel_nodes(waypointsInitial, altitude2,width2,height2)

    DFTreeHilbert = HilbertTree(DJImini3Camera,areas_de_interesse2)
    DFTreeHilbert.add_fistLevel_nodes(waypointsInitial, altitude2,width2,height2)
    DFTreeHilbert.add_another_levels()


    waypoints = np.array(waypointsLawMower)
    path_length_LawMower = 0
    for i in range(np.size(waypoints,0)-1):
        path_length_LawMower += np.linalg.norm(waypoints[i+1] - waypoints[i])
   # print("Coverage Path length (LawMower): ", path_length_LawMower/1000, " km")

    waypointsDF = np.array(DFTree.generate_path())
    path_length_DeepFirst = 0
    for i in range(np.size(waypointsDF,0)-1):
        path_length_DeepFirst += np.linalg.norm(waypointsDF[i+1] - waypointsDF[i])
    #print("Coverage Path length (Depth First): ", path_length4/1000, " km")

    waypointsBF = np.array(BFTree.generate_path())
    path_length_BreadthFirst = 0
    for i in range(np.size(waypointsBF,0)-1):
        path_length_BreadthFirst += np.linalg.norm(waypointsBF[i+1] - waypointsBF[i])
    #print("Coverage Path length (Breadth First): ", path_length3/1000, " km")

    waypointsSC = np.array(ShortTree.generate_path())
    path_length_ShortCut = 0
    for i in range(np.size(waypointsSC,0)-1):
        path_length_ShortCut += np.linalg.norm(waypointsSC[i+1] - waypointsSC[i])
    #print("Coverage Path length (Shortcut): ", path_length/1000, " km")

    waypoints = []

    for i,node in enumerate(DFTreeHilbert):
        waypoints.append([node.x, node.y, node.Altitude])
    waypoints = np.array(waypoints)


    path_length_Hilbert = 0
    for i in range(np.size(waypoints,0)-1):
        path_length_Hilbert += np.linalg.norm(waypoints[i+1] - waypoints[i])
    #print("Coverage Path length Hilbert: ", path_length2/1000, " km")

    data_Hilbert.append(path_length_Hilbert/path_length_LawMower)
    data_Shortcut.append(path_length_ShortCut/path_length_LawMower)
    data_DepthFirst.append(path_length_DeepFirst/path_length_LawMower)
    data_BreadthFirst.append(path_length_BreadthFirst/path_length_LawMower)

    #print("Coverage Path length (Hilbert): ", path_length_Hilbert/path_length_LawMower)
    #print("Coverage Path length (Shortcut): ", path_length_ShortCut/path_length_LawMower)
    #print("Coverage Path length (Depth First): ", path_length_DeepFirst/path_length_LawMower)
    #print("Coverage Path length (Breadth First): ", path_length_BreadthFirst/path_length_LawMower)
    #print("Coverage Path length (LawMower): ", path_length_LawMower/path_length_LawMower)

#resultados
print("Setup experimental:")
print("p =", np.round(interesting_area * 100,2), "%", "interesting")
print("C =", len(areas_de_interesse), "patches")
print("Area =",2*L*height,"x",2*L*width)
print("GSD",desirableGSD,"cm/px")
print("Resultados:")
print("Coverage Path length (LawMower): ", path_length_LawMower/1000, " km")
print(f"Hilbert: {np.mean(data_Hilbert):.2f} +/- {np.std(data_Hilbert):.2f}")
print(f"ShortCut: {np.mean(data_Shortcut):.2f} +/- {np.std(data_Shortcut):.2f}")
print(f"DepthFirst: {np.mean(data_DepthFirst):.2f} +/- {np.std(data_DepthFirst):.2f}")
print(f"BreadthFirst: {np.mean(data_BreadthFirst):.2f} +/- {np.std(data_BreadthFirst):.2f}")
