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


##################################### Setup Experimental #########################################
#random.seed(42)  # For reproducibility
#DJImini3Camera = SensorCamera(9.6,9.6,4032,4032)
DJImini3Camera = SensorCamera()
desirableGSD = 1*0.43 #cm/px
altitude = DJImini3Camera.h_desirable(desirableGSD)/100
width,height = DJImini3Camera.l(altitude) # dimensão horizontal do sensor para a altitude desejada
L = 4
poly = Polygon([(-L*height, -L*width), (L*height, -L*width), (L*height, L*width), (-L*height, L*width)])
angle_deg = 0  # ângulo da varredura em graus (0 = horizontal, 90 = vertical)
waypointsLawMower = lawmowerPath(poly, height, width, angle_deg)

base = Polygon([(-0.1*height*L, -0.1*width*L), (0.1*height*L,-0.1*width*L), (0.1*height*L,0.1*width*L), (-0.1*height*L,0.1*width*L)])

p = 10 # proporção de área de interesse em relação à área total
c = 10 # número de patches de interesse
base = affinity.scale(base,np.sqrt(p/c), np.sqrt(p/c), origin=(0, 0))

################################## Inicio dos Experimentos ###############################################
quantidade = 100
data_Hilbert, data_Shortcut, data_DepthFirst, data_BreadthFirst, = [],[],[],[]
for _ in range(quantidade):
    areas_de_interesse = []
    cont = 0
    while len(areas_de_interesse) < c:
        cont += 1
        if cont > 10000:
            cont = 0
            areas_de_interesse = []
        i = random.uniform(-1 + np.sqrt(p/c)*0.1, 1 - np.sqrt(p/c)*0.1)
        j = random.uniform(-1 + np.sqrt(p/c)*0.1, 1 - np.sqrt(p/c)*0.1)     
        poly_rand = affinity.translate(base, i*L*height, j*L*width)
        if(not any(p.intersects(poly_rand) for p in areas_de_interesse)):
            areas_de_interesse.append(poly_rand)
                
    interesting_area = 0
    for area in areas_de_interesse:
        interesting_area += poly.intersection(base).area
    interesting_area = interesting_area /poly.area
    #print("p =", np.round(interesting_area * 100,2), "%", "interesting")
    #print("C =", len(areas_de_interesse), "patches")

    altitude2 = DJImini3Camera.h_desirable(4*desirableGSD)/100
    width2,height2 = DJImini3Camera.l(altitude2) # dimensão horizontal do sensor para a altitude desejada
    waypointsInitial = lawmowerPath(poly, height2, width2, angle_deg)

    LawnmowerTree = Tree(sensorCamera=DJImini3Camera)
    LawnmowerTree.add_fistLevel_nodes(waypointsLawMower, altitude,width,height)

    BFTree = BreadthFirst(DJImini3Camera,areas_de_interesse)
    BFTree.add_fistLevel_nodes(waypointsInitial, altitude2,width2,height2)

    ShortTree = Shortcut(DJImini3Camera,areas_de_interesse)
    ShortTree.add_fistLevel_nodes(waypointsInitial, altitude2,width2,height2)

    DFTree = DeepFirst(DJImini3Camera,areas_de_interesse)
    DFTree.add_fistLevel_nodes(waypointsInitial, altitude2,width2,height2)

    DFTreeHilbert = HilbertTree(DJImini3Camera,areas_de_interesse)
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
