import os
import cv2
import numpy as np
import re

class Render():
    def __init__(self,sim,texture_path,grid_size_y = 10):
        self.planesDict = {} #dict planes_id -> plane_handles
        self.sim = sim #instância do simulador
        self.parent_handle = sim.getObjectHandle("/Floor/box") #instância do /Floor/box
        self.previous_tiles = set()
        self.active_tiles = {} # Criar um dicionário para armazenar os tiles ativo
        self.arquivos_ordenados = Render.ordenar_arquivos(texture_path) #ordenas corretamente os nomes dos tildes
        self.grid_size_y, self.grid_size_x = self.get_grid_size(grid_size_y) #número de linhas e colunas da ortofotmapa
        
    @staticmethod
    def ordenar_arquivos(texture_path):
        arquivos = os.listdir(texture_path)
        return sorted(arquivos, key=lambda x: int(re.search(r'tile_(\d+)\.tif', x).group(1)))

    def get_grid_size(self,grid_size_y):
        num_tiles = len(self.arquivos_ordenados)
        return grid_size_y, int(np.ceil(num_tiles / grid_size_y))  # Assume 10 linhas e calcula as colunas dinamicamente

    @staticmethod
    def is_image_empty(image_path):
        img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            return True
        return np.all(img == 0)

    def create_planes(self,texture_path):
        x = 0
        y = 0
        plane_handles = []
        planes_id = []

        #ler lista de arquivos no diretório
        for i, file in enumerate(self.arquivos_ordenados, start=0):
            print(i," ",file)

            if not os.path.exists(texture_path+ '/' + file):
                raise FileNotFoundError(f"Erro: Arquivo de textura não encontrado! Verifique o caminho: {texture_path}")
            
            if Render.is_image_empty(texture_path+ '/' + file):
                print(f"A imagem {texture_path+ '/' + file} está vazia.")
            else:
                print(f"A imagem {texture_path+ '/' + file} NÃO está vazia.")

                texture_path2 = os.path.join(texture_path, file)
                texture_handle = self.sim.createTexture(texture_path2,0,[0.1,0.1],None, None, 0, [2048,2048])[0]
                plane_handle = self.sim.createPrimitiveShape(1,  [12.0, 12.0, 0.002])
                plane_handles.append(plane_handle)
                planes_id.append(int(re.search(r'\d+', file).group()))
                # Definir o plano como filho de "/Floor/box"
                self.sim.setObjectParent(plane_handle, self.parent_handle, True)
                # Definir posição do objeto
                self.sim.setObjectPosition(plane_handle, -1, [6 - 12*x, -6 + 12*y, 0.002])
                self.sim.setObjectOrientation(plane_handle, -1, [0, 0, np.pi]) 

            y += 1
            if(y == 10):
                y = 0
                x += 1
        self.planesDict = dict(zip(planes_id, plane_handles))
    
    def update_scene(self,robot_x,robot_y,texture_path):
        #self.sim.pauseSimulation()
        # Identificar os tiles que precisam ser renderizados
        current_tiles = self.get_tiles_in_radius(robot_x, robot_y)
        print(f"Tiles atuais: {current_tiles}")
        # Adicionar novos tiles
        for i in current_tiles - self.previous_tiles:
            self.sim.pauseSimulation()
            file = self.arquivos_ordenados[i]
            texture_file = os.path.join(texture_path, file)
            if os.path.exists(texture_file) and not Render.is_image_empty(texture_file):
                
                tile_index_x = i // 10
                tile_index_y = i % 10
                tile_x = -tile_index_x * 12 + 6
                tile_y = tile_index_y * 12 - 6
                plane_handle = self.planesDict[i]
                
                ID, resolucao = self.sim.getTextureId("tile_" + str(i))
                self.sim.setShapeTexture(plane_handle,ID,self.sim.texturemap_plane,0,[12.0,12.0])
                self.active_tiles[i] = plane_handle
            self.sim.startSimulation()
        
        # Remover tiles que saíram do alcance
        for i in self.previous_tiles - current_tiles:
            self.sim.pauseSimulation()
            if i in self.active_tiles:
                plane_handle = self.planesDict[i]
                self.sim.setShapeTexture(plane_handle,-1,self.sim.texturemap_plane,0,[0.1,0.1])
                del self.active_tiles[i]
        
        self.sim.startSimulation()

        self.previous_tiles = current_tiles

    # Função para calcular os tiles dentro do alcance do drone
    def get_tiles_in_radius(self,robot_x, robot_y, tile_size=12):
        nearby_tiles = set()
        for dx in range(-1, 2):  # Considera tiles adjacentes dentro do raio
            for dy in range(-1, 2):
                tile_x = int(-(robot_x//tile_size)*tile_size + 6 + dx * tile_size)
                tile_y = int((robot_y//tile_size)*tile_size + 6 + dy * tile_size)
                tile_index_x = (tile_x - 6) // tile_size
                tile_index_y = (tile_y + 6) // tile_size
                if 0 <= tile_index_x < self.grid_size_x and 0 <= tile_index_y < self.grid_size_y:
                    tile_index = tile_index_x * self.grid_size_y + tile_index_y
                    if 0 <= tile_index < len(self.arquivos_ordenados):
                        nearby_tiles.add(tile_index)
        return nearby_tiles