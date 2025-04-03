import rasterio
import os
import numpy as np
from rasterio.windows import Window

# Caminho do arquivo TIFF
input_tif = "inch.tif"
output_folder = "tiles"

# Criar pasta de saída
os.makedirs(output_folder, exist_ok=True)

# Tamanho dos blocos (em pixels)
tile_size = 5280  # Ajuste conforme necessário

# Abrir o arquivo TIFF
with rasterio.open(input_tif) as dataset:
    width, height = dataset.width, dataset.height  # Dimensões da imagem
    count = 0  # Contador para nomear os arquivos

    # Percorrer a imagem em blocos
    for i in range(0, width, tile_size):
        for j in range(0, height, tile_size):
            # Garantir que o bloco não ultrapasse os limites da imagem
            w = min(tile_size, width - i)
            h = min(tile_size, height - j)

            # Criar uma janela para ler os dados do bloco
            window = Window(i, j, w, h)
            tile = dataset.read(window=window)

            # Criar metadados para o bloco
            transform = dataset.window_transform(window)
            profile = dataset.profile
            profile.update({
                "height": h,
                "width": w,
                "transform": transform
            })

            # Salvar bloco como TIFF
            tile_path = os.path.join(output_folder, f"tile_{count}.tif")
            with rasterio.open(tile_path, "w", **profile) as dst:
                dst.write(tile)

            count += 1

print(f"Quebra concluída! {count} imagens geradas em '{output_folder}'")
