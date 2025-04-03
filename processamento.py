from PIL import Image
import os

# Configurações
input_folder = "inch 12 m"  # Pasta de origem das imagens
output_folder = "inch 12 m 2"  # Pasta de destino
new_size = (3956, 3956)  # Novo tamanho da imagem (largura, altura)

# Criar a pasta de destino se não existir
os.makedirs(output_folder, exist_ok=True)

# Processar todas as imagens na pasta de origem
for filename in os.listdir(input_folder):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):  
        img_path = os.path.join(input_folder, filename)
        img = Image.open(img_path)  # Abre a imagem
        img_resized = img.resize(new_size, Image.LANCZOS)  # CORRETO
        output_path = os.path.join(output_folder, filename)
        img_resized.save(output_path)  # Salva a imagem redimensionada
        print(f"Imagem {filename} salva em {output_folder}")
