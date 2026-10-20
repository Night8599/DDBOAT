import os

translations = {
    "Missao ": "Mission ",
    "raio de chegada": "arrival radius",
    "corredor arm": "armed corridor",
    "modo de curva arc": "arc turn mode",
    "<name>Pontos</name>": "<name>Points</name>",
    "<name>Corredor e reta guia</name>": "<name>Corridor and guide line</name>",
    "<name>Missao</name>": "<name>Mission</name>",
    "A - Retorno": "A - Return",
    "Ponto de retorno": "Return point",
    "B - Inicio": "B - Start",
    "Inicio da reta guia": "Guide line start",
    "Linha sul": "South line",
    "Linha norte": "North line",
    "Reta guia": "Guide line",
    "Rota do barco (posicao filtrada)": "Boat route (filtered position)",
    "Posicao filtrada pelo Kalman": "Kalman filtered position",
    "GPS cru": "Raw GPS",
    "Fixes do GPS sem filtro": "Unfiltered GPS fixes",
    "Fim da rota": "End of route",
    "Ultima posicao registrada": "Last recorded position",
    "Inicio (g)": "Start (g)",
    "MISSAO iniciada:": "MISSION started:",
    "do ponto de retorno": "from the return point",
    "(corredor armado)": "(armed corridor)",
    "fora do corredor: a checagem arma quando ele entrar 10 m": "outside the corridor: check arms when entering 10 m",
    "Chegada t=": "Arrival t=",
    "CHEGOU ao ponto de retorno:": "ARRIVED at return point:",
    "raio": "radius"
}

def translate_kml_directory():
    # Obtém o diretório atual do script (project_root/tools/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Sobe um nível para acessar a raiz do projeto (project_root/)
    project_root = os.path.dirname(script_dir)
    
    # Define os caminhos relativos à raiz do projeto
    input_folder = os.path.join(project_root, "data", "boat", "logs")
    output_folder = os.path.join(project_root, "data", "boat", "logs_en")

    if not os.path.exists(input_folder):
        print(f"Erro: A pasta {input_folder} não foi encontrada.")
        return

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in os.listdir(input_folder):
        if filename.endswith(".kml"):
            filepath = os.path.join(input_folder, filename)
            
            with open(filepath, 'r', encoding='utf-8') as file:
                content = file.read()
            
            for pt_text, en_text in translations.items():
                content = content.replace(pt_text, en_text)
            
            output_filepath = os.path.join(output_folder, filename)
            with open(output_filepath, 'w', encoding='utf-8') as file:
                file.write(content)
                
            print(f"Traduzido: {filename} -> {output_folder}")

if __name__ == "__main__":
    translate_kml_directory()