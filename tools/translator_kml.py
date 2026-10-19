import os

# Dicionário de tradução (Português -> Inglês)
translations = {
    # Títulos e descrições gerais
    "Missao ": "Mission ",
    "raio de chegada": "arrival radius",
    "corredor arm": "armed corridor",
    "modo de curva arc": "arc turn mode",
    
    # Nomes de pastas
    "<name>Pontos</name>": "<name>Points</name>",
    "<name>Corredor e reta guia</name>": "<name>Corridor and guide line</name>",
    "<name>Missao</name>": "<name>Mission</name>",
    
    # Pontos e Linhas
    "A - Retorno": "A - Return",
    "Ponto de retorno": "Return point",
    "B - Inicio": "B - Start",
    "Inicio da reta guia": "Guide line start",
    "Linha sul": "South line",
    "Linha norte": "North line",
    "Reta guia": "Guide line",
    
    # Elementos de Rota e GPS
    "Rota do barco (posicao filtrada)": "Boat route (filtered position)",
    "Posicao filtrada pelo Kalman": "Kalman filtered position",
    "GPS cru": "Raw GPS",
    "Fixes do GPS sem filtro": "Unfiltered GPS fixes",
    "Fim da rota": "End of route",
    "Ultima posicao registrada": "Last recorded position",
    
    # Eventos e Status
    "Inicio (g)": "Start (g)",
    "MISSAO iniciada:": "MISSION started:",
    "do ponto de retorno": "from the return point",
    "(corredor armado)": "(armed corridor)",
    "fora do corredor: a checagem arma quando ele entrar 10 m": "outside the corridor: check arms when entering 10 m",
    "Chegada t=": "Arrival t=",
    "CHEGOU ao ponto de retorno:": "ARRIVED at return point:",
    "raio": "radius"
}

def translate_kml_files(input_folder=".", output_folder="translated_kmls"):
    # Cria a pasta de saída se não existir
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Varre todos os arquivos na pasta
    for filename in os.listdir(input_folder):
        if filename.endswith(".kml"):
            filepath = os.path.join(input_folder, filename)
            
            # Lê o conteúdo original do KML
            with open(filepath, 'r', encoding='utf-8') as file:
                content = file.read()
            
            # Aplica as traduções
            for pt_text, en_text in translations.items():
                content = content.replace(pt_text, en_text)
            
            # Salva o novo arquivo na pasta de saída
            output_filepath = os.path.join(output_folder, filename)
            with open(output_filepath, 'w', encoding='utf-8') as file:
                file.write(content)
                
            print(f"Traduzido com sucesso: {filename} -> Salvo em '{output_folder}/'")

if __name__ == "__main__":
    translate_kml_files()