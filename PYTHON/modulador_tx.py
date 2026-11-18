import os
import matplotlib
matplotlib.use('Agg')  # Motor no interactivo para guardar imágenes silenciosamente
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile
import CONDICION_IMAGEN as k

plt.rcParams.update({
    'font.size': 12,
    'font.family': 'serif',
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'figure.dpi': 100
})

DIRECTORIO_SCRIPT = os.path.dirname(os.path.abspath(__file__))
CARPETA_IMAGENES = os.path.expanduser("~/IMAGENES_PROYECTO_SA828")  
CARPETA_SALIDA = os.path.join(DIRECTORIO_SCRIPT, "Audios")

FS = 48000                  
F_PORTADORA = 1800          
MUESTRAS_POR_PIXEL = 20     

MUESTRAS_SYNC = 400         
MUESTRAS_PORCHE_TRAS = 240  
MUESTRAS_PORCHE_DEL = 240   

SILENCIO_INICIAL_S = 0.2
PORTADORA_PURA_S = 15.0     
MUESTRAS_GUARDA_CABECERA = 480  
SILENCIO_FINAL_S = 0.2

NIVEL_PICO = 0.9            


def crear_linea_APT(matriz_base, fs, nombre_archivo, muestras_por_pixel=MUESTRAS_POR_PIXEL):
    try:
        f_portadora = F_PORTADORA
        filas = matriz_base.shape[0]
        columnas = matriz_base.shape[1]

        sync = np.ones(MUESTRAS_SYNC) * 255
        porche_tras = np.zeros(MUESTRAS_PORCHE_TRAS)
        porche_del = np.zeros(MUESTRAS_PORCHE_DEL)

        video = np.repeat(matriz_base.astype(np.float64), muestras_por_pixel, axis=1)

        matriz_sync = np.tile(sync, (filas, 1))                
        matriz_tras = np.tile(porche_tras, (filas, 1))
        matriz_del = np.tile(porche_del, (filas, 1))
        matriz_final = np.hstack((matriz_sync, matriz_tras, video, matriz_del))
        muestras_linea = matriz_final.shape[1]

        silencio_ini = np.zeros(int(SILENCIO_INICIAL_S * fs))
        cabecera = np.ones(int(PORTADORA_PURA_S * fs)) * 255
        guarda = np.zeros(MUESTRAS_GUARDA_CABECERA)
        silencio_fin = np.zeros(int(SILENCIO_FINAL_S * fs))
        buffer = np.concatenate((silencio_ini, cabecera, guarda, matriz_final.flatten(), silencio_fin))

        A = buffer / 255.0
        t = np.arange(len(buffer)) / fs
        senal_salida = NIVEL_PICO * A * np.sin(2 * np.pi * f_portadora * t)
        audio_16bits = np.int16(np.clip(np.round(senal_salida * 32767), -32768, 32767))

        os.makedirs(os.path.dirname(nombre_archivo), exist_ok=True)
        wavfile.write(nombre_archivo, fs, audio_16bits)
        print(f"Archivo {nombre_archivo} generado con éxito")
        print(f"{filas} x {columnas} pixeles, {muestras_linea} muestras por línea ({1000 * muestras_linea / fs:.0f} ms), "
              f"duración total {len(buffer) / fs:.1f} s")

        fig, ax = plt.subplots(figsize=(8, 4))
        inicio = len(silencio_ini) + len(cabecera) + len(guarda)
        muestras_zoom = 1200
        ax.plot(t[inicio:inicio + muestras_zoom], senal_salida[inicio:inicio + muestras_zoom])
        ax.set_title("Señal modulada, inicio de línea")
        ax.set_xlabel("Tiempo [s]")
        ax.set_ylabel("Amplitud")
        ax.grid(True, linestyle='--', alpha=0.6)
        plt.tight_layout()

        os.makedirs(CARPETA_SALIDA, exist_ok=True)
        ruta_png = os.path.join(CARPETA_SALIDA, f"{os.path.basename(nombre_archivo)[:-4]}.png")
        plt.savefig(ruta_png)  
        plt.close(fig)         

        print(f"Gráfico guardado en: {ruta_png}\n")
        return nombre_archivo
    except Exception as e:
        print(f"[¡] Error: {e}")


if __name__ == '__main__':
    lista_direcciones = [f for f in os.listdir(CARPETA_IMAGENES) if f.lower().endswith(k.EXTENSIONES)]
    print("Ingrese el numero de la imagen que necesita enviar:")
    for i, f in enumerate(lista_direcciones):
        print(f"{i}: {f}")
    
    # Se agrega explícitamente la opción 15 para procesar todo
    print("15: Generar audios para TODAS las imágenes de la carpeta")

    try:
        op = int(input("\nIngrese el número de imagen que se quiere enviar: \n"))
        
        if op == 15:
            print("\nIniciando procesamiento por lotes...")
            for f in lista_direcciones:
                print(f"--- Procesando {f} ---")
                matriz_base = k.cargar_normalizar(os.path.join(CARPETA_IMAGENES, f))
                nombre_archivo = os.path.join(CARPETA_SALIDA, f"{f.split('.')[0]}_modulada.wav")
                crear_linea_APT(matriz_base, FS, nombre_archivo)
            print("¡Todas las imágenes fueron convertidas a audio exitosamente!")
            
        else:
            print(f"\nProcesando {lista_direcciones[op]}...")
            matriz_base = k.cargar_normalizar(os.path.join(CARPETA_IMAGENES, lista_direcciones[op]))
            print(f"Formato matriz: {matriz_base.shape}")
            nombre_archivo = os.path.join(CARPETA_SALIDA, f"{lista_direcciones[op].split('.')[0]}_modulada.wav")
            crear_linea_APT(matriz_base, FS, nombre_archivo)
            
    except IndexError:
        print("Error: El número ingresado no corresponde a ninguna imagen en la lista ni a la opción 15.")
    except ValueError:
        print("Error: Debes ingresar un número válido.")
    except FileNotFoundError as e:
        print(f"Error: {e}")