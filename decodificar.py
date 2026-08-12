"""
Decodificador minimo de un cuadro del dron M22 (LYHFPV).
No necesita dron ni Jetson: trabaja sobre los .bin de fixtures/.

    python3 decodificar.py fixtures/frame_00.bin salida.png

EL PUNTO CLAVE
--------------
El dron NO envia un JPEG. Envia solo el 'scan data' (los datos entropy-coded),
SIN cabecera. Por eso buscar el marcador de inicio FF D8 nunca encuentra nada:
no existe en el stream.

Como se comprobo: en JPEG, el byte 0xFF es el escape de marcador. Si el bitstream
comprimido produce un 0xFF literal, el codificador DEBE meterle un 0x00 detras
para que ningun decodificador lo confunda con un marcador (esto se llama byte
stuffing). Sobre los 6 cuadros capturados hay 1071 bytes 0xFF y los 1071 estan
seguidos de 0x00. Si el byte siguiente fuera arbitrario la probabilidad seria
(1/256)^1071. Ver verificar_formato.py.

La solucion es fabricar la cabecera que falta y pegarla adelante:

    cuadro reproducible = CABECERA + scan + FF D9

Los parametros de la cabecera se determinaron probando:
  - submuestreo 4:4:4   -> 4:2:0 y 4:2:2 dan rayas de colores, 4:4:4 da la imagen
  - Huffman estandar    -> Annex K (por eso optimize=False)
  - cuantizacion q~40   -> ESTIMACION, ver la nota al final
"""

import sys
import io
from PIL import Image


def cabecera_jpeg(ancho=1280, alto=720, calidad=40):
    """Genera la cabecera JPEG que el dron no manda.

    En vez de tipear a mano las tablas DQT y DHT (64 valores cada una, muy facil
    de equivocarse), dejamos que PIL codifique una imagen tapadera y cortamos su
    salida justo al final del segmento SOS. Todo lo que viene antes del SOS es
    exactamente la cabecera que necesitamos.
    """
    buf = io.BytesIO()
    Image.new("RGB", (ancho, alto)).save(
        buf, "JPEG",
        quality=calidad,
        subsampling=0,      # 0 = 4:4:4 (sin submuestreo de color)
        optimize=False,     # fuerza tablas Huffman estandar, no optimizadas
        progressive=False,
    )
    t = buf.getvalue()

    p = t.find(b"\xff\xda")               # SOS: Start Of Scan
    largo = (t[p + 2] << 8) | t[p + 3]    # los 2 bytes siguientes son el largo del segmento
    return t[: p + 2 + largo]


def reconstruir(scan, ancho=1280, alto=720, calidad=40):
    """Convierte el scan crudo del dron en un JPEG completo y valido."""
    return cabecera_jpeg(ancho, alto, calidad) + scan + b"\xff\xd9"   # FF D9 = EOI


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("uso: python3 decodificar.py <archivo.bin> [salida.png] [calidad]")
        return 1

    entrada = sys.argv[1]
    salida = sys.argv[2] if len(sys.argv) > 2 else entrada.rsplit(".", 1)[0] + ".png"
    calidad = int(sys.argv[3]) if len(sys.argv) > 3 else 40

    scan = open(entrada, "rb").read()
    jpg = reconstruir(scan, calidad=calidad)

    img = Image.open(io.BytesIO(jpg))
    img.load()
    img.convert("RGB").save(salida)

    print("scan leido    : %d bytes" % len(scan))
    print("cabecera       : %d bytes" % len(cabecera_jpeg(calidad=calidad)))
    print("JPEG resultante: %d bytes" % len(jpg))
    print("imagen         : %s %s -> %s" % (img.size, img.mode, salida))
    return 0


if __name__ == "__main__":
    sys.exit(main())

# NOTA / TAREA PENDIENTE ------------------------------------------------------
# calidad=40 NO es la tabla de cuantizacion real del dron: es una estimacion,
# el punto donde el recorte de blancos cae a ~0. Consecuencia: la ESTRUCTURA de
# la imagen es exacta, pero el contraste y la saturacion son aproximados.
# Probar valores entre 20 y 60 para ver el efecto.
# Para obtener la tabla verdadera hay que sacarla de la app del dron (del APK, o
# leyendo la cabecera de una foto que la app guarde).
