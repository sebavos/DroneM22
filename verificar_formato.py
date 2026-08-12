"""
Comprobacion de que los cuadros del dron son 'scan data' JPEG puro.

    python3 verificar_formato.py

Regla que se verifica: dentro del scan de un JPEG, el byte 0xFF es el escape de
marcador. Si el bitstream comprimido produce un 0xFF literal, el codificador
tiene que insertar un 0x00 detras (byte stuffing) para que no se confunda con un
marcador. Entonces, en scan valido, TODO 0xFF va seguido de 0x00 (o de un restart
marker D0..D7, que aca no aparecen: eso indica DRI=0).

Por que este test y no buscar el marcador FF D8: en datos de alta entropia,
buscar marcadores da decenas de falsos positivos. Un barrido sobre un solo cuadro
encontro 10 "SOI", 35 "EOI" y 62 "SOS", todos ruido. La regla del stuffing, en
cambio, es una invariante que se puede medir sobre cientos de casos.
"""

import glob
import os
import collections

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")

total_ff = 0
total_stuffed = 0

print("archivo         bytes    0xFF   seguidos de 0x00   otros")
print("-" * 62)

for f in sorted(glob.glob(os.path.join(CARPETA, "*.bin"))):
    d = open(f, "rb").read()
    posiciones = [i for i, b in enumerate(d) if b == 0xFF and i + 1 < len(d)]
    siguientes = collections.Counter(d[i + 1] for i in posiciones)
    stuffed = siguientes.get(0x00, 0)
    otros = [(hex(k), v) for k, v in siguientes.most_common() if k != 0x00]

    total_ff += len(posiciones)
    total_stuffed += stuffed

    print("%-14s %7d %6d %12d %5s   %s"
          % (os.path.basename(f), len(d), len(posiciones), stuffed,
             "OK" if stuffed == len(posiciones) else "FALLA",
             otros if otros else ""))

print("-" * 62)
if total_ff == 0:
    print("No se encontraron bytes 0xFF. Hay fixtures en %s ?" % CARPETA)
else:
    pct = 100.0 * total_stuffed / total_ff
    print("TOTAL: %d bytes 0xFF, %d seguidos de 0x00 (%.1f%%)" % (total_ff, total_stuffed, pct))
    if total_stuffed == total_ff:
        print()
        print("CONCLUSION: cumple el byte stuffing al 100%.")
        print("Los datos son scan JPEG sin cabecera. Falta SOI+DQT+DHT+SOF0+SOS.")
        print("Ver decodificar.py para fabricarla.")
    else:
        print()
        print("CONCLUSION: NO cumple la regla. Revisar el reensamblado de fragmentos")
        print("(orden de los chunks, offset del payload) antes de dudar del formato.")
