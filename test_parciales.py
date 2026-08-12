"""
Test de la logica de recuperación de cuadros parciales.

Simula la pérdida de fragmentos sobre los fixtures existentes y verifica que
el JPEG resultante es decodificable (con artefactos localizados, pero sin crash).

    python3 test_parciales.py
"""

import io
import os
import sys
import glob

# Reutilizar la lógica de cabecera del decodificador
from decodificar import cabecera_jpeg

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
CHUNK_SIZE = 1024  # payload por fragmento
PARTIAL_THRESHOLD = 0.80


def simular_fragmentacion(scan_data):
    """Divide el scan en chunks de 1024 bytes (como hace el dron)."""
    chunks = {}
    n = 0
    for offset in range(0, len(scan_data), CHUNK_SIZE):
        chunks[n] = scan_data[offset:offset + CHUNK_SIZE]
        n += 1
    return chunks, n


def reconstruir_con_huecos(chunks, total_chunks, indices_a_borrar):
    """Simula pérdida de paquetes y reconstrucción con relleno de ceros."""
    chunks_parcial = dict(chunks)
    for i in indices_a_borrar:
        if i in chunks_parcial:
            del chunks_parcial[i]

    ratio = len(chunks_parcial) / total_chunks
    if ratio < PARTIAL_THRESHOLD:
        return None, ratio, "debajo del umbral"

    # Rellenar huecos con ceros
    for i in range(total_chunks):
        if i not in chunks_parcial:
            chunks_parcial[i] = b"\x00" * CHUNK_SIZE

    scan = b"".join(chunks_parcial[i] for i in range(total_chunks))
    header = cabecera_jpeg(1280, 720, 40)
    jpg = header + scan + b"\xff\xd9"
    return jpg, ratio, "recuperado"


def verificar_jpeg(jpg_data):
    """Intenta decodificar el JPEG con PIL."""
    try:
        from PIL import Image
        img = Image.open(io.BytesIO(jpg_data))
        img.load()
        return True, "%s %s" % (img.size, img.mode)
    except Exception as e:
        return False, str(e)


def main():
    bins = sorted(glob.glob(os.path.join(FIXTURES, "*.bin")))
    if not bins:
        print("No se encontraron fixtures en %s" % FIXTURES)
        return 1

    print("=" * 70)
    print(" Test de recuperación de cuadros parciales")
    print(" Fixtures: %d archivos" % len(bins))
    print(" Umbral: %.0f%%" % (PARTIAL_THRESHOLD * 100))
    print("=" * 70)

    escenarios = [
        ("0 perdidos (completo)", []),
        ("1 perdido (medio)", "mid1"),
        ("2 perdidos (distribuidos)", "dist2"),
        ("5 perdidos (distribuidos)", "dist5"),
        ("15 perdidos (~22%)", "dist15"),
    ]

    total_tests = 0
    total_ok = 0

    for bin_path in bins:
        nombre = os.path.basename(bin_path)
        scan = open(bin_path, "rb").read()
        chunks, n_chunks = simular_fragmentacion(scan)

        print("\n--- %s (%d bytes, %d chunks) ---" % (nombre, len(scan), n_chunks))

        for desc, perdidos in escenarios:
            if perdidos == []:
                indices = []
            elif perdidos == "mid1":
                indices = [n_chunks // 2]
            elif perdidos == "dist2":
                indices = [n_chunks // 3, 2 * n_chunks // 3]
            elif perdidos == "dist5":
                step = n_chunks // 6
                indices = [step * i for i in range(1, 6)]
            elif perdidos == "dist15":
                step = max(1, n_chunks // 16)
                indices = [step * i for i in range(1, 16) if step * i < n_chunks]
            else:
                indices = perdidos

            jpg, ratio, estado = reconstruir_con_huecos(chunks, n_chunks, indices)
            total_tests += 1

            if jpg is None:
                print("  %-35s  ratio=%.0f%%  DESCARTADO (bajo umbral)" % (desc, ratio * 100))
                if ratio < PARTIAL_THRESHOLD:
                    total_ok += 1  # Correcto: se descartó porque debía
                continue

            decodifica, info = verificar_jpeg(jpg)
            status = "OK" if decodifica else "FALLO"
            if decodifica:
                total_ok += 1
            print("  %-35s  ratio=%.0f%%  %s  jpeg=%d bytes  %s"
                  % (desc, ratio * 100, status, len(jpg), info))

    print("\n" + "=" * 70)
    print(" Resultado: %d/%d tests pasaron" % (total_ok, total_tests))
    print("=" * 70)
    return 0 if total_ok == total_tests else 1


if __name__ == "__main__":
    sys.exit(main())
