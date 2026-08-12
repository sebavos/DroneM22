# Dron M22 (LYHFPV) + NVIDIA Jetson Nano — Entrega y continuación

Streaming de video en vivo del dron Wi-Fi M22 hacia un navegador, corriendo en una
Jetson Nano. **El video funciona.** Este documento explica cómo operarlo, cómo está
resuelto el formato, y qué queda por hacer.

Repo original: https://github.com/Javiermoralesubo/Camara-drone-hd
Repo actual: https://github.com/sebavos/DroneM22

> **Antes de publicar esto en cualquier lado:** las credenciales viven en
> `ACCESO-PRIVADO.txt`, excluido por `.gitignore`. Verificá que siga excluido
> antes de hacer push.

---

## 1. Estado actual

| Qué | Estado |
|---|---|
| Video en vivo 1280x720 | funcionando |
| Framerate | ~7 fps (era 3.4; ver §5.3) |
| Pérdida de paquetes | 0–3% según señal |
| CPU en la Jetson | ~7% |
| Arranque automático | sí, vía systemd |
| Reconexión al dron tras apagado | automática (autoconnect + watchdog) |
| Tabla de cuantización | **estimada**, ver §5 |

## 2. Acceso

- **Jetson:** `192.168.14.8` (eth0, cable) — usuario `uavlab1`
- **Contraseña:** en `ACCESO-PRIVADO.txt`, que **no** va al repositorio (está en `.gitignore`)
- **Interfaz web:** http://192.168.14.8:5000
- **Telemetría JSON:** http://192.168.14.8:5000/stats

La IP la da DHCP y **ya cambió una vez** (el README original decía `.7`). Si no
responde, buscá la Jetson en la red antes de suponer que está apagada.

Cargá tu propia llave SSH: `ssh-copy-id uavlab1@192.168.14.8`

### Red

- `eth0` → LAN del laboratorio, `192.168.14.8`
- `wlan0` → AP del dron `LYHFPV_M22 2B967C` (red abierta), toma `192.168.169.2`
- El dron es `192.168.169.1`, puerto de comandos UDP `8800`, y devuelve el video al `8804`

## 3. Operación

### Doble clic en el escritorio de la Jetson

En el escritorio hay un icono **«Cámara Dron M22»**. Doble clic y abre el video.
También está en el menú de aplicaciones.

El icono no abre el navegador a ciegas: antes levanta el servicio si está caído,
reconecta `wlan0` al dron si se fue a otra red, y espera a que estén llegando
cuadros de verdad. Si tras 25 s no hay video, avisa con un cartel que lo más
probable es que el dron esté apagado, y abre la página igual mostrando el HUD.

Para diagnosticar desde una terminal, sin abrir el navegador:

```bash
~/abrir_camara.sh --diagnostico
```

Para que el icono pueda levantar el servicio sin pedir contraseña hay un
`/etc/sudoers.d/drone-stream` que autoriza a `uavlab1` **solo** estos comandos:
`systemctl start|restart|stop drone-stream` y el script del watchdog. Cualquier
otro `sudo` sigue pidiendo contraseña.

### Desde otra computadora de la red

El servicio arranca solo al bootear y se reinicia si se cae.

```bash
sudo systemctl status  drone-stream     # ver estado
sudo systemctl restart drone-stream     # reiniciar
sudo systemctl stop    drone-stream     # detener
tail -f ~/drone_stream.log              # ver el log
```

### El dron se apaga solo por inactividad

Es la causa principal de que "se pierda la conexión". Cuando el dron se duerme,
desaparece su AP y `wlan0` se queda sin enlace (o se va a otra red conocida).

Hay **dos mecanismos** que lo recuperan cuando volvés a prender el dron, sin que
nadie tenga que hacer nada:

1. **Autoconnect de NetworkManager** — el perfil del dron tiene
   `autoconnect-priority 100`, así que si `wlan0` quedó desconectado, NM vuelve solo.
2. **Watchdog `drone-wifi-watchdog.timer`** — cada 30 s: si `wlan0` no está en el AP
   del dron *y* el AP está visible, lo reconecta. Cubre el caso que el autoconnect
   no cubre: NM evalúa la prioridad solo al momento de conectar, **no** abandona una
   conexión que ya funciona porque aparezca una de mayor prioridad. Sin el watchdog,
   si `wlan0` se fue al Wi-Fi del campus, ahí se quedaba aunque el dron volviera.

```bash
sudo systemctl list-timers drone-wifi-watchdog.timer   # cuándo corre
sudo journalctl -t drone-wifi -n 20                    # qué hizo
```

Reconexión manual, si hace falta:

```bash
iwgetid -r                                  # ¿a qué AP está asociado wlan0?
nmcli connection up "LYHFPV_M22 2B967C"     # reconectar al dron
```

Mientras no hay enlace, el servicio sigue corriendo y la web muestra el HUD de
telemetría con *"Esperando respuesta del dron"*. No hay que reiniciar nada.

### Rotación del log

`~/drone_stream.log` rota vía `/etc/logrotate.d/drone-stream` (diario, máximo 5 MB,
5 archivos, comprimidos). Usa `copytruncate` porque el servicio mantiene el archivo
abierto en modo append.

Casi todo el volumen del log son accesos a `/stats`: la página web lo consulta una
vez por segundo, ~86.000 líneas por día si queda un navegador abierto.

### Ajustar contraste / color

```bash
sudo systemctl stop drone-stream
DRONE_JPEG_Q=30 python3 -u ~/drone_stream.py     # probar; valores 20,30,40,50,60
```

Más bajo = más contraste y saturación. El default es 40.

## 4. El formato del video (lo importante)

**El dron no envía JPEG.** Envía únicamente el *scan data* (datos entropy-coded),
**sin cabecera**. Por eso buscar el marcador `FF D8` nunca encuentra nada: no existe
en el stream. Ese era el bloqueo del proyecto.

Cómo se comprobó: en JPEG el byte `0xFF` es el escape de marcador, así que dentro del
scan todo `0xFF` literal lleva un `0x00` detrás (*byte stuffing*). Sobre los 6 cuadros
capturados hay **1071 bytes `0xFF` y los 1071 están seguidos de `0x00`**. Corré
`verificar_formato.py` para verlo.

La solución es fabricar la cabecera que falta:

```
cuadro reproducible = CABECERA + scan + FF D9
```

Parámetros determinados experimentalmente:

- Submuestreo **4:4:4** — con 4:2:0 o 4:2:2 salen rayas de colores
- Tablas Huffman **estándar (Annex K)** — en PIL: `optimize=False`
- Cuantización estándar a **q≈40** — estimada, ver §5

La cabecera no se tipea a mano: se hace que una librería codifique una imagen
tapadera y se corta su salida al final del segmento SOS. Ver `decodificar.py`.

### Estructura del paquete UDP (1080 bytes)

| Offset | Contenido |
|---|---|
| 0..31 | header de transporte (bytes 2-3 = `1080` LE, largo del paquete) |
| 32 | índice de fragmento — **`0` marca inicio de cuadro nuevo** |
| 36 | cantidad total de fragmentos del cuadro |
| 40..43 | uint32 LE — tamaño total del scan |
| 44..45 | uint16 LE — ancho (`1280`) |
| 46..47 | uint16 LE — alto (`720`) |
| 54..55 | `AA AA` — fin de cabecera |
| 56..1079 | payload de video (1024 bytes) |

Los campos de los offsets 36 y 40 sirven para **validar el reensamblado**: si la
cantidad de fragmentos recibidos y el tamaño ensamblado no coinciden con lo
declarado, se perdió un paquete y el cuadro se descarta.

### Comandos UDP hacia `192.168.169.1:8800`

| Comando | Función |
|---|---|
| `EF 00 01 00` | enciende el transmisor. Solo al arrancar, o para re-despertar la cámara si dejó de llegar video |
| `EF 00 04 00` | keepalive. **Cada uno provoca una ráfaga de cuadros**, así que su frecuencia controla el framerate. Se manda cada 0.1 s |

Son los **únicos dos conocidos**. Ver §5.

## 5. Qué falta — hoja de ruta

Ordenado por relación valor/dificultad.

**1. Tabla de cuantización real.**
`q=40` es una estimación: es el punto donde el recorte de blancos cae a ~0. La
estructura de la imagen es exacta, pero contraste y saturación son aproximados.
La tabla verdadera está en la app del dron — sacarla del APK, o hacer que la app
guarde una foto y leerle la cabecera. Bien acotado y verificable.

**2. Explorar el espacio de comandos (EN PROGRESO).**
Solo se conocen `EF 00 01 00` y `EF 00 04 00`. Todo el rango `EF 00 xx 00` está sin
explorar: ahí pueden estar resolución, framerate, foto, grabación, gimbal. Es la
línea con más potencial de descubrimiento.
*Actualización:* Actualmente estamos usando **Frida** (mediante el script `drone_debug.js`) para interceptar la aplicación oficial de Android y descubrir qué comandos UDP envía realmente para mejorar los FPS y controlar la cámara.

**3. Framerate — parcialmente resuelto, queda margen.**
El dron **emite una ráfaga de cuadros por cada keepalive que recibe**. Con el
keepalive original de 1 s el video llegaba a tirones: ráfagas cortas separadas por
pausas de casi 1 s (histograma bimodal, nada entre 150 y 500 ms). Bajando el
keepalive a **0.1 s** el framerate pasó de 3.4 a ~7 fps y el jitter de 379 a 147 ms.
Ver la tabla de mediciones en `drone_stream.py`, constante `HB_INTERVAL`.

Lo que queda: la **mediana** del intervalo entre cuadros es de ~60 ms, o sea que el
dron puede entregar a ~16 fps. Pero todavía aparecen pausas de 250–600 ms que bajan
el promedio a 7.

**Tres causas ya descartadas con mediciones** — no vale la pena volver a probarlas:

| Hipótesis | Cómo se descartó |
|---|---|
| Desborde del buffer UDP de recepción | Contadores del kernel en `/proc/net/snmp`: `RcvbufErrors` e `InErrors` con delta **0** sobre 6817 datagramas en 20 s. El kernel no descarta nada |
| Capacidad del enlace Wi-Fi | Señal **-18 dBm**, calidad 70/70, `tx failed: 0`, cero errores de interfaz. Se usan 2.9 Mbps de 36 Mbit/s: **8% de utilización** |
| Ahorro de energía del Wi-Fi | `iw dev wlan0 set power_save off` → 6.43 fps vs 6.44, jitter 152 vs 153 ms. **Sin efecto** |

Dato adicional: hay ~5% de cuadros incompletos **con cero descartes del kernel**, así
que esos paquetes se pierden en el aire, no en la máquina.

Por eliminación, la pausa la genera el **firmware del dron**. La única palanca que
queda es el punto 2: encontrar un comando que fije framerate o bitrate, en lugar de
este esquema de ráfaga-por-keepalive que es un hack. Y la vía más directa para
encontrarlo es **capturar el tráfico UDP de la app del fabricante**: en una sola
sesión se ve cada cuánto manda el keepalive, qué otros comandos usa, y qué framerate
consigue (o sea, el techo real del hardware).

**4. Cuadros parciales.**
Hoy se descarta el cuadro si falta un fragmento. Se podría rellenar el hueco y
decodificar igual, mostrando la parte buena.

**5. Grabación a disco.** Hoy no hay forma de guardar el video, solo verlo en vivo.

**6. Servidor de producción.**
Flask corre con su servidor de desarrollo. Para uso real, `gunicorn` o `waitress`.

## 6. Archivos

| Archivo | Qué es |
|---|---|
| `fixtures/frame_00..05.bin` | 6 cuadros crudos capturados del dron |
| `fixtures/frame_00..05.meta` | fragmentos y tamaño de cada cuadro |
| `verificar_formato.py` | comprueba el byte stuffing — la prueba del formato |
| `decodificar.py` | convierte un `.bin` en PNG; la lógica clave, comentada |
| `capture.py` | captura cruda desde el dron (herramienta de diagnóstico) |
| `ACCESO-PRIVADO.txt` | credenciales — **excluido de git** |
| `.gitignore` | mantiene las credenciales fuera del repositorio |
| `drone_stream.py` | el servidor completo que corre en la Jetson |
| `drone_stream.py.orig` | versión original de Javier, antes del arreglo |
| `drone-stream.service` | unit de systemd del servidor |
| `drone-wifi-watchdog.sh` + `.service` + `.timer` | watchdog de reconexión al dron |
| `drone-stream` (logrotate) | rotación del log |
| `abrir_camara.sh` | lo que ejecuta el icono del escritorio |
| `Camara-Dron-M22.desktop` | el icono en sí |
| `drone-stream.sudoers` | permisos acotados para el icono |

### Los fixtures son lo más útil de esta carpeta

Con esos 6 archivos se trabaja el decodificador **sin dron y sin Jetson**, en
cualquier notebook con Python y PIL (`pip install pillow`; en la Jetson ya está
instalado como `python3-pil`):

```bash
python3 verificar_formato.py                          # ver la prueba del formato
python3 decodificar.py fixtures/frame_00.bin out.png  # reconstruir una imagen
```

Si el dron se descarga o se rompe, el proyecto sigue.
