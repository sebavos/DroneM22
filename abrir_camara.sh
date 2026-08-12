#!/bin/bash
# Abre la camara del dron M22 en el navegador.
# Pensado para lanzarse con doble clic desde el icono del escritorio, pero
# tambien sirve desde una terminal para diagnosticar.
#
# Hace tres cosas antes de abrir el navegador:
#   1. levanta el servicio drone-stream si esta caido
#   2. devuelve wlan0 al AP del dron si se fue a otra red
#   3. espera a que realmente esten llegando cuadros
# Si tras la espera no hay video, avisa que lo mas probable es que el dron este
# apagado (se apaga solo por inactividad) y abre igual la web, que muestra el HUD.

#   abrir_camara.sh                -> uso normal (lo que hace el doble clic)
#   abrir_camara.sh --diagnostico  -> hace todo menos abrir el navegador

URL="http://localhost:5000"
SSID="LYHFPV_M22 2B967C"
ESPERA=25          # segundos maximos esperando cuadros

DIAG=0
[ "${1:-}" = "--diagnostico" ] && DIAG=1

export DISPLAY="${DISPLAY:-:0}"

aviso() {
    notify-send -i camera-video "Cámara Dron M22" "$1" 2>/dev/null
    echo "[*] $1"
}

# --- 1. servicio ---------------------------------------------------------
if ! systemctl is-active --quiet drone-stream; then
    aviso "Iniciando el servicio de video..."
    sudo -n /bin/systemctl start drone-stream 2>/dev/null \
        || sudo -n /bin/systemctl restart drone-stream 2>/dev/null \
        || echo "[-] No se pudo iniciar el servicio (revisar sudoers)"
    sleep 3
fi

# --- 2. Wi-Fi del dron ---------------------------------------------------
# Se reintenta un par de veces: durante un reescaneo o una reasociacion, nmcli
# puede no listar la conexion por un instante y no hay que confundir eso con
# estar desconectado.
en_el_dron() {
    for _ in 1 2 3; do
        [ "$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null \
             | grep ':wlan0$' | cut -d: -f1)" = "$SSID" ] && return 0
        sleep 1
    done
    return 1
}

if ! en_el_dron; then
    aviso "Reconectando al Wi-Fi del dron..."
    sudo -n /usr/local/bin/drone-wifi-watchdog.sh 2>/dev/null \
        || nmcli connection up "$SSID" >/dev/null 2>&1
    sleep 3
fi

# --- 3. esperar cuadros reales -------------------------------------------
# No basta con que el puerto responda: el servidor contesta igual mostrando el
# HUD. Se comprueba que 'ultimo_cuadro' sea reciente, o sea que hay video vivo.
hay_video() {
    python3 - "$ESPERA" <<'PY'
import json, sys, time, urllib.request

limite = time.time() + float(sys.argv[1])
while time.time() < limite:
    try:
        with urllib.request.urlopen("http://localhost:5000/stats", timeout=2) as r:
            d = json.load(r)
        if time.time() - d.get("ultimo_cuadro", 0) < 5:
            sys.exit(0)
    except Exception:
        pass
    time.sleep(1)
sys.exit(1)
PY
}

echo "[*] Esperando video (hasta ${ESPERA}s)..."
if hay_video; then
    aviso "Video en vivo listo"
    VIDEO_OK=1
else
    VIDEO_OK=0
    if [ "$DIAG" -eq 0 ]; then
        zenity --warning --no-wrap --title="Cámara Dron M22" \
            --text="No llega video del dron.\n\nLo más probable es que el dron esté <b>apagado</b>:\nse apaga solo por falta de uso.\n\nPrendelo y volvé a hacer doble clic en el icono.\n\nSe va a abrir la página igual, mostrando la telemetría." \
            2>/dev/null
    else
        echo "[-] No llega video (el dron probablemente esta apagado)"
    fi
fi

# --- 4. abrir el navegador -----------------------------------------------
if [ "$DIAG" -eq 1 ]; then
    echo "[=] Diagnostico terminado. video_ok=$VIDEO_OK"
    echo "[=] En uso normal aca se abriria: $URL"
    exit 0
fi

if command -v chromium-browser >/dev/null; then
    # --app abre una ventana limpia, sin barra de direcciones
    setsid chromium-browser --app="$URL" >/dev/null 2>&1 &
else
    setsid xdg-open "$URL" >/dev/null 2>&1 &
fi

exit 0
