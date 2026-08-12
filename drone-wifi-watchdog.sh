#!/bin/bash
# El dron M22 se apaga solo por inactividad. Cuando eso pasa, wlan0 se va a otra
# red conocida (la del campus) y NetworkManager NO vuelve al dron por su cuenta:
# la prioridad de autoconnect solo se evalua al momento de conectar, no desplaza
# una conexion que ya funciona. Este watchdog cubre exactamente ese hueco.
SSID="LYHFPV_M22 2B967C"

activo=$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null \
         | grep ':wlan0$' | cut -d: -f1)

# Ya esta donde queremos
[ "$activo" = "$SSID" ] && exit 0

# Solo intentar si el AP del dron esta efectivamente visible; si el dron sigue
# apagado no tiene sentido cortar la conexion actual.
if nmcli -t -f SSID device wifi list 2>/dev/null | grep -qxF "$SSID"; then
    logger -t drone-wifi "dron visible, wlan0 en '${activo:-nada}' -> reconectando"
    if nmcli connection up "$SSID" >/dev/null 2>&1; then
        logger -t drone-wifi "reconectado al dron"
    else
        logger -t drone-wifi "fallo la reconexion al dron"
    fi
fi
exit 0
