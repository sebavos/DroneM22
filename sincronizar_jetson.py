"""
Script de sincronización automática hacia la NVIDIA Jetson Nano.

Transfiere todos los archivos del repositorio a la Jetson vía SSH (paramiko)
usando la contraseña guardada en ACCESO-PRIVADO.txt y configura la llave SSH
pública para que futuras conexiones no pidan contraseña.

Uso:
    python sincronizar_jetson.py              # Sincronizar archivos
    python sincronizar_jetson.py --restart    # Sincronizar y reiniciar servicio drone-stream
"""
import argparse
import os
import sys
import paramiko

JETSON_IP = "192.168.14.9"
JETSON_USER = "uavlab1"
JETSON_PASS = "UAVLAB1"
REMOTE_DIR = "/home/uavlab1"

# Archivos a sincronizar
FILES_TO_SYNC = [
    "drone_stream.py",
    "test_payload.py",
    "drone_stream_h264.py",
    "drone_stream_h265_udp.py",
    "explorar_comandos.py",
    "decodificar.py",
    "capture.py",
    "verificar_formato.py",
    "abrir_camara.sh",
    "fast_receiver.c",
    "drone_stream_fast.py",
    "drone_control.py",
    "drone_ai.py",
    "test_control.py",
    "test_arm_raw.py",
]


def setup_ssh_key(ssh):
    """Instala la llave SSH pública local en la Jetson para acceso sin clave."""
    pub_key_path = os.path.expanduser("~/.ssh/id_ed25519.pub")
    if not os.path.exists(pub_key_path):
        pub_key_path = os.path.expanduser("~/.ssh/id_rsa.pub")

    if os.path.exists(pub_key_path):
        pub_key = open(pub_key_path, "r").read().strip()
        sftp = ssh.open_sftp()
        try:
            sftp.mkdir(f"{REMOTE_DIR}/.ssh")
        except IOError:
            pass  # ya existe
        
        auth_keys_path = f"{REMOTE_DIR}/.ssh/authorized_keys"
        existing = ""
        try:
            with sftp.open(auth_keys_path, "r") as f:
                existing = f.read().decode("utf-8")
        except IOError:
            pass
        
        if pub_key not in existing:
            new_content = existing.strip() + ("\n" if existing.strip() else "") + pub_key + "\n"
            with sftp.open(auth_keys_path, "w") as f:
                f.write(new_content)
            print("[+] Llave SSH instalada en la Jetson.")
        else:
            print("[+] Llave SSH ya estaba presente en authorized_keys.")
        
        sftp.chmod(f"{REMOTE_DIR}/.ssh", 0o700)
        sftp.chmod(auth_keys_path, 0o600)
        sftp.close()


def sync_files():
    parser = argparse.ArgumentParser(description="Sincronizar archivos con la Jetson Nano")
    parser.add_argument("--restart", action="store_true", help="Reiniciar el servicio drone-stream tras sincronizar")
    parser.add_argument("--run-stream", action="store_true", help="Ejecutar drone_stream.py directamente en la Jetson")
    args = parser.parse_args()

    # Detectar si se está ejecutando erróneamente dentro de la Jetson
    if os.path.exists("/etc/nv_tegra_release") or os.environ.get("USER") == "uavlab1":
        print("[!] Este script ('sincronizar_jetson.py') se debe ejecutar en tu PC Windows, no en la Jetson.")
        print("[!] En la Jetson debes ejecutar directamente:")
        print("    python3 drone_stream.py          # Para probar el servidor de video")
        print("    python3 explorar_comandos.py     # Para explorar nuevos comandos UDP")
        return 0

    print(f"[*] Conectando a Jetson Nano ({JETSON_USER}@{JETSON_IP})...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(JETSON_IP, username=JETSON_USER, password=JETSON_PASS, timeout=5)
        print("[+] Conexión SSH establecida.")
    except Exception as e:
        print(f"[-] Error de conexión SSH: {e}")
        print("    Asegúrate de que la Jetson está encendida y accesible en 192.168.14.8")
        return 1

    # Configurar la clave pública
    setup_ssh_key(ssh)

    # Transferir archivos vía SFTP
    sftp = ssh.open_sftp()
    local_base = os.path.dirname(os.path.abspath(__file__))

    print("\n[*] Sincronizando archivos...")
    for filename in FILES_TO_SYNC:
        local_path = os.path.join(local_base, filename)
        if os.path.exists(local_path):
            remote_path = f"{REMOTE_DIR}/{filename}"
            sftp.put(local_path, remote_path)
            # Asegurar permisos de ejecución si es .sh
            if filename.endswith(".sh"):
                sftp.chmod(remote_path, 0o755)
            print(f"  -> {filename} enviada a {remote_path}")
    sftp.close()

    print("\n[+] Sincronización completada con éxito.")

    if args.restart:
        print("[*] Reiniciando servicio drone-stream en la Jetson...")
        stdin, stdout, stderr = ssh.exec_command("sudo systemctl restart drone-stream")
        print(stdout.read().decode())
        print("[+] Servicio drone-stream reiniciado.")

    ssh.close()
    return 0


if __name__ == "__main__":
    sys.exit(sync_files())
