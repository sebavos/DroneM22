import requests
import time

# Cambia esto si corres el script desde otra PC
JETSON_IP = "192.168.14.9"
BASE_URL = f"http://{JETSON_IP}:5000/api"

def print_menu():
    print("\n" + "="*40)
    print(" 🕹️  MENÚ DE PRUEBA DE DRON M22")
    print("="*40)
    print("1. Armar dron (Habilita motores)")
    print("2. Desarmar dron (Seguridad)")
    print("3. Takeoff (Despegar)")
    print("4. Land (Aterrizar)")
    print("5. Emergency STOP (¡Apagar motores ya!)")
    print("6. Mover (Throttle 50% por 2 seg)")
    print("7. Calibrar giroscopio")
    print("8. Tomar Foto")
    print("9. Activar IA (Seguir persona)")
    print("0. Salir")
    print("="*40)

def send_post(endpoint, json_data=None):
    url = f"{BASE_URL}/{endpoint}"
    try:
        if json_data:
            res = requests.post(url, json=json_data, timeout=2)
        else:
            res = requests.post(url, timeout=2)
        print(f"\n[Respuesta del Servidor]: {res.json()}")
    except Exception as e:
        print(f"\n[Error]: No se pudo conectar a {url}. ¿El servidor está corriendo?")
        print(e)

def main():
    while True:
        print_menu()
        opcion = input("Elige una opción: ")

        if opcion == "1":
            send_post("arm")
        elif opcion == "2":
            send_post("disarm")
        elif opcion == "3":
            send_post("takeoff")
        elif opcion == "4":
            send_post("land")
        elif opcion == "5":
            send_post("emergency")
        elif opcion == "6":
            print("Enviando Throttle 50%...")
            # Enviar comando de movimiento
            send_post("rc", {"throttle": 50, "roll": 0, "pitch": 0, "yaw": 0})
            time.sleep(2)
            print("Regresando a hover (neutral)...")
            send_post("hover")
        elif opcion == "7":
            send_post("calibrate")
        elif opcion == "8":
            send_post("photo")
        elif opcion == "9":
            send_post("ai/start", {"target_class": "person"})
        elif opcion == "0":
            print("Saliendo...")
            break
        else:
            print("Opción no válida.")

if __name__ == "__main__":
    main()
