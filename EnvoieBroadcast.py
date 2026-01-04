import socket
import time
import json

BROADCAST_IP = "192.168.1.255"   # même masque que ta carte
BROADCAST_PORT = 1234            # port où ton STM32 écoute

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print("Envoi de broadcasts vers la carte STM32...")

    while True:
        # Exemple de message JSON (similaire à ce que ta carte envoie)
        message = {
            "type": "ping",
            "id": "python-tool",
            "timestamp": time.time()
        }

        data = json.dumps(message).encode("utf-8")

        sock.sendto(data, (BROADCAST_IP, BROADCAST_PORT))
        print(f"> Broadcast envoyé : {data}")

        time.sleep(5)   # toutes les 5 secondes

if __name__ == "__main__":
    main()
