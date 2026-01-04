import socket
import json
from datetime import datetime

def send_json_message(ip, port, message):
  try:
   with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as sock:
    print(f"Tentative de connexion vers {ip}:{port} ...")
    sock.settimeout(1000)
    sock.connect((ip, port))
    print("Connecté !")
    sock.sendall(json.dumps(message).encode('utf-8'))
    print(f"Message envoyé : {json.dumps(message, indent=2)}")
    try:
      response = sock.recv(1024)
      print(f"Réponse reçue : {response.decode('utf-8')}")
    except socket.timeout:
      print("Pas de réponse (timeout)")
  except Exception as e:
    print(f"Erreur client : {e}")

def start_server(bind_ip='0.0.0.0', bind_port=1234):
 while True:
   with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
     server_sock.bind((bind_ip, bind_port))
     server_sock.listen(1)
     print(f"Serveur en écoute sur {bind_ip}:{bind_port}")
     
     conn, addr = server_sock.accept()
     with conn:
       print(f"Connexion de {addr}")
       data = conn.recv(1024)
       if data:
         print(f"Message reçu : {data.decode('utf8')}")
         conn.sendall(b"OK")
         
def build_message(msg_type):
 if msg_type == "status":
   return {
        "type": "status",
        "payload": {
            "id": "nucleo-01",
            "uptime": 123456,
            "ip": "192.168.1.101",
            "role": "peer",
            "last_sync": "2025-09-01T08:30:00Z"
        }
    }
 elif msg_type == "data":
   ts = datetime.now().isoformat()  # timestamp au format ISO 8601
   ax = 1.234
   ay = -0.567
   az = 0.890
   return {
      "type": "data_response",
    "id": "nucleo-01",
    "timestamp":"2025-09-01T08:30:00Z",
    "acceleration": {
        "x": round(ax, 3),
        "y": round(ay, 3),
        "z": round(az, 3)
    },
    "status": "normal"
        }
 elif msg_type == "sync":
   return {
    "type": "sync",
    "payload": {
        "source_id": "nucleo-02",
        "data": {
            "temperature": 24.1,
            "humidity": 43.8,
            "timestamp": "2025-09-01T08:32:50Z"
        }
    }
 }
 elif msg_type =="peers":
   return {
    "type": "peers",
    "payload": [
        { "id": "nucleo-02", "ip": "192.168.1.102" },
        { "id": "nucleo-03", "ip": "192.168.1.103" }
    ]
 }
 else:
   print("Type de message inconnu.")
   return None
 
def main():
  mode = input("Choisir le mode (serveur/client) : ").strip().lower()
  if mode == "serveur":
    start_server()
  elif mode == "client":
    ip = input("Adresse IP du serveur (ex: 192.168.1.101) : ").strip()
    port = int(input("Port du serveur (ex: 12345) : ").strip())
    msg_type = input("Type de message (status/data/sync/peers) : ").strip().lower()
    message = build_message(msg_type)
    if message:
      send_json_message(ip, port, message)
    else:
      print("Mode invalide. Veuillez choisir 'serveur' ou 'client'.")

if __name__ == "__main__":
    main()