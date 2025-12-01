import socket

def udp_server(ip="0.0.0.0", port=1234):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((ip, port))
    print(f"Serveur UDP en écoute sur {ip}:{port}")

    while True:
        data, addr = sock.recvfrom(1024)
        print(f"\n📡 Message reçu de {addr} :")
        print(data.decode())

udp_server()
