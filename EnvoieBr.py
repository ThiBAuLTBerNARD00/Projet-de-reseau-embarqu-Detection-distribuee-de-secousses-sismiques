import socket

UDP_IP = "192.168.1.180"
UDP_PORT = 1234
MESSAGE = "Hello!"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.sendto(MESSAGE.encode(), (UDP_IP, UDP_PORT))

print(f"Message envoyé à {UDP_IP}:{UDP_PORT}")
