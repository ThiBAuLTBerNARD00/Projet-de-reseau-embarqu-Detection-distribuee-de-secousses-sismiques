import socket
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
MESSAGE="Hllo"
sock.sendto(MESSAGE.encode(),("192.168.1.180",1234))