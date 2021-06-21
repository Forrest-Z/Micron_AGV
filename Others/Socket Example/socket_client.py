# run this on the TX2 side

import socket

if __name__ == '__main__':
    ip_addr = "localhost"
    port_num = 8888
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip_addr, port_num))
    print("Socket Connected")

    while True:
        client_socket.send("Hello I am the Client\r\n".encode("utf-8"))
