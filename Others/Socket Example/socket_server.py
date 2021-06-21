# run this on the ros side

import socket


class Server():
    def __init__(self, ip_addr, port_num):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip_addr, port_num)
        self.sock.bind(server_address)
        self.sock.listen(1)

    def run(self):
        print('waiting for a connection')
        self.connection, client_address = self.sock.accept()
        print('connected')
        while True:
            data = self.connection.recv(1024)
            if data:
                msg = data.decode("utf-8")
                print(msg)

    def stop(self):
        self.connection.close()
        self.shutdown.set()


if __name__ == '__main__':
    # ip_addr = 'localhost'
    ip_addr = '10.169.36.30'
    port_num = 8889
    my_server = Server(ip_addr, port_num)

    my_server.run()
