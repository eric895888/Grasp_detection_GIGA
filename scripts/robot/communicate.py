import socket

class Communicate:
    def __init__(self):
        self.host = '192.168.255.10'
        self.port = 11000
        self.step = -1
        self.sock = None
        #self.block = False

    def sock_setting(self):
        HOST = self.host
        PORT = self.port

        # create socket
        try:
            socket.setdefaulttimeout(5)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except OSError as msg:
            self.sock = None
            print(msg)
            return -1, str(msg)
        # connect to server
        try:
            self.sock.connect((HOST,PORT))
        except OSError as msg:
            print('hi')
            self.sock.close()
            self.sock = None
            print(msg)
            return -1, "Socket "+str(msg)

        print('Connected Successfully: %s,%s' %(HOST,PORT))
        return 0, "Connected successfully"

    def send_data_to_robot(self, Tx, Ty, Tz, Rx, Ry, Rz, speed, step):
        send_data = str(Tx) + ',' + str(Ty) + ',' + str(Tz) + ',' + \
               str(Rx) + ',' + str(Ry) + ',' + str(Rz) + ',' + \
               str(speed) + ',' + str(step) + ';'
        #send_byte_data = bytearray(1024)
        #send_byte_data = bytearray(send_data.encode())
        send_byte_data = send_data.encode()
        if self.sock is None:
            print('socket not open: cannot send data')
            return
        # send data
        try:
            # while self.block:
            #     None
            self.sock.send(send_byte_data)
            #self.block = True
        except OSError:
            print('Error sending')
        return

    def send_data_to_robot_close(self):
        send_data = 'c'
        send_byte_data = send_data.encode()
        if self.sock is None:
            print('socket not open: cannot send data')
            return -1, "Socket not open"
        # send data
        try:
            self.sock.send(send_byte_data)
            self.sock.close()
        except OSError:
            print('Error sending')
            return -1, "Error sending"

        return 0, "Close successfully"

    def recv_data_from_robot(self):
        # receive data
        receive_data = None
        try:
            # while not self.block:
            #     None
            receive_byte_data = self.sock.recv(1024)
            #self.block = False
            receive_data = receive_byte_data.decode()
        except OSError:
            print('Error receiving')
            return receive_data

        if receive_data.find(';') == -1:
            print(receive_data)
            print('Receive data error: not include \';\'')
            return receive_data
        if receive_data.find(';') != len(receive_data)-1:
            receive_data = receive_data[0:receive_data.find(';') + 1]
        split_data = receive_data.split(',',5)
        if len(split_data) != 6:
            split_data[0] = split_data[0].split(';',1)[0]
        else:
            split_data[5] = split_data[5].split(';',1)[0]
        return split_data




