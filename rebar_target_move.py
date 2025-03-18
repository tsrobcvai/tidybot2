import zmq
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.REQ)

# Suppose the remote server has IP 192.168.1.123, port 6666
socket.connect("tcp://10.50.245.100:6666")

for i in range(100):
    # Default: abs target pose, local frame
    rep = {'action': np.array([0.0, 1.88, 0.0])}  # i / 100 * 1.05
     # min(0.2, i / 50) * 3.14   0* 3.14
    socket.send_pyobj(rep)
    reply = socket.recv_pyobj()
    # print("Received reply:", reply)