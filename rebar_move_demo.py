import zmq
import time
import numpy as np
from base_server import BaseManager
from constants import BASE_RPC_HOST, BASE_RPC_PORT, RPC_AUTHKEY
from constants import POLICY_CONTROL_PERIOD

def main():
    # set up a ZMQ context and a REP socket
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    # Bind to an address/port that is accessible from local PC at port 6666
    socket.bind("tcp://*:6666")

    manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    base = manager.Base(max_vel=(0.5, 0.5, 1.57), max_accel=(0.25, 0.25, 0.79))
    base.reset()
    print("Base is reset and ready. Listening for commands via ZMQ...")

    try:
        while True:
            # 1. Wait for next request from client
            message = socket.recv_pyobj()  
            
            # 2. Parse the message. Example: { "action": [0.0, 0.0, 1.57] }
            action = message.get("action", None)
            # 3. Execute action if provided
            if action is not None:
                base.execute_action({'base_pose': np.array(action)})
                print("Executed action:", action)
            else:
                pass
            robot_state = socket.send_pyobj(base.get_state())
            print(base.get_state())
            time.sleep(POLICY_CONTROL_PERIOD) # Note: Not precise
    
    finally:
        base.close()
        print("Base movement over.")

if __name__ == "__main__":
    main()
