import zmq
import numpy as np
import cv2
import time
from deoxys.utils import YamlConfig
from real_robot_scripts.real_robot_utils import RealRobotObsProcessor
from rebar_scripts.pose_estimation.slot_detector.detect_aurco import ArUcoDetector


class Tidybot2:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        # Suppose the remote server has IP 192.168.1.123, port 6666
        self.socket.connect("tcp://10.50.245.100:6666")

        self.obs_cfg = YamlConfig("rebar_configs/tidybot_obs_cfg.yml").as_easydict()
        self.obs_processor = RealRobotObsProcessor(self.obs_cfg, processor_name="ImageProcessor")

        self.marker_cfg = YamlConfig("rebar_configs/markers.yml").as_easydict()
        self.ArUcoDetector = ArUcoDetector(dictionary=cv2.aruco.DICT_6X6_250)

    def move(self, target_pose):
        while True:
            # Default: abs target pose, local frame
            rep = {'action': target_pose}  # i / 100 * 1.05 # min(0.2, i / 50) * 3.14   0* 3.14
            self.socket.send_pyobj(rep)
            reply = self.socket.recv_pyobj()
            # print("Received reply:", reply)
            if (
                    np.max(
                        np.abs(
                            np.array(reply["base_pose"]) # TODO: figure out how to get the current pose
                            - np.array(target_pose)
                        )
                    )
                    < 0.001  # org 1e-3
            ):
                break
    def detect_marker(self, target="Desk", id=0):
        target_marker_id = self.marker_cfg[target][id]
        intrinsics = {
            "fx": self.marker_cfg["cam_intrinsics"]["fx"],
            "fy": self.marker_cfg["cam_intrinsics"]["fy"],
            "cx": self.marker_cfg["cam_intrinsics"]["cx"],
            "cy": self.marker_cfg["cam_intrinsics"]["cy"]
        }
        tag_size = self.marker_cfg["tag_size"]
        self.obs_processor.get_real_robot_img_obs()
        obs = self.obs_processor.obs
        rgb_frame = obs["agentview5_rgb"]
        # if target=="Desk":
        #     rgb_frame = obs["agentview5"]
        # else:
        #     rgb_frame = obs["webcam_6"]
        results, _ = self.ArUcoDetector.detect(rgb_frame, img_idx=0, intrinsics=intrinsics, tag_size=tag_size)
        marker_pose_R = None
        marker_pose_t = None
        for marker in results:
            if marker["id"] == target_marker_id:
                marker_pose_R = marker["pose_R"]
                marker_pose_t = marker["pose_t"]
        assert marker_pose_R is not None, "Target marker was not found"
        marker_pose_mat = np.eye(4)
        marker_pose_mat[:3, :3] = marker_pose_R
        marker_pose_mat[:3, 3] = marker_pose_t

        return marker_pose_mat

    def get_base_action(self, cam_in_base_mat, marker_pose_mat, target_pose_mat):
        """
        Args:
            cam_in_base_mat: 4x4
            marker_pose_mat: 4x4,
            target_pose_mat: 4x4
        Returns:
            target x,y,theta in the local frame of base, np.array 3,
        """

        # marker_in_base = cam_in_base_mat @ marker_pose_mat
        # target_in_base = cam_in_base_mat @ target_pose_mat
        # diff  = marker_in_base @ np.linalg.inv(target_in_base)
        diff_in_base = (
                cam_in_base_mat
                @ marker_pose_mat
                @ np.linalg.inv(target_pose_mat)
                @ np.linalg.inv(cam_in_base_mat)
        )
        dx = diff_in_base[0, 3]
        dy = diff_in_base[1, 3]
        # Extract yaw (theta) around Z from the top-left 2x2 rotation
        # Note: the standard approach is arctan2(r21, r11) = np.arctan2(R[1,0], R[0,0])
        dtheta = np.arctan2(diff_in_base[1, 0], diff_in_base[0, 0])

        return np.array([dx, dy, dtheta])

    def move_to_marker(self, target="Desk", id=0):
        """10 HZ close loop control"""
        zmq_init = False
        while True:
            if not zmq_init:
                rep = {'action': None}
                self.socket.send_pyobj(rep)
                reply = self.socket.recv_pyobj()
                state = np.array(reply["base_pose"])
                zmq_init = True
            # Default: abs target pose, local frame
            close_loop_start_time = time.time()
            marker_detect_start = time.time()
            marker_pose_mat = self.detect_marker(target, id)

            cam_in_base_mat = np.array(self.marker_cfg["cam_in_base"][target])
            target_pose_mat = np.array(self.marker_cfg["target_rel_pose"][target])
            action = self.get_base_action(cam_in_base_mat, marker_pose_mat, target_pose_mat) #TODO: calibrate the cam

            rep = {'action': action + state}
            self.socket.send_pyobj(rep)

            print("marker detection per frame:", time.time() - marker_detect_start)
            print("np.array(action):", np.array(action))

            if time.time() - close_loop_start_time < 0.1: # HACK 10 HZ
                time.sleep(time.time() - close_loop_start_time)

            reply = self.socket.recv_pyobj()
            state = np.array(reply["base_pose"])

            if (
                    np.max(
                        np.abs(
                            np.array(action)
                        )
                    )
                    < 0.005  # org 1e-3
            ):

                break

if __name__ == "__main__":
    base = Tidybot2()
    target_pose = np.array([0.0, 1.88, 0.0])
    base.move(target_pose)