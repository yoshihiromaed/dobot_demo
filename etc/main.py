import threading

from lib import DobotDllType as dType
from utils.BaseClass import BaseClass
import numpy as np
import cv2
import keyboard


class Main(BaseClass):
    def __init__(self):
        super().__init__()

        self.camera_id = 1

        self.robot_cmds = np.array([
            [158.97, 118.90, 0, 0],  # Dobot pos [x, y, z, r]
            [298.12, 118.19, 0, 0],  # Dobot pos [x, y, z, r]
            [287.50, -99.33, 0, 0],  # Dobot pos [x, y, z, r]
            [161.33, -98.86, 0, 0],  # Dobot pos [x, y, z, r]
        ], dtype=np.float32)

        self.img_points = np.array([
            [128, 329],  # img pos [x, y]
            [128, 113],  # img pos [x, y]
            [468, 122],  # img pos [x, y]
            [477, 314],  # img pos [x, y]
        ], dtype=np.float32)

        self.homography_matrix, _ = cv2.findHomography(self.img_points, self.robot_cmds[:, :2])

    def image_to_robot_coords(self, px, py):
        input_point = np.array([[px, py]], dtype=np.float32)
        output_point = cv2.perspectiveTransform(np.array([input_point]), self.homography_matrix)
        return output_point[0][0]

    def check_arm_cord4calibration(self):
        self.move2home_pos()

        while True:
            print(self.get_position())
            if keyboard.is_pressed('q'):
                break

    def check_image_cord4calibration(self):
        def click_event(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                print(f"clicked pos: ({x}, {y})")

                frame = param.copy()
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

                cv2.imshow("Camera", frame)

        self.move2home_pos()

        cap = cv2.VideoCapture(self.camera_id)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow("Camera", frame)
            cv2.setMouseCallback("Camera", click_event, frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def test_homography_matrix(self):
        self.move2home_pos()

        clicked_x, clicked_y = None, None

        def click_event(event, x, y, flags, param):
            global clicked_x, clicked_y
            if event == cv2.EVENT_LBUTTONDOWN:
                clicked_x, clicked_y = x, y

                print(f"clicked pos: ({clicked_x}, {clicked_y})")

                robot_x, robot_y = self.image_to_robot_coords(clicked_x, clicked_y)
                robot_x, robot_y = self.clamp_radius(robot_x, robot_y)

                print(f"Dobot cord: X={robot_x:.2f}, Y={robot_y:.2f}")

                lastIndex = dType.SetPTPCmd(
                    self.api, dType.PTPMode.PTPMOVLXYZMode,
                    robot_x, robot_y, 0, 0,
                    isQueued=1
                )[0]

                thread = threading.Thread(target=self.exec_dobot_cmd, args=(lastIndex,))
                thread.start()

                self.thread_list.append(thread)

        cap = cv2.VideoCapture(self.camera_id)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow("Camera", frame)
            cv2.setMouseCallback("Camera", click_event)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def detect_color_region(self, img, color='red'):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if color == 'red':
            lower1 = np.array([0, 120, 70])
            upper1 = np.array([10, 255, 255])

            lower2 = np.array([170, 120, 70])
            upper2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)

            mask = mask1 + mask2
        else:
            lower1 = np.array([85, 50, 50])
            upper1 = np.array([130, 255, 255])

            mask = cv2.inRange(hsv, lower1, upper1)

        red_region = cv2.bitwise_and(img, img, mask=mask)

        cv2.imshow("Region", red_region)
        cv2.erode(mask, None, mask, iterations=2)
        cv2.dilate(mask, None, mask, iterations=5)
        cv2.imshow("Mask", mask)

        moments = cv2.moments(mask)

        if moments["m00"] != 0:
            cX = int(moments["m10"] / moments["m00"])
            cY = int(moments["m01"] / moments["m00"])
        else:
            cX, cY = 0, 0

        print(f"centroid: ({cX}, {cY})")

        return cX, cY, mask

    def body(self):
        self.move2home_pos()

        color = 'red'
        clicked_x, clicked_y = None, None
        suction_cup_state = False
        cX, cY = 0, 0

        def click_event(event, x, y, flags, param):
            global clicked_x, clicked_y
            nonlocal suction_cup_state, cX, cY, color
            if event == cv2.EVENT_LBUTTONDOWN:
                clicked_x, clicked_y = x, y

                if not suction_cup_state:
                    clicked_x, clicked_y = cX, cY
                    if cX == 0 and cY == 0:
                        clicked_x, clicked_y = x, y
                else:
                    clicked_x, clicked_y = x, y

                print(f"clicked pos: ({clicked_x}, {clicked_y})")

                robot_x, robot_y = self.image_to_robot_coords(clicked_x, clicked_y)
                robot_x, robot_y = self.clamp_radius(robot_x, robot_y)
                print(f"Dobot cord: X={robot_x:.2f}, Y={robot_y:.2f}")

                if not suction_cup_state:
                    z = -60
                    if color == 'blue':
                        z -= 2
                    dType.SetPTPCmd(
                        self.api, dType.PTPMode.PTPMOVLXYZMode,
                        robot_x, robot_y, z, 0,
                        isQueued=1
                    )
                    dType.SetEndEffectorSuctionCup(self.api, 1, 1, isQueued=1)
                    lastIndex = dType.SetPTPCmd(
                        self.api, dType.PTPMode.PTPMOVLXYZMode,
                        robot_x, robot_y, 0, 0,
                        isQueued=1
                    )[0]

                    if color == 'red':
                        color = 'blue'
                    else:
                        color = 'red'
                else:
                    dType.SetPTPCmd(
                        self.api, dType.PTPMode.PTPMOVLXYZMode,
                        robot_x, robot_y, -55, 0,
                        isQueued=1
                    )
                    dType.SetEndEffectorSuctionCup(self.api, 1, 0, isQueued=1)
                    lastIndex = dType.SetPTPCmd(
                        self.api, dType.PTPMode.PTPMOVLXYZMode,
                        robot_x, robot_y, 0, 0,
                        isQueued=1
                    )[0]

                suction_cup_state = not suction_cup_state

                thread = threading.Thread(target=self.exec_dobot_cmd, args=(lastIndex,))
                thread.start()

                self.thread_list.append(thread)

        cap = cv2.VideoCapture(self.camera_id)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow("Camera", frame)
            cX, cY, _ = self.detect_color_region(frame, color=color)
            cv2.setMouseCallback("Camera", click_event)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main = Main()
    # main.check_arm_cord4calibration()
    # main.check_image_cord4calibration()
    # main.test_homography_matrix()
    Main().body()
