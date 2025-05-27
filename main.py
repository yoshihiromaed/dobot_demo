import threading
from lib import DobotDllType as dType
import numpy as np
import cv2
import keyboard
from image_utils import detect_color_region

from dobot_utils import init_dobot, move2home_pos, clamp_radius, get_position, exec_dobot_cmd


def init():
    dobot_handle = init_dobot()

    dobot_handle.camera_id = 1

    # 4 point position in arm coordinate system
    # The robot_cmds are the positions of the four points in the arm coordinate system.
    # These point correspond to the points in image coordinate system for each point
    dobot_handle.robot_pos = np.array([
        [158.97, 118.90, 0, 0],  # Dobot pos [x, y, z, r]
        [298.12, 118.19, 0, 0],  # Dobot pos [x, y, z, r]
        [287.50, -99.33, 0, 0],  # Dobot pos [x, y, z, r]
        [161.33, -98.86, 0, 0],  # Dobot pos [x, y, z, r]
    ], dtype=np.float32)

    # 4 point position in image coordinate system
    # The img_points are the positions of the four points in the image coordinate system.
    # These point correspond to the points in arm coordinate system for each point
    dobot_handle.img_points = np.array([
        [128, 329],  # img pos [x, y]
        [128, 113],  # img pos [x, y]
        [468, 122],  # img pos [x, y]
        [477, 314],  # img pos [x, y]
    ], dtype=np.float32)

    # Calculate homography matrix
    # The homography matrix is used to transform points from the image coordinate system to the robot coordinate system
    dobot_handle.homography_matrix, _ = cv2.findHomography(dobot_handle.img_points, dobot_handle.robot_pos[:, :2])

    return dobot_handle


def image_to_robot_coords(dobot_handle, px, py):
    input_point = np.array([[px, py]], dtype=np.float32)
    output_point = cv2.perspectiveTransform(np.array([input_point]), dobot_handle.homography_matrix)
    return output_point[0][0]


def check_arm_cord4calibration(dobot_handle):
    move2home_pos(dobot_handle)

    while True:
        print(get_position(dobot_handle))
        if keyboard.is_pressed('q'):
            break


def check_image_cord4calibration(dobot_handle):
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"clicked pos: ({x}, {y})")

            frame = param.copy()
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            cv2.imshow("Camera", frame)

    move2home_pos(dobot_handle)

    cap = cv2.VideoCapture(dobot_handle.camera_id)

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


def test_homography_matrix(dobot_handle):
    move2home_pos(dobot_handle)

    clicked_x, clicked_y = None, None

    def click_event(event, x, y, flags, param):
        global clicked_x, clicked_y
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_x, clicked_y = x, y

            print(f"clicked pos: ({clicked_x}, {clicked_y})")

            robot_x, robot_y = image_to_robot_coords(dobot_handle, clicked_x, clicked_y)
            robot_x, robot_y = clamp_radius(dobot_handle, robot_x, robot_y)

            print(f"Dobot cord: X={robot_x:.2f}, Y={robot_y:.2f}")

            lastIndex = dType.SetPTPCmd(
                dobot_handle.api, dType.PTPMode.PTPMOVLXYZMode,
                robot_x, robot_y, 0, 0,
                isQueued=1
            )[0]

            thread = threading.Thread(target=exec_dobot_cmd, args=(dobot_handle, lastIndex,))
            thread.start()

            dobot_handle.thread_list.append(thread)

    cap = cv2.VideoCapture(dobot_handle.camera_id)

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


def main(dobot_handle):
    move2home_pos(dobot_handle)

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

            robot_x, robot_y = image_to_robot_coords(dobot_handle, clicked_x, clicked_y)
            robot_x, robot_y = clamp_radius(dobot_handle, robot_x, robot_y)
            print(f"Dobot cord: X={robot_x:.2f}, Y={robot_y:.2f}")

            if not suction_cup_state:
                z = -60
                if color == 'blue':
                    z -= 2
                dType.SetPTPCmd(
                    dobot_handle.api, dType.PTPMode.PTPMOVLXYZMode,
                    robot_x, robot_y, z, 0,
                    isQueued=1
                )
                dType.SetEndEffectorSuctionCup(dobot_handle.api, 1, 1, isQueued=1)
                lastIndex = dType.SetPTPCmd(
                    dobot_handle.api, dType.PTPMode.PTPMOVLXYZMode,
                    robot_x, robot_y, 0, 0,
                    isQueued=1
                )[0]

                if color == 'red':
                    color = 'blue'
                else:
                    color = 'red'
            else:
                dType.SetPTPCmd(
                    dobot_handle.api, dType.PTPMode.PTPMOVLXYZMode,
                    robot_x, robot_y, -55, 0,
                    isQueued=1
                )
                dType.SetEndEffectorSuctionCup(dobot_handle.api, 1, 0, isQueued=1)
                lastIndex = dType.SetPTPCmd(
                    dobot_handle.api, dType.PTPMode.PTPMOVLXYZMode,
                    robot_x, robot_y, 0, 0,
                    isQueued=1
                )[0]

            suction_cup_state = not suction_cup_state

            thread = threading.Thread(target=exec_dobot_cmd, args=(dobot_handle, lastIndex,))
            thread.start()

            dobot_handle.thread_list.append(thread)

    cap = cv2.VideoCapture(dobot_handle.camera_id)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("Camera", frame)
        cX, cY, _ = detect_color_region(frame, color=color)
        cv2.setMouseCallback("Camera", click_event)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Initialize Dobot
    # dobot_handle: Dobot handle object
    # dobot_handle.api: Dobot API object
    # dobot_handle.state: Dobot connection state
    # dobot_handle.thread_list: List of threads for Dobot commands
    # dobot_handle.R_min: Minimum radius for Dobot
    # dobot_handle.R_max: Maximum radius for Dobot
    # dobot_handle.camera_id: Camera ID for video capture (append this init function)
    # dobot_handle.robot_cmds: Robot commands for calibration (append this init function)
    # dobot_handle.img_points: Image points for calibration (append this init function)
    # dobot_handle.homography_matrix: Homography matrix for image to robot coordinates transformation (append this init function)
    dobot_handle = init()

    # Check arm coordinates for calibration
    # The function check_arm_cord4calibration is used to check the arm coordinates for calibration.
    # Move the arm yourself, check the positions of the corresponding 4-point arm coordinate system,
    # and enter them in dobot_handle.robot_pos.
    # check_arm_cord4calibration(dobot_handle)

    # Check image coordinates for calibration
    # The function check_image_cord4calibration is used to check the image coordinates for calibration.
    # click the positions of the corresponding 4-point image coordinate system, and enter them in dobot_handle.img_points.
    # check_image_cord4calibration(dobot_handle)

    # Test homography matrix
    # The function test_homography_matrix is used to test the homography matrix.
    # The expected action is that the arm moves to the location clicked on the image.
    # test_homography_matrix(dobot_handle)

    # Main function
    main(dobot_handle)
