import threading
from lib import DobotDllType as dType
from lib.DobotDllType import GetPose
import numpy as np

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

class Dobot_Handle(object):
    pass

def init_dobot():
    dobot_handle = Dobot_Handle()
    # Load Dll and get the CDLL object
    api = dType.load()
    # Connect Dobot
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:", CON_STR[state])

    dobot_handle.api = api
    dobot_handle.state = state
    dobot_handle.thread_list = []
    dobot_handle.R_min = 115
    dobot_handle.R_max = 320

    return dobot_handle


def destory_dobot(dobot_handle):
    # Disconnect Dobotq
    dType.DisconnectDobot(dobot_handle.api)


def clamp_radius(dobot_handle, x, y, r_min=None, r_max=None):
    if r_min is None:
        r_min = dobot_handle.R_min
    if r_max is None:
        r_max = dobot_handle.R_max

    r = np.sqrt(x ** 2 + y ** 2)

    if r < r_min:
        scale = r_min / r
    elif r > r_max:
        scale = r_max / r
    else:
        scale = 1

    return x * scale, y * scale


def get_position(dobot_handle):
    return GetPose(dobot_handle.api)[:4]


def exec_dobot_cmd(dobot_handle, lastIndex: int):
    for _thread in dobot_handle.thread_list[:]:
        if _thread is not threading.current_thread():
            _thread.join()
            dobot_handle.thread_list.remove(_thread)

    # Start to Execute Command Queue
    dType.SetQueuedCmdStartExec(dobot_handle.api)

    # Wait for Executing Last Command
    while lastIndex > dType.GetQueuedCmdCurrentIndex(dobot_handle.api)[0]:
        dType.dSleep(100)
        print(get_position(dobot_handle))

    # Stop to Execute Command Queued
    dType.SetQueuedCmdStopExec(dobot_handle.api)


def move2home_pos(dobot_handle):
    if (dobot_handle.state != dType.DobotConnect.DobotConnect_NoError):
        assert "state error", dobot_handle.state

    print("start to move to home pos")

    # Clean Command Queued
    dType.SetQueuedCmdClear(dobot_handle.api)

    # Async Motion Params Setting
    dType.SetHOMEParams(dobot_handle.api, 200, 0, 0, 0, isQueued=1)
    dType.SetPTPJointParams(dobot_handle.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
    dType.SetPTPCommonParams(dobot_handle.api, 100, 100, isQueued=1)

    # Async Home
    lastIndex = dType.SetHOMECmd(dobot_handle.api, temp=0, isQueued=1)[0]

    exec_dobot_cmd(dobot_handle, lastIndex)
    print("end to move to home pos")
