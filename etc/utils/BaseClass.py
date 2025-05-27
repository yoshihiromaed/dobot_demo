import threading

from lib import DobotDllType as dType
from abc import ABC, abstractmethod
from lib.DobotDllType import GetPose
import numpy as np

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}


class BaseClass(ABC):
    def __init__(self):
        # Load Dll and get the CDLL object
        api = dType.load()
        # Connect Dobot
        state = dType.ConnectDobot(api, "", 115200)[0]
        print("Connect status:", CON_STR[state])

        self.api = api
        self.state = state
        self.thread_list = []
        self.R_min = 115
        self.R_max = 320

    def __del__(self):
        # Disconnect Dobotq
        dType.DisconnectDobot(self.api)

    def clamp_radius(self, x, y, r_min=None, r_max=None):
        if r_min is None:
            r_min = self.R_min
        if r_max is None:
            r_max = self.R_max

        r = np.sqrt(x ** 2 + y ** 2)

        if r < r_min:
            scale = r_min / r
        elif r > r_max:
            scale = r_max / r
        else:
            scale = 1

        return x * scale, y * scale

    def get_position(self):
        return GetPose(self.api)[:4]

    def exec_dobot_cmd(self, lastIndex: int):
        for _thread in self.thread_list[:]:
            if _thread is not threading.current_thread():
                _thread.join()
                self.thread_list.remove(_thread)

        # Start to Execute Command Queue
        dType.SetQueuedCmdStartExec(self.api)

        # Wait for Executing Last Command
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(100)
            print(self.get_position())

        # Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(self.api)

    def move2home_pos(self):
        if (self.state != dType.DobotConnect.DobotConnect_NoError):
            assert "state error", self.state

        print("start to move to home pos")

        # Clean Command Queued
        dType.SetQueuedCmdClear(self.api)

        # Async Motion Params Setting
        dType.SetHOMEParams(self.api, 200, 0, 0, 0, isQueued=1)
        dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued=1)
        dType.SetPTPCommonParams(self.api, 100, 100, isQueued=1)

        # Async Home
        lastIndex = dType.SetHOMECmd(self.api, temp=0, isQueued=1)[0]

        self.exec_dobot_cmd(lastIndex)
        print("end to move to home pos")

    @abstractmethod
    def body(self):
        pass
