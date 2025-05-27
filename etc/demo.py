from lib import DobotDllType as dType
from utils.BaseClass import BaseClass


class Demo(BaseClass):
    def body(self):
        state = self.state
        api = self.api

        if (state == dType.DobotConnect.DobotConnect_NoError):
            # Clean Command Queued
            dType.SetQueuedCmdClear(self.api)

            self.move2home_pos()

            # Async PTP Motion
            for i in range(0, 5):
                if i % 2 == 0:
                    offset = 50
                else:
                    offset = -50
                lastIndex = \
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200 + offset, offset, offset, offset,
                                    isQueued=1)[0]

            # Start to Execute Command Queue
            dType.SetQueuedCmdStartExec(api)

            # Wait for Executing Last Command
            while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                dType.dSleep(100)

            # Stop to Execute Command Queued
            dType.SetQueuedCmdStopExec(api)


if __name__ == "__main__":
    Demo().body()
