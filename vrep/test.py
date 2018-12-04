import vrep
from vrep import simx_opmode_blocking as blocking
from time import sleep

class Test():
    def __init__(self):
        self._client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 1000, 5)
        print 'ClientID: %d', self._client_id
        handle = self.get_obj_handle('Wheel0_SteerJoint')
        print handle

    def get_obj_handle(self, obj_name):
        _, handle = vrep.simxGetObjectHandle(self._client_id, obj_name, blocking)
        return handle


Test()
