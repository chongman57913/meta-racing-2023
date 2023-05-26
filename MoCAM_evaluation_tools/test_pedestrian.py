import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random

def main():
    synchronous_master = False
    try:
        SpawnActor = carla.command.SpawnActor
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()

        # 拿到这个世界所有物体的蓝图
        bp4 = world.get_blueprint_library().find('walker.pedestrian.0001')
        blueprint_library = world.get_blueprint_library()

        spawn_point = carla.Transform(carla.Location(x=-12.5, y=1.5, z=0.2),carla.Rotation())

        pedestrain = world.spawn_actor(bp4, spawn_point)
        control = carla.WalkerControl()
        revert_flag = False

        while True:
            # world.wait_for_tick()
            control = carla.WalkerControl()
            control.direction.y =0
            control.direction.z = 0
            control.speed =0.3
            if(pedestrain.get_location().x>14.8):
                revert_flag = True
            if(pedestrain.get_location().x<-12.5):
                revert_flag = False
            if(revert_flag):
                control.direction.x = -1
            else:
                control.direction.x = 1
            pedestrain.apply_control(control)
            time.sleep(1)
        
    finally:
        if pedestrain:
            pedestrain.destroy()

        time.sleep(10)
if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
