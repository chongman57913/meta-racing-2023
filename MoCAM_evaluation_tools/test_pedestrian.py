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
        # 从浩瀚如海的蓝图中找到奔驰的蓝图
        # ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        # 给我们的车加上特定的颜色
        # ego_vehicle_bp.set_attribute('color', '52, 192, 235')

        # ego_vehicle_bp_2 = blueprint_library.find('vehicle.audi.etron')
        # ego_vehicle_bp_2.set_attribute('color', '79, 235, 52')
        # 找到所有可以作为初始点的位置并随机选择一个
        # transform = random.choice(world.get_map().get_spawn_points())
        
        # 设置固定点
        # transform = carla.Transform(carla.Location(x=-13, y=80, z=2), carla.Rotation(yaw=90))
        # 在这个位置生成汽车
        # ego_vehicle_0 = world.spawn_actor(ego_vehicle_bp, transform)

        # ego_vehicle_1 = world.spawn_actor(ego_vehicle_bp_2, carla.Transform(carla.Location(x=-22.2, y=134.8, z=2), carla.Rotation(yaw=0)))
        # ego_vehicle_1 = world.spawn_actor(ego_vehicle_bp, carla.Transform(carla.Location(x=-13, y=80, z=2), carla.Rotation(yaw=90)))
        # at staright lane

        # ego_vehicle_1 = world.spawn_actor(ego_vehicle_bp, carla.Transform(carla.Location(x=-40, y=1, z=2), carla.Rotation(yaw=90)))
        # # at staright lane

        # ego_vehicle_2 = world.spawn_actor(ego_vehicle_bp, carla.Transform(carla.Location(x=-40, y=1, z=2), carla.Rotation(yaw=90)))
        # at start point front

        # set walker as spawn_actor
        # walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
        spawn_point = carla.Transform(carla.Location(x=-12.5, y=1.5, z=0.2),carla.Rotation())
        # spawn_point = carla.Transform(carla.Location(x=152, y=62.3, z=2),carla.Rotation(yaw=-90))
        pedestrain = world.spawn_actor(bp4, spawn_point)
        control = carla.WalkerControl()
        revert_flag = False
        # weather = carla.WeatherParameters(
        # cloudiness=80.0,
        # precipitation=30.0,
        # sun_altitude_angle=90.0)
        # world.set_weather(weather)

        #print(world.get_weather())
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
            # Debug Tool
            # logging.warning(pedestrain.get_location())
            # print(,control.direction.x,revert_flag)
        
    finally:
        # print('\ndestroying vehicles')
        # ego_vehicle_0.destroy()
        # ego_vehicle_1.destroy()
        # ego_vehicle_2.destroy()
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
