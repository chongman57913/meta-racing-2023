#!/usr/bin/python
import carla
import time



def main():
    actor_list = []

    try:

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()

        actor_list = world.get_actors()

        traffic_light_list = []

        for i in actor_list:
            if i.type_id.startswith('traffic.traffic_light'):
                traffic_light_list.append(i)
        print(traffic_light_list)



        #红绿灯是按顺序添加的
        # how to judge traffic_light_list 朝向
        # 朝向带有A or C or Ds or B字母墙体的红绿灯
        print('Light_0:',traffic_light_list[0].get_transform().rotation.yaw)
        print('Light_1:',traffic_light_list[1].get_transform().rotation.yaw)
        print('Light_2:',traffic_light_list[2].get_transform().rotation.yaw)
        print('Light_3:',traffic_light_list[3].get_transform().rotation.yaw)

        print('Light_0:',traffic_light_list[0].get_location().x, traffic_light_list[0].get_location().y, traffic_light_list[0].get_location().z)
        print('Light_1:',traffic_light_list[1].get_location().x, traffic_light_list[1].get_location().y, traffic_light_list[1].get_location().z)
        print('Light_2:',traffic_light_list[2].get_location().x, traffic_light_list[2].get_location().y, traffic_light_list[2].get_location().z)
        print('Light_3:',traffic_light_list[3].get_location().x, traffic_light_list[3].get_location().y, traffic_light_list[3].get_location().z)

        # 自定义四个红绿灯状态(2号和3号只有红灯和绿灯，但是carla强制要求红绿灯有Red，Yellow，Green三个状态，所以这里修改为2号3号设置为Yellow时也显示红灯)

        traffic_light_states = [carla.TrafficLightState.Red,
                                carla.TrafficLightState.Red,
                                carla.TrafficLightState.Green,
                                carla.TrafficLightState.Red]

        traffic_light_green_times = [10.0, 10.0, 10.0, 10.0]
        traffic_light_yellow_times = [5.0, 5.0, 5.0, 5.0]
        traffic_light_red_times = [15.0, 15.0, 15.0, 15.0]
        traffic_light_states = [carla.TrafficLightState.Red, carla.TrafficLightState.Red, carla.TrafficLightState.Green, carla.TrafficLightState.Red]
        world.freeze_all_traffic_lights(True)
        print("Lights frozen.")

        while True:
            for idx, traffic_light in enumerate(traffic_light_list):
                traffic_light.set_state(traffic_light_states[idx])
                traffic_light.set_green_time(traffic_light_green_times[idx])
                traffic_light.set_yellow_time(traffic_light_yellow_times[idx])
                traffic_light.set_red_time(traffic_light_red_times[idx])

            world.wait_for_tick()
            world.freeze_all_traffic_lights(False)
            print("Lights changing...")
            world.freeze_all_traffic_lights(True)
            time.sleep(0.1)

        # traffic_light_list[0].set_state(carla.TrafficLightState.Red)
        # traffic_light_list[1].set_state(carla.TrafficLightState.Red)
        # traffic_light_list[2].set_state(carla.TrafficLightState.Green)
        # traffic_light_list[3].set_state(carla.TrafficLightState.Red)

        # 停止所有红绿灯变化，防止设置完后就变灯
        # world.freeze_all_traffic_lights(True)
        # print('Lights frozen ...')

        # 让红绿灯重新开始变化
        # world.freeze_all_traffic_lights(False)
        # print('Lights changing ...')

        # for i in traffic_light_list:
        #     print (i)
        #     print (i.is_frozen())

    finally:
        
        print('done.')


if __name__ == '__main__':

    main()
