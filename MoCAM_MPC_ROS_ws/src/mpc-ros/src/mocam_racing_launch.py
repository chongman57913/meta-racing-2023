#! /usr/bin/env python3
import rospy
from mocam_racing import mpc_core
from mpc.mpc_path_tracking import mpc_path_tracking

# def record_time():
    # rospy.init_node('my_node', anonymous=True)
    # mpc_path_tracking=mpc_path_tracking()
    # mpc_path_tracking.controller()
    # time_flag=mpc_path_tracking.flag
    
    # rate = rospy.Rate(1000) # 1000 Hz
    # start_time = rospy.get_time()
    
    # if time_flag:
    #     record_time=rospy.get_time()-start_time
    #     # pub.publish(0.0)
    # print("finished_time:",record_time)    

if __name__ == '__main__':
    mp = mpc_core() 
    mp.cal_vel()
    
    # mpc_path_tracking=mpc_path_tracking()
    # mpc_path_tracking.controller()
    # time_flag=mpc_path_tracking.flag
    # start_time=rospy.get_time()
    # print(start_time)
    # print(time_flag)
    
    # if time_flag:
    #     record_time=rospy.get_time()-start_time
    #     print("finished_time",record_time)
    # else:
    #     print("time_flag is False")

    # try:
        # record_time()
    # except rospy.ROSInterruptException:
        # pass

