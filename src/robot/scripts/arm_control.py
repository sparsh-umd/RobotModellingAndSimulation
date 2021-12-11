#!/usr/bin/env python

import rospy
from rospy.timer import sleep
from std_msgs.msg import Float64

def talker():
    pub_arm_base = rospy.Publisher('/robot/arm_base_controller/command', Float64, queue_size=10)
    pub_arm_middle = rospy.Publisher('/robot/arm_middle_controller/command', Float64, queue_size=10)
    pub_arm_end = rospy.Publisher('/robot/arm_end_controller/command', Float64, queue_size=10)
    pub_left_end_eff = rospy.Publisher('/robot/left_end_eff_controller/command', Float64, queue_size=10)
    pub_right_end_eff = rospy.Publisher('/robot/right_end_eff_controller/command', Float64, queue_size=10)

    publishers = [
        pub_arm_base,
        pub_arm_middle,
        pub_arm_end,
        pub_left_end_eff,
        pub_right_end_eff
    ]

    home_pos = False

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    angle_input = None
    publisher_index = None
    while not rospy.is_shutdown():
        if not home_pos:
            sleep(1)
            print('Going to home position!')
            for i in range(len(publishers)):
                publishers[i].publish(0.0)
            home_pos = True
        publisher_index = int(input('Choose publisher index(1-4): '))
        if publisher_index < 1 or publisher_index > 4:
            print('Publisher index should be between 1 and 4. Please retry.')
            continue
        publisher_index -= 1
        angle_input = float(input('Enter desired angle: '))

        if publisher_index == 3 or publisher_index == 4:
            publishers[3].publish(angle_input)
            publishers[4].publish(angle_input)
        else:
            publishers[publisher_index].publish(angle_input)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
