#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState as JointStateMoveIt
from dynamixel_msgs.msg import JointState as JointStateDynamixel

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('edy_joint_state_publisher')

        rate = 50 # 20Hz
        r = rospy.Rate(rate)

        self.l_joint1name = ''
        self.l_joint1current_pos = 0.0
        self.l_joint1velocity = 0.0
        self.l_joint1load = 0.0

	self.l_joint2name = ''
        self.l_joint2current_pos = 0.0
        self.l_joint2velocity = 0.0
        self.l_joint2load = 0.0

        self.l_joint3name = ''
        self.l_joint3current_pos = 0.0
        self.l_joint3velocity = 0.0
        self.l_joint3load = 0.0

	self.l_joint4name = ''
        self.l_joint4current_pos = 0.0
        self.l_joint4velocity = 0.0
        self.l_joint4load = 0.0

        self.l_joint5name = ''
        self.l_joint5current_pos = 0.0
        self.l_joint5velocity = 0.0
        self.l_joint5load = 0.0

	self.l_joint6name = ''
        self.l_joint6current_pos = 0.0
        self.l_joint6velocity = 0.0
        self.l_joint6load = 0.0

        self.l_joint7name = ''
        self.l_joint7current_pos = 0.0
        self.l_joint7velocity = 0.0
        self.l_joint7load = 0.0

	self.l_joint8name = ''
        self.l_joint8current_pos = 0.0
        self.l_joint8velocity = 0.0
        self.l_joint8load = 0.0
     

        # Start controller state subscribers
        rospy.Subscriber('/edy_position_controller_l_1/state', JointStateDynamixel, self.controller_state_handler_l_1)
	rospy.Subscriber('/edy_position_controller_l_2/state', JointStateDynamixel, self.controller_state_handler_l_2)
        rospy.Subscriber('/edy_position_controller_l_3/state', JointStateDynamixel, self.controller_state_handler_l_3)
	rospy.Subscriber('/edy_position_controller_l_4/state', JointStateDynamixel, self.controller_state_handler_l_4)
        rospy.Subscriber('/edy_position_controller_l_5/state', JointStateDynamixel, self.controller_state_handler_l_5)
	rospy.Subscriber('/edy_position_controller_l_6/state', JointStateDynamixel, self.controller_state_handler_l_6)
        rospy.Subscriber('/edy_position_controller_l_7/state', JointStateDynamixel, self.controller_state_handler_l_7)
	rospy.Subscriber('/edy_position_controller_l_8/state', JointStateDynamixel, self.controller_state_handler_l_8)

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStateMoveIt)

        rospy.loginfo("Publishing joint_state at " + str(rate) + "Hz")

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def controller_state_handler_l_1(self, msg):
        self.l_joint1name = msg.name
        self.l_joint1current_pos = msg.current_pos
        self.l_joint1velocity = msg.velocity
        self.l_joint1load = msg.load

    def controller_state_handler_l_2(self, msg):
        self.l_joint2name = msg.name
        self.l_joint2current_pos = msg.current_pos
        self.l_joint2velocity = msg.velocity
        self.l_joint2load = msg.load

    def controller_state_handler_l_3(self, msg):
        self.l_joint3name = msg.name
        self.l_joint3current_pos = msg.current_pos
        self.l_joint3velocity = msg.velocity
        self.l_joint3load = msg.load

    def controller_state_handler_l_4(self, msg):
        self.l_joint4name = msg.name
        self.l_joint4current_pos = msg.current_pos
        self.l_joint4velocity = msg.velocity
        self.l_joint4load = msg.load

    def controller_state_handler_l_5(self, msg):
        self.l_joint5name = msg.name
        self.l_joint5current_pos = msg.current_pos
        self.l_joint5velocity = msg.velocity
        self.l_joint5load = msg.load

    def controller_state_handler_l_6(self, msg):
        self.l_joint6name = msg.name
        self.l_joint6current_pos = msg.current_pos
        self.l_joint6velocity = msg.velocity
        self.l_joint6load = msg.load

    def controller_state_handler_l_7(self, msg):
        self.l_joint7name = msg.name
        self.l_joint7current_pos = msg.current_pos
        self.l_joint7velocity = msg.velocity
        self.l_joint7load = msg.load

    def controller_state_handler_l_8(self, msg):
        self.l_joint8name = msg.name
        self.l_joint8current_pos = msg.current_pos
        self.l_joint8velocity = msg.velocity
        self.l_joint8load = msg.load


    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStateMoveIt()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        msg.name.append(self.l_joint1name)
        msg.position.append(self.l_joint1current_pos)
        msg.velocity.append(self.l_joint1velocity)
        msg.effort.append(self.l_joint1load)

        msg.name.append(self.l_joint2name)
        msg.position.append(self.l_joint2current_pos)
        msg.velocity.append(self.l_joint2velocity)
        msg.effort.append(self.l_joint2load)

        msg.name.append(self.l_joint3name)
        msg.position.append(self.l_joint3current_pos)
        msg.velocity.append(self.l_joint3velocity)
        msg.effort.append(self.l_joint3load)

        msg.name.append(self.l_joint4name)
        msg.position.append(self.l_joint4current_pos)
        msg.velocity.append(self.l_joint4velocity)
        msg.effort.append(self.l_joint4load)

        msg.name.append(self.l_joint5name)
        msg.position.append(self.l_joint5current_pos)
        msg.velocity.append(self.l_joint5velocity)
        msg.effort.append(self.l_joint5load)

        msg.name.append(self.l_joint6name)
        msg.position.append(self.l_joint6current_pos)
        msg.velocity.append(self.l_joint6velocity)
        msg.effort.append(self.l_joint6load)

        msg.name.append(self.l_joint7name)
        msg.position.append(self.l_joint7current_pos)
        msg.velocity.append(self.l_joint7velocity)
        msg.effort.append(self.l_joint7load)

        msg.name.append(self.l_joint8name)
        msg.position.append(self.l_joint8current_pos)
        msg.velocity.append(self.l_joint8velocity)
        msg.effort.append(self.l_joint8load)

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        self.joint_states_pub.publish(msg)

if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
