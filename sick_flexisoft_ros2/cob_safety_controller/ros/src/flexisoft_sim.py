#!/usr/bin/env python3
import cob_common
import rclpy
from cob_common.cob_msgs import EmergencyStopState

def flexisoft_sim():
    rclpy.init()
    node = rclpy.create_node('cob_flexisoft_sim')

    # emergency_stop topic
    pub_em_stop = node.create_publisher(EmergencyStopState, 'emergency_stop_state', 1)
    msg_em = EmergencyStopState()
    msg_em.emergency_button_stop = False
    msg_em.scanner_stop = False
    msg_em.emergency_state = 0

    while rclpy.ok():
        pub_em_stop.publish(msg_em)
        rclpy.spin_once(node)
        node.get_logger().info("Publishing emergency stop state")
        rclpy.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        flexisoft_sim()
    except KeyboardInterrupt:
        print("Interrupted")
        
#!/usr/bin/env python

# import rospy
# from cob_msgs.msg import EmergencyStopState

# def flexisoft_sim():
# 	rospy.init_node('cob_flexisoft_sim')

# 	# emergency_stop topic
# 	pub_em_stop = rospy.Publisher('emergency_stop_state', EmergencyStopState, queue_size=1)
# 	msg_em = EmergencyStopState()
# 	msg_em.emergency_button_stop = False
# 	msg_em.scanner_stop = False
# 	msg_em.emergency_state = 0


# 	while not rospy.is_shutdown():
# 		pub_em_stop.publish(msg_em)
# 		rospy.sleep(1.0)

# if __name__ == '__main__':
# 	try:
# 		flexisoft_sim()
# 	except rospy.ROSInterruptException:
# 		print("Interupted")
# 		pass
