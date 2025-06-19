#!/usr/bin/env python3
from mujoco_msgs.msg import MujocoContacts
import rclpy.duration
from visualization_msgs.msg import Marker, MeshFile
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy






class MujocoContactPublisher(Node):

    def __init__(self):
            
            super().__init__('mujoco_contact_publisher')
            self.publisher = self.create_publisher(Marker, '/visualization/mujoco_contacts', 10)
            qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
            self.sub = self.create_subscription(MujocoContacts, '/simulation/contacts', self.callback, qos_profile)        

            self.declare_parameter("object_name", "base_collisionbox")
            self.object_name = self.get_parameter("object_name").value    

    def callback(self, msg):
           
        for c in msg.contacts:
              
            if c.object1_name == self.object_name or c.object2_name == self.object_name:
                
                now = self.get_clock().now().to_msg()
                marker = Marker()
                marker.header.frame_id = 'world'  # Make sure RViz has a "map" frame
                marker.header.stamp = now
                marker.ns = "mujoco_contacts"
                marker.id = 50  
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

                marker.scale.x = 0.04  # Line width
                marker.scale.y = 0.04
                marker.scale.z = 0.04
                marker.color.a = 1.0 
                marker.color.r = 0.2
                marker.color.g = 0.2
                marker.color.b = 0.8
                marker.pose.position = c.position

                self.publisher.publish(marker)
                    


         




















def main(args=None):
    rclpy.init(args=args)

    node = MujocoContactPublisher()
    rclpy.spin(node)

	# initial_pose_publisher.destroy_node()
	# rclpy.shutdown()

if __name__ == '__main__':
	main()