#!/usr/bin/env python3
import rclpy.duration
from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node


class ModelPublisher(Node):

    def __init__(self):
            super().__init__('model_publisher')
            self.model_publisher = self.create_publisher(Marker, '/visualization/collision_model', 10)

            self.model_timer = self.create_timer(0.1, self.model_callback)

            self.declare_parameter("model_alpha", 1.0)
            self.model_alpha = self.get_parameter("model_alpha").value

            self.model = Marker()
            self.model.header.frame_id = "base"
            self.model.ns = "model"
            self.model.id = 10  # Unique ID per circle
            self.model.type = Marker.MESH_RESOURCE
            self.model.mesh_resource = "package://haptiquad_contacts/assets/anymal_base_collisionbox_united.stl"
            self.model.action = Marker.ADD
            self.model.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            self.model.scale.x = 1.0
            self.model.scale.y = 1.0
            self.model.scale.z = 1.0
            self.model.color.a = self.model_alpha
            self.model.color.r = 0.5
            self.model.color.b = 0.2

            

    def model_callback(self):

        self.model.header.stamp = self.get_clock().now().to_msg()
        self.model_publisher.publish(self.model)                 






def main(args=None):
    rclpy.init(args=args)

    node = ModelPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
	main()