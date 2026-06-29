#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RadiometricProcessingNode(Node):
    """Online radiometric processing — replace message types and callbacks as needed."""

    def __init__(self) -> None:
        super().__init__('radiometric_processing')

        self.declare_parameter('input_topic', '/raw_image')
        self.declare_parameter('output_topic', '/radiometric_image')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._pub = self.create_publisher(String, output_topic, 10)
        self._sub = self.create_subscription(
            String, input_topic, self._on_input, 10
        )

        self.get_logger().info(
            f'Radiometric processing node ready: sub={input_topic} pub={output_topic}'
        )

    def _on_input(self, msg: String) -> None:
        # TODO: radiometric processing logic
        out = String()
        out.data = msg.data
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadiometricProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
