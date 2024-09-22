import rclpy
from rclpy.node import Node

from ex09.srv import FullNameSumService


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FullNameSumService, 'concat_three_strings', self.concat_three_strings_callback)

    def concat_three_strings_callback(self, request, response):
        response.full = request.surname + ' ' + request.name + ' ' + request.first_name
        self.get_logger().info('Incoming request\n %s %s %s' % (request.surname, request.name, request.first_name))
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()