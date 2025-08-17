import rclpy
from rclpy.node import Node
from aubo_msgs.srv import SetPoseStampedGoal

class ExecutorServiceNode(Node):
    def __init__(self):
        super().__init__('executor_service_node')
        self.srv = self.create_client(SetPoseStampedGoal, 'set_pose_stamped_goal')

    def call_test(self):
        # Example logic: always succeed
        request = SetPoseStampedGoal.Request()
        request.pose_stamped.header.frame_id = 'base_link'
        request.pose_stamped.pose.position.x = 0.383
        request.pose_stamped.pose.position.y = 0.374
        request.pose_stamped.pose.position.z = 0.502
        request.pose_stamped.pose.orientation.x = 0.201
        request.pose_stamped.pose.orientation.y = 0.678
        request.pose_stamped.pose.orientation.z = 0.678
        request.pose_stamped.pose.orientation.w = 0.201
        request.speed_factor = 0.5
        response = self.srv.call(request)
        if response.success:
            self.get_logger().info('Service called: set_pose_stamped_goal')
        self.get_logger().info(f'Service response: {response.success}, {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ExecutorServiceNode()
    node.call_test()

if __name__ == '__main__':
    main()