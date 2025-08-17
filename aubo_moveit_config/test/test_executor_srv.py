import rclpy
from rclpy.node import Node
from aubo_msgs.srv import SetPoseStampedGoal

class ExecutorServiceNode(Node):
    def __init__(self):
        super().__init__('executor_service_test')
        self.srv = self.create_client(SetPoseStampedGoal, 'set_end_effector_pose')

    def call_test(self):
        # Example logic: always succeed
        request = SetPoseStampedGoal.Request()
        request.goal.header.frame_id = 'base_link'
        request.goal.pose.position.x = 0.383
        request.goal.pose.position.y = 0.374
        request.goal.pose.position.z = 0.502
        request.goal.pose.orientation.x = 0.201
        request.goal.pose.orientation.y = 0.678
        request.goal.pose.orientation.z = 0.678
        request.goal.pose.orientation.w = 0.201
        request.speed_factor = 0.5
        self.get_logger().info('Service called: set_end_effector_pose')
        response = self.srv.call(request)
        self.get_logger().info(f'Service response: {response.success}, {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ExecutorServiceNode()
    node.call_test()

if __name__ == '__main__':
    main()