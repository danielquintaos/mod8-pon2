import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener
from tf2_ros.transformations import euler_from_quaternion

class TurtleBot3Navigator(Node):
    def __init__(self):
        super().__init__('turtlebot3_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.current_pose = PoseStamped()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        self.current_pose.pose = msg.pose.pose

    def get_current_orientation(self):
        orientation_q = self.current_pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self.send_goal_async(goal_msg)

    def send_goal_async(self, goal_msg):
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Aqui você pode processar o feedback, como a posição atual

    def navigate_to(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = sin(theta / 2.0)
        pose.pose.orientation.w = cos(theta / 2.0)

        self.send_goal(pose)

def main(args=None):
    rclpy.init(args=args)
    navigator = TurtleBot3Navigator()

    try:
        # Exemplo: Navegar para a posição (x=2.0, y=1.0, theta=0.0)
        navigator.navigate_to(2.0, 1.0, 0.0)

        # Executa o executor para processar callbacks
        navigator.executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

