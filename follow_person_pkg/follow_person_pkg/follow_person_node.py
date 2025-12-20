import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

class FollowPerson(Node):
    def __init__(self):
        super().__init__('follow_person')
        
        # Subscribe to /person_position topic
        self.subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.person_callback, 10)
        
        # Navigation action client
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Ensure Navigation2 is running before sending a goal
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server is not available!')
            return
        
        self.last_goal = None
        self.safe_distance = 0.2  # Maintain 1 meter distance
        self.min_update_distance = 0.5  # Update if movement > 0.5m
        self.waiting_for_goal = False

        self.get_logger().info('Follow Person Node Initialized')

    def person_callback(self, msg):
        """Process new person position and send a navigation goal."""
        if self.waiting_for_goal:
            return  # Ignore new positions until current goal is reached

        self.get_logger().info(f'Received person position: x={msg.pose.position.x}, y={msg.pose.position.y}')

        # Calculate distance to last goal
        if self.last_goal:
            distance = math.sqrt(
                (msg.pose.position.x - self.last_goal.pose.position.x) ** 2 +
                (msg.pose.position.y - self.last_goal.pose.position.y) ** 2
            )
            if distance < self.min_update_distance:
                self.get_logger().info('Ignoring minor movement')
                return  
        else:
            distance = float('inf')  # Ensure first movement is always accepted

        # Compute new goal position with safe distance
        dx = msg.pose.position.x - self.safe_distance * math.cos(math.atan2(msg.pose.position.y, msg.pose.position.x))
        dy = msg.pose.position.y - self.safe_distance * math.sin(math.atan2(msg.pose.position.y, msg.pose.position.x))
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = dx
        goal_msg.pose.pose.position.y = dy
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self.calculate_orientation(dx, dy, msg.pose.position.x, msg.pose.position.y)

        self.get_logger().info(f'Sending goal: x={dx}, y={dy}, distance to person: {distance}')
        self.waiting_for_goal = True

        # Send goal asynchronously
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        # Save last goal for distance tracking
        self.last_goal = msg  

    def calculate_orientation(self, x1, y1, x2, y2):
        """Calculate the orientation quaternion to face the target."""
        angle = math.atan2(y2 - y1, x2 - x1)
        return PoseStamped().pose.orientation.__class__(x=0.0, y=0.0, z=math.sin(angle / 2), w=math.cos(angle / 2))

    def goal_response_callback(self, future):
        """Handles the response from Nav2 when a goal is sent."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            self.waiting_for_goal = False
            return

        self.get_logger().info('Goal accepted! Waiting for completion...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handles goal completion event."""
        self.get_logger().info('Goal reached!')
        self.waiting_for_goal = False  

def main(args=None):
    rclpy.init(args=args)
    node = FollowPerson()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



