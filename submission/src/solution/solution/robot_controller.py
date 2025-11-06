import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
from assessment_interfaces.msg import ItemList, ItemHolders
from auro_interfaces.srv import ItemRequest
from math import atan2
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
#from geometry_msgs.srv import PoseWithCovarianceStamped as SetInitialPoseSrv
from nav2_msgs.srv import SetInitialPose
import time

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller_node')
        self.declare_parameter('robot_name','robot1')
        self.declare_parameter('x',-3.5)
        self.declare_parameter('y',0.0)
        self.declare_parameter('yaw',0.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        
        self.get_logger().info(f"\n\nRobot ID: {self.robot_name}\n\n")

        time.sleep(7) # Wait to ensure all resources are fully loaded before proceeding
        
        self._client = ActionClient(self, NavigateToPose, '/'+self.robot_name+'/navigate_to_pose')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')
        # Subscriber for item detection (x, y coordinates)
        self.create_subscription(ItemList, '/'+self.robot_name+'/items', self.item_detection_callback, 1)
        self.create_subscription(ItemHolders, '/item_holders', self.item_holders_callback, 1)

        # Service client initialization for handling item pickup requests
        self.pick_up_req = self.create_client(ItemRequest, '/pick_up_item')
        
        # Wait for the service to become available, retrying every second
        while not self.pick_up_req.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('The /pick_up_item service is unavailable. Retrying in 1 second...')

        # Initialize the service client to set the robot's initial pose
        self.init_pose_srv = self.create_client(
            SetInitialPose, '/'+self.robot_name+'/set_initial_pose')

        # Wait for the set_initial_pose service to become available, retrying every 5 seconds
        while not self.init_pose_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('/'+self.robot_name+'/set_initial_pose service is is unavailable. Retrying in 5 seconds...')

        # Set the initial pose of the robot
        self.set_initial_pose_callback()
            
        # Publisher for controlling the robot's velocity
        self.velocity_publisher = self.create_publisher(Twist, '/'+self.robot_name+'/cmd_vel', 10)

        # Target state variables
        self.x_pixel = 0  # X-coordinate of the detected target in pixels
        self.y_pixel = 0 # Y-coordinate of the detected target in pixels
        self.diameter = 0  # Diameter of the detected target
        self.target_detected = False # Flag to indicate if a target is detected
        self.navigation = False # Flag for navigation status
        self.goal_flag = False # Flag for goal completion
        self.goal_active = False # Flag to indicate if a goal is active

        # Set predefined goal coordinates for each robot
        if self.robot_name == "robot1": 
            self.for_red = [-3.3, -2.3]
            self.for_green = [2.7, -2.3]
            self.for_blue = [2.3, 2.3]
            
        elif self.robot_name == "robot2": 
            self.for_red = [-3.5, 2.5]
            self.for_green = [2.6, -2.6]
            self.for_blue = [2.7, 2.7]
            
        elif self.robot_name == "robot3": 
            self.for_red = [-3.5, -2.6]
            self.for_green = [2.8, -2.4]
            self.for_blue = [2.4, 2.4]
            
        self.x = 0
        self.y = 0
        
        self.first_nav = False
        if self.robot_name == "robot2":
            self.x = 0.5
            self.y = -2.5
            self.goal_flag = False
            self.navigation = True
            self.goal_active=False
            self.first_nav = True
            time.sleep(5)
            
            #self.send_goal()
        
    def set_initial_pose_callback(self):
        '''
        Sends a request to set the initial pose of the robot.
        '''
        
        # Create a new request for setting the initial pose
        request = SetInitialPose.Request()

        # Set the frame of reference for the pose
        request.pose.header.frame_id = "map"
        
        request.pose.pose.pose.position.x = self.x
        request.pose.pose.pose.position.y = self.y
        request.pose.pose.pose.position.z = 0.0

        request.pose.pose.pose.orientation.x = 0.0
        request.pose.pose.pose.orientation.y = 0.0
        request.pose.pose.pose.orientation.z = 0.0
        request.pose.pose.pose.orientation.w = 1.0

        future = self.init_pose_srv.call_async(request)
        future.add_done_callback(self.init_pose_response_callback)

    def init_pose_response_callback(self,future):
        '''
        Callback to handle the response of the set_initial_pose request.
        '''
        
        try:
            response = future.result()
            self.get_logger().info('Initial pose set successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to set initial pose: {e}')
            

    def send_goal(self):
        '''
        Sends a navigation goal to the robot.
        '''
        
        # If a goal is already active or flagged, exit early
        if self.goal_flag:
            return
        
        # Set flags indicating a goal is being processed
        self.goal_flag = True
        self.goal_active = True

        # Ensure the navigation action server is available
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            return

        # Create and populate the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.x
        goal_msg.pose.pose.position.y = self.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending navigation goal to position: (x: {goal_msg.pose.pose.position.x}, y: {goal_msg.pose.pose.position.y})')
        
        send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''
        Callback function to handle the server's response to the navigation goal request.
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected by the server.')
            self.goal_active = False
            return
        
        self.get_logger().info('Navigation goal accepted by the server. Waiting for the result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # Placeholder for feedback
        pass

    def result_callback(self, future):
        '''
        Callback function to handle the result of the navigation goal.
        '''
        
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Successfully reached the goal!')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
        
        # Reset flags after goal execution
        self.goal_flag = False
        self.goal_active = False
        self.navigation = False
        self.first_nav = False
        
        # Prepare and send the offload request after navigation
        request = ItemRequest.Request()
        request.robot_id = self.robot_name
        future = self.offload_client.call_async(request)
        future.add_done_callback(self.offload_callback)

    def offload_callback(self, future):
        '''
        Callback function to handle the response from the offload service.
        '''
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Item successfully offloaded.')
                self.holding_item = False # Update the robot's state to indicate no longer holding the item
            else:
                self.get_logger().warn('Failed to offload the item.')

        except Exception as e:
            self.get_logger().error(f'Error occurred while calling the offload service: {e}')
            
    def item_holders_callback(self, msg):
        for item in msg.data:
            if item.robot_id == self.robot_name:
                if self.first_nav == False:
                    self.navigation = item.holding_item
                if item.item_colour == 'RED':
                    self.x = self.for_red[0]
                    self.y = self.for_red[1]
                elif item.item_colour == 'BLUE':
                    self.x = self.for_blue[0]
                    self.y = self.for_blue[1]
                elif item.item_colour == 'GREEN':
                    self.x = self.for_green[0]
                    self.y = self.for_green[1]
                    

    def item_detection_callback(self, msg):
        '''
        Callback function to process item detection data and identify the largest detected item.
        '''
        try:
            largest_item = max(msg.data, key=lambda item: item.diameter)
            self.x_pixel = largest_item.x
            self.y_pixel = largest_item.y
            self.diameter = largest_item.diameter
            self.target_detected = True # Mark that a target has been detected
        except ValueError:
            self.target_detected = False

    def find_goal(self):
        '''
        Main loop to follow the target by deciding between navigation, moving to the target, or searching for a target.
        '''
        while rclpy.ok():
            if not self.navigation and not self.goal_active:
                if self.target_detected:
                    self.move_to_target() # Move towards the detected target
                else:
                    self.search_for_target() # Search for a target if none is detected
            elif self.navigation:
                self.initiate_navigation() # Initiate navigation process if in navigation mode

            rclpy.spin_once(self, timeout_sec=0.1)

    def move_to_target(self):
        '''
        Moves the robot towards the detected target based on its position and size.
        '''
        
        twist = Twist()
        k_angular = 0.005
        k_linear = 0.35

        if self.diameter > 300:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)

            # Send a request to pick up the item
            request = ItemRequest.Request()
            request.robot_id = self.robot_name
            future = self.pick_up_req.call_async(request)
            future.add_done_callback(self.pick_callback)
        else:
            twist.linear.x = max(k_linear * (1 - self.diameter / 300), 0.1)
            twist.angular.z = k_angular * self.x_pixel
            self.velocity_publisher.publish(twist)

    def search_for_target(self):
        twist = Twist()
        twist.angular.z = 0.5
        if self.robot_name == 'robot2':
            twist.angular.z = -0.5
        self.velocity_publisher.publish(twist)

    def initiate_navigation(self):
        if not self.goal_flag:
            self.get_logger().info(f"\n\nRobot ID: {self.robot_name}\n\n")

            self.send_goal()

    def pick_callback(self, future):
        '''
        Callback function to handle the response from the pick-up service.
    '''
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Item successfully picked up.')
            else:
                self.get_logger().warn('Failed to pick up the item.')
        except Exception as e:
            self.get_logger().error(f'Error calling pick-up service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    node.find_goal()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
