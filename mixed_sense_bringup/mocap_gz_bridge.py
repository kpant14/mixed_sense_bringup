#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import PoseStamped

class MocapGzBridge(Node):
    """
    Move a target entity (gazebo model) along the path tracked by the Qualisys Motion Capture System 
    """
    def __init__(self, target_name="drone162") -> None:
        self.target_name = target_name
        super().__init__("mocap_gz_bridge")
        
        self.declare_parameter('gz_entity', 'x500_1')
        self.declare_parameter('gz_world_name', 'AbuDhabi')
        self.declare_parameter('mocap_rigid_body', 'drone162')
    
        # Client for setting the entity state in Gazebo
        self.gz_world_name = self.get_parameter('gz_world_name').get_parameter_value().string_value
        self.get_logger().info(f'World Name: {self.gz_world_name}')

        self.client = self.create_client(SetEntityPose, f"/world/{self.gz_world_name}/set_pose")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not available, waiting...')
        
        
        # Subscription to the pose msg from qualisys_mocap node
        self.rigid_body = self.get_parameter('mocap_rigid_body').get_parameter_value().string_value
        self.get_logger().info(f'Rigid body: {self.rigid_body}')

        self.mocap_pose_sub_ = self.create_subscription(
            PoseStamped,
            f'/{self.rigid_body}/pose',
            self.pose_cb,
            10
        )
        # Entity information of the model in Gazebo
        self.gz_entity = self.get_parameter('gz_entity').get_parameter_value().string_value
        self.get_logger().info(f'Gazebo body: {self.gz_entity}')

        self.entity = Entity()
        self.entity.name = self.gz_entity
        self.request = SetEntityPose.Request()

        # Timer for periodically setting the state of the entity inside Gazebo
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.drone_pose=None
    
    # Callback for handling the pose msg from qualisys_mocap 
    def pose_cb(self, msg: PoseStamped):
        self.drone_pose = msg
        
    # Callback for handling the pose service 
    def cmdloop_callback(self):
        if self.drone_pose is not None:
            if self.drone_pose.pose is not None:
                self.request.entity = self.entity
                self.request.pose = self.drone_pose.pose
                future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    mocap_gz_bridge = MocapGzBridge()
    executor.add_node(mocap_gz_bridge)
    executor.spin()

    try:
        rclpy.shutdown()
    except Exception():
        pass

if __name__ == '__main__':
    main()
