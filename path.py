import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math

class PathController(Node):
    def __init__(self):
        super().__init__('path_controller')

        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.timer = self.create_timer(0.5, self.publish_path)

        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_pose = None

        self.min_distance = 0.1
        self.min_angle = 0.1
        self.max_path_length = 1370


        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def calculate_distance(self, pose1, pose2):
        """计算两个位姿之间的直线距离"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_angle_difference(self, pose1, pose2):
        """计算两个位姿之间的角度差"""
        # 将四元数转换为欧拉角
        q1 = [pose1.orientation.x, pose1.orientation.y, 
              pose1.orientation.z, pose1.orientation.w]
        q2 = [pose2.orientation.x, pose2.orientation.y, 
              pose2.orientation.z, pose2.orientation.w]
        
        # 计算偏航角(yaw)的差值
        euler1 = tf_transformations.euler_from_quaternion(q1)
        euler2 = tf_transformations.euler_from_quaternion(q2)
        
        return abs(euler1[2] - euler2[2])  # 返回偏航角的绝对差值

    def odom_callback(self, msg):
        """里程计回调函数，记录路径点"""
        current_pose = msg.pose.pose
        
        # 如果是第一个点，直接添加
        if self.last_pose is None:
            self.add_pose_to_path(current_pose, msg.header.stamp)
            self.last_pose = current_pose
            return
        
        # 计算与上一个点的距离和角度差
        distance = self.calculate_distance(current_pose, self.last_pose)
        angle_diff = self.calculate_angle_difference(current_pose, self.last_pose)
        
        # 只有当移动距离或角度变化超过阈值时才记录新点
        if distance > self.min_distance or angle_diff > self.min_angle:
            self.add_pose_to_path(current_pose, msg.header.stamp)
            self.last_pose = current_pose

    def add_pose_to_path(self, pose, stamp):
        """将位姿添加到路径中"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = pose
        
        self.path.poses.append(pose_stamped)
        
        # 限制路径长度，避免内存无限增长
        if len(self.path.poses) > self.max_path_length:
            self.path.poses.pop(0)  # 移除最旧的点
        
        # 更新路径的时间戳
        self.path.header.stamp = self.get_clock().now().to_msg()

    def publish_path(self):
        """发布路径消息"""
        if len(self.path.poses) > 0:
            self.path_pub.publish(self.path)

def main(args=None):
    

    rclpy.init(args=args)
    controller = PathController()
    rclpy.spin(controller)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()