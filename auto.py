import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import tf_transformations
from enum import Enum

class RobotState(Enum):
    NORMAL = 0
    OBSTACLE_AVOIDANCE = 1
    EMERGENCY_STOP =2

class ClassicVFHController(Node):
    def __init__(self):
        super().__init__('classic_vfh_controller')
        
        # 创建发布器
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建订阅器
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 创建定时器
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # 状态变量
        self.state = RobotState.NORMAL
        self.laser_data = None
        self.current_yaw = 0.0
        self.last_yaw = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_best_direction = 0.0
        
        # VFH参数（根据论文）
        self.sector_angle = 5  # 5度扇区
        self.num_sectors = 72  # 360/5 = 72个扇区
        self.window_size = 80  # 活动窗口大小（个）
        self.cell_size = 0.05  # 活动单元格长度 (m)
        
        # 障碍向量参数（论文公式2）
        self.a = 10.0  # 常数a
        self.d_max = math.sqrt(2) * (self.window_size - 1) / 2 * self.cell_size
        self.b = self.a / self.d_max  # 常数b
        
        # 控制参数
        self.normal_speed = 0.8
        self.max_linear_speed = 0.5
        self.min_linear_speed = 0.04
        self.max_angular_speed = 2.094
        self.h_m = 0.0 #最大极坐标障碍密度

        self.mu1 = 2 #目标方向cost
        self.mu2 = 3 #上一次选择方向cost

        #安全距离
        self.Width_min = 0.76 + 2 * 0.3
        
        # 阈值参数
        self.threshold = 7.5  # 需要根据实际情况调整
        
        # 目标方向
        self.target_direction = 0

        # 当前速度和转向速率
        self.current_speed = 0.0
        self.current_steering_rate = 0.0

        self.fai = 0.0
        self.y = [0, 0.1, 0, 0, 0]
        self.omega = 0.0 #混沌路径规划角度
        
        # 调试信息
        self.get_logger().info("Classic VFH Controller initialized")

    def laser_callback(self, msg):
        self.laser_data = msg

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        # 计算转向速率
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        current_yaw = euler[2]
        
        if time_diff > 0:
            self.current_steering_rate = (current_yaw - self.last_yaw) / time_diff
        else:
            self.current_steering_rate = 0.0
            
        self.last_yaw = current_yaw
        self.current_yaw = current_yaw
        self.last_cmd_time = current_time

    def build_polar_histogram(self, ranges, angle_increment):
        """构建极坐标直方图（论文第一阶段）"""
        # 初始化极坐标直方图
        polar_histogram = np.zeros(self.num_sectors)
        
        if ranges is None:
            return polar_histogram
            
        # 计算活动窗口的范围（以机器人为中心）
        #half_window = (self.window_size - 1) / 2 * self.cell_size
        half_window = self.window_size * self.cell_size / 2
        for i, distance in enumerate(ranges):
            if math.isnan(distance) or distance == 0.0 or distance > 30.0:
                continue
                
            # 计算障碍物在激光雷达坐标系中的位置
            angle = i * angle_increment
            obstacle_x = distance * math.cos(angle)
            obstacle_y = distance * math.sin(angle)
            
            # 检查是否在活动窗口内
            if abs(distance) > half_window:
                continue
                
            # 障碍物到机器人的距离
            d = distance
            
            # 计算方向（从障碍物指向机器人）(0 <= beta <= 2pi)
            beta = math.atan2(-obstacle_y, -obstacle_x)
            if beta < 0:
                beta += 2 * math.pi
                
            # 确定扇区索引
            sector_index = int(beta / math.radians(self.sector_angle)) % self.num_sectors
            
            # 计算障碍向量大小（论文公式2）
            # 这里简化：c_i,j* = 1（每次检测都视为确定障碍）
            m = 1.0 * (self.a - self.b * d)
            if m < 0:
                m = 0
                
            # 累加到极坐标直方图
            polar_histogram[sector_index] += m
            
        return polar_histogram

    def smooth_polar_histogram(self, histogram):
        """平滑极坐标直方图（论文公式5）"""
        l = 5  # 平滑参数
        n = len(histogram)
        smoothed = np.zeros(n)
        self.h_m = 0.0
        
        for k in range(n):
            numerator = 0
            for i in range(-l, l+1):
                index = (k + i) % n
                weight = l - abs(i) + 1
                numerator += weight * histogram[index]
            smoothed[k] = numerator / (2*l + 1)
            #self.get_logger().info(f"{k}'s h_c is {smoothed[k]}") #调试
            if smoothed[k] > self.h_m:
                self.h_m = smoothed[k]
                #self.get_logger().info(f"h_m is {self.h_m}") #调试

            
        return smoothed

    def find_candidate_valleys(self, histogram, threshold):
        """寻找候选山谷"""
        valleys = []
        n = len(histogram)
        
        i = 0
        while i < n:
            # 寻找连续的低密度扇区
            if histogram[i] < threshold:
                start = i
                # 计算连续低密度扇区的长度
                while i < n and histogram[i] < threshold:
                    i += 1
                end = i - 1
                
                # 计算山谷大小
                size = (end - start + 1) % n
                if size > 0:
                    valleys.append({
                        'start': start,
                        'end': end,
                        'size': size,
                        'center': (start + end) // 2
                    })
            else:
                i += 1
                
        # 处理环形边界
        if len(valleys) > 1 and valleys[0]['start'] == 0 and valleys[-1]['end'] == n-1:
            # 合并首尾山谷
            merged_valley = {
                'start': valleys[-1]['start'],
                'end': valleys[0]['end'] + n,
                'size': valleys[-1]['size'] + valleys[0]['size'],
                'center': (valleys[-1]['center'] + valleys[0]['center']) % n // 2 
            }
            valleys = [merged_valley] + valleys[1:-1]
            
        return valleys

    def select_best_direction(self, valleys, target_sector):
        """选择最佳方向（论文第二阶段）"""
        if not valleys:
            return None, None
        '''    
        # 找到范围最大的山谷
        best_size = 0
        best_valley = None
        '''
        
        #找到最接近目标方向的山谷
        min_cost = float('inf')
        best_valley = None
        
        
        for valley in valleys:
            '''
            if valley['size'] > best_size:
                best_valley = valley
            ''' 
            # 计算山谷中心与目标方向的夹角
            angle_diff = min(
                abs(valley['center'] - target_sector),
                self.num_sectors - abs(valley['center'] - target_sector)
            )
            
            angle_last_diff = abs(valley['center'] - self.last_best_direction)

            cost = self.mu1 * angle_diff + self.mu2 * angle_last_diff
            if cost < min_cost:
                min_cost = cost
                best_valley = valley
              
        if best_valley is None:
            return None, None
            
        # 根据山谷宽度选择方向（论文图8策略）
        s_max = 18  # 宽山谷阈值(90度)
        
        if best_valley['size'] > s_max:
            # 宽山谷：选择靠近目标方向or上一次最佳方向的方向

            k_ns = best_valley['start']
            k_fs = (k_ns + s_max) % self.num_sectors
            best_directions = ((k_ns + k_fs) % self.num_sectors) // 2
            k_ne = best_valley['end']
            k_fe = (k_ne - s_max) % self.num_sectors
            best_directione = ((k_ne + k_fe) % self.num_sectors) // 2

            angle_diffs = min(
                abs(target_sector - best_directions),
                self.num_sectors - abs(target_sector - best_directions)
            )

            angle_diffe = min(
                abs(target_sector - best_directione),
                self.num_sectors - abs(target_sector - best_directione)
            )

            if angle_diffe > angle_diffs:
                best_direction = best_directions
            else:
                best_direction = best_directione
        else:
            # 窄山谷：选择山谷中心
            best_direction = best_valley['center']
        
        self.last_best_direction = best_direction
            
        return best_direction, best_valley
    
    def calculate_dynamic_speed(self, current_direction_sector, smoothed_histogram, steering_rate):
        """计算动态速度（论文第IV-D节，公式6-8）"""
        # 获取当前方向的极坐标障碍密度
        h_c = smoothed_histogram[current_direction_sector]
        
        # 限制h_c不超过h_m（论文公式7）
        h_c = min(h_c, self.h_m)
        
        # 计算基于障碍密度的速度（论文公式6）
        v_prime = self.max_linear_speed * (1 - h_c / self.h_m)
        
        # 计算基于转向速率的最终速度（论文公式8）
        steering_ratio = abs(steering_rate) / self.max_angular_speed
        v = v_prime * (1 - steering_ratio) + self.min_linear_speed
        
        # 确保速度在合理范围内
        v = max(self.min_linear_speed, min(self.max_linear_speed, v))
        
        return v


    def classic_vfh_algorithm(self, ranges, angle_increment, current_steering_rate):
        """经典VFH算法实现"""
        if ranges is None:
            return 0.0, self.max_linear_speed
            
        # 1. 构建极坐标直方图
        polar_histogram = self.build_polar_histogram(ranges, angle_increment)
        
        # 2. 平滑直方图
        smoothed_histogram = self.smooth_polar_histogram(polar_histogram)
        
        # 3. 寻找候选山谷
        valleys = self.find_candidate_valleys(smoothed_histogram, self.threshold)
        
        # 4. 计算目标扇区
        target_sector = int(self.target_direction / math.radians(self.sector_angle)) % self.num_sectors
        
        # 5. 选择最佳方向
        best_sector, selected_valley = self.select_best_direction(valleys, target_sector)
        
        if best_sector is None:
            # 没有找到候选山谷，选择密度最低的方向
            best_sector = np.argmin(smoothed_histogram)
            selected_valley = None

        if selected_valley:
            speed = self.calculate_dynamic_speed(best_sector, smoothed_histogram, current_steering_rate)
        else:
            # 没有找到合适的山谷，使用保守速度
            speed = self.min_linear_speed

        # 将扇区转换为角度（-π到π）
        best_angle = best_sector * math.radians(self.sector_angle)
        
        if best_angle > math.pi:
            best_angle -= 2 * math.pi
        #self.get_logger().info(f"best_angle is {best_angle}") #调试   
        return best_angle, speed

    def check_obstacle_proximity(self, ranges):
        """检查是否需要避障"""
        if ranges is None:
            return False
            
        # 检查前方90度范围内的最小距离
        num_ranges = len(ranges)
        front_start = num_ranges * 2 // 8
        front_end = num_ranges * 6 // 8
        front_ranges = ranges[front_start:front_end]
        
        valid_ranges = [d for d in front_ranges if not (math.isnan(d) or d == 0.0)]
        
        if not valid_ranges:
            return False
            
        min_distance = min(valid_ranges)
        
        # 紧急停止检查
        if min_distance < 0.3:
            self.state = RobotState.EMERGENCY_STOP
            self.get_logger().warn("Emergency stop! Obstacle too close!")
            return True
            
        return min_distance < 2.04  # 2.1米内开始避障

    def chaos_path_planning(self):
        self.y, self.omega, v = calculation(self.normal_speed, self.omega, self.fai, self.y)
        return self.normal_speed, self.omega

    def control_loop(self):
        if self.laser_data is None:
            return
            
        ranges = self.laser_data.ranges
        angle_increment = self.laser_data.angle_increment
        
        cmd_vel = Twist()
        
        # 状态机
        if self.state == RobotState.NORMAL:
            if self.check_obstacle_proximity(ranges):
                self.state = RobotState.OBSTACLE_AVOIDANCE
                self.get_logger().info("Switching to obstacle avoidance mode")
            else:
                # 正常行驶：使用混沌路径规划
                linear_vel, angular_vel = self.chaos_path_planning()
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel
                self.current_speed = linear_vel
                
        elif self.state == RobotState.OBSTACLE_AVOIDANCE:
            # 使用经典VFH算法进行避障
            best_direction, speed = self.classic_vfh_algorithm(ranges, angle_increment, self.current_steering_rate)
            
            # 设置控制命令
            cmd_vel.linear.x = speed
            cmd_vel.angular.z = best_direction
            
            # 限制角速度范围
            cmd_vel.angular.z = max(-self.max_angular_speed, 
                                   min(self.max_angular_speed, cmd_vel.angular.z))
            #self.get_logger().info(f"w is {cmd_vel.angular.z}") #调试

            # 检查是否可以返回正常模式
            if not self.check_obstacle_proximity(ranges):
                self.state = RobotState.NORMAL
                self.get_logger().info("Returning to normal mode")
                
        elif self.state == RobotState.EMERGENCY_STOP:
            # 紧急停止
            cmd_vel.linear.x = -0.2
            cmd_vel.angular.z = 0.0
            self.current_speed = -0.2
            
            # 检查是否可以恢复
            if not self.check_obstacle_proximity(ranges):
                self.state = RobotState.NORMAL
                self.get_logger().info("Emergency cleared, returning to normal mode")
        
        # 发布控制命令
        self.publisher_.publish(cmd_vel)

def myRK4(ode, h, y0):
    y = y0

    k1 = ode(0, y)

    k2 = ode(0 + 1/2 * h, y + h * k1 / 2)

    k3 = ode(0 + 1/2 * h, y + h * k2 / 2)

    k4 = ode(0 + h, y + h * k3)

    temp = h * 1/6 * (k1 + 2 * k2 + 2 * k3 + k4)

    y = y + temp

    return y, k1[0], k3[0]

def robot_chua(t, y, alpha, beta, M, C, v, K, fai):
    dydt = np.zeros(5)
    
    q = (len(C) + 1) / 2
    fx = M[int(q*2 - 1)] * y[0] 
    for j in range(int(2*q - 1)):
        fx += 0.5 * (M[j] - M[j+1]) * (abs(y[0] + C[j]) - abs(y[0] - C[j]))
    
    dydt[0] = alpha * (y[1] - fx)
    dydt[1] = y[0] - y[1] + y[2]
    dydt[2] = -beta * y[1]

    return dydt
        
def calculation(v, omega, fai, y_last):
    alpha = 9
    beta = 14
    C = [1, 2.15, 3.6, 6.2, 9]
    M = [0.9/7, -3/7, 3.5/7, -2.7/7, 4/7, -2.4/7]  
    h = 0.016
    K = 0.6

    y, omega, v = myRK4(lambda t, y: robot_chua(t, y, alpha, beta, M, C, v, K, fai), h, y_last)

    omega_n = K*omega
    v_n = K*v
    return y, omega_n, v_n

def main(args=None):
    rclpy.init(args=args)
    controller = ClassicVFHController() #记得修改load和setup中的启动
    rclpy.spin(controller)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
