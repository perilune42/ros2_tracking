import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import csv


class GPSFollowerNode(Node):
    def __init__(self):
        super().__init__('gps_path_follower_node')

        MAP_PATH = '/home/projects/ros2_ws/src/ros2_tracking/path/path.csv'
        
        self.controller = GPSPathFollower(
            K_P=0.60, K_I=0.0, K_D=0.15, THROTTLE=0.40,
            PATH_SEARCH_LENGTH=100, PATH_LOOK_AHEAD=35, 
            PATH_LOOK_BEHIND=5, MAP_PATH=MAP_PATH
        )

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/gps/utm',
            self.gps_callback,
            10)
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/spatial_pos',
            self.cam_callback,
            10)
        
        self.opponent_detected = False
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("GPS Path Follower Node has started.")
        self.detection_counter = 0;
        
    def cam_callback(self, msg):
        
        if msg.data:  
            cam_x = msg.data[0]
            cam_z = msg.data[2]
            if cam_z < 3.0:
                self.detection_counter += 1
                self.opponent_detected = self.detection_counter > 3
            else:
                self.opponent_detected = False
                self.detection_counter = 0
        else:
            self.opponent_detected = False
            self.detection_counter = 0

    def gps_callback(self, msg):
        current_pos = np.array([msg.data[0], msg.data[1]])
        
        steering, throttle, info = self.controller.run_opp(current_pos, self.opponent_detected)
        self.get_logger().info(info)

        cmd = Twist()
        
        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steering)

        self.publisher.publish(cmd)
        
        self.get_logger().debug(f"Steer: {steering:.2f} | Throttle: {throttle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    


class GPSPathFollower:
    def __init__(self, K_P, K_I, K_D, THROTTLE, 
                 PATH_SEARCH_LENGTH, PATH_LOOK_AHEAD, PATH_LOOK_BEHIND, 
                 MAP_PATH):
        self.Kp, self.Ki, self.Kd = K_P, K_I, K_D
        self.throttle = THROTTLE
        
        self.search_len = PATH_SEARCH_LENGTH
        self.look_ahead = PATH_LOOK_AHEAD
        self.look_behind = PATH_LOOK_BEHIND
        
        MAP_PATH
        self.path = []# Nx2 array of [x, y]
            
        waypoints = []
        with open(MAP_PATH, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                waypoints.append([float(row[0]), float(row[1])])
        self.path = np.array(waypoints).reshape((-1, 2))
        
        self.last_cte = 0
        self.integral = 0
        self.last_index = 0

    def run(self, pos):
        current_x, current_y = pos
        start = max(0, self.last_index - self.look_behind)
        end = min(len(self.path), self.last_index + self.search_len)
        window = self.path[start:end]
        
        # find closest wp
        dists = np.linalg.norm(window - [current_x, current_y], axis=1)
        local_idx = np.argmin(dists)
        self.last_index = start + local_idx

        idx_a = self.last_index
        idx_b = min(len(self.path) - 1, self.last_index + self.look_ahead)
        
        p_car = np.array([current_x, current_y])
        p1 = self.path[idx_a]
        p2 = self.path[idx_b]

        if np.array_equal(p1, p2):
            cte = np.linalg.norm(p_car - p1)
        else:
            line_vec = p2 - p1
            car_vec = p_car - p1
            line_len = np.sum(line_vec**2)
            t = np.dot(car_vec, line_vec) / line_len
            projection = p1 + t * line_vec
            
            # positive = right of path, negative = left of path
            cte = -np.linalg.norm(p_car - projection) # was wrong, flipped sign
            
            side = line_vec[0] * car_vec[1] - line_vec[1] * car_vec[0]
            if side < 0:
                cte = -cte

        # steering is inverted: error is positive (right), steer left (negative)
        self.integral += cte
        derivative = cte - self.last_cte
        
        steering = -(self.Kp * cte + self.Ki * self.integral + self.Kd * derivative)
        
        self.last_cte = cte
        
        return np.clip(steering, -1.0, 1.0), self.throttle
    
    def run_opp(self, pos, opp_detected):
        current_x, current_y = pos
        start = max(0, self.last_index - self.look_behind)
        end = min(len(self.path), self.last_index + self.search_len)
        window = self.path[start:end]
        
        # 1. Find closest waypoint
        dists = np.linalg.norm(window - [current_x, current_y], axis=1)
        local_idx = np.argmin(dists)
        self.last_index = start + local_idx

        idx_a = self.last_index
        idx_b = min(len(self.path) - 1, self.last_index + self.look_ahead)
        
        p_car = np.array([current_x, current_y])
        p1 = self.path[idx_a]
        p2 = self.path[idx_b]

        # add offset
        lateral_offset = 0.0
        if opp_detected:
            lateral_offset = -0.25

        info = ""
        if np.array_equal(p1, p2):
            cte = np.linalg.norm(p_car - p1)
        else:
            line_vec = p2 - p1
            car_vec = p_car - p1
            line_len = np.sum(line_vec**2)
            t = np.dot(car_vec, line_vec) / line_len
            projection = p1 + t * line_vec
            
            cte = -(np.linalg.norm(p_car - projection) - lateral_offset) # flipped sign
            info = f"opp_detected: {opp_detected} cte: {cte}"
            side = line_vec[0] * car_vec[1] - line_vec[1] * car_vec[0]
            
            # current_side_dist = dist_to_path if side >= 0 else -dist_to_path
            if side < 0:
                cte = -cte
                
        self.integral += cte
        derivative = cte - self.last_cte
        
        steering = -(self.Kp * cte + self.Ki * self.integral + self.Kd * derivative)
        self.last_cte = cte
        
        current_throttle = (self.throttle * 0.8) if opp_detected else self.throttle
        
        return np.clip(steering, -1.0, 1.0), current_throttle, info
        
