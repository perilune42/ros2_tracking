import numpy as np
import csv

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

    def run(self, pos, opp_pos):
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
            cte = np.linalg.norm(p_car - projection)
            
            side = line_vec[0] * car_vec[1] - line_vec[1] * car_vec[0]
            if side < 0:
                cte = -cte

        # steering is inverted: error is positive (right), steer left (negative)
        self.integral += cte
        derivative = cte - self.last_cte
        
        steering = -(self.Kp * cte + self.Ki * self.integral + self.Kd * derivative)
        
        self.last_cte = cte
        
        return np.clip(steering, -1.0, 1.0), self.throttle
