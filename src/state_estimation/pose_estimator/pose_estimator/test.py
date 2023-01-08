import numpy as np

from .kalman_filter import KalmanFilter



def main():
    robot_name = "anymal_c"
    
    kalman_filter = KalmanFilter(robot_name)
    
    kalman_filter.predict(np.array([0,0,0]), np.array([0,0,-9.81]))
    
    kalman_filter.fuse_imu()
    
    qj = np.zeros(12)
    qj_dot = np.zeros(12)
    qj_dot[2] = 1
    kalman_filter.fuse_odo(qj, qj_dot, ["LF"])
    
    print(kalman_filter._state)

if __name__ == '__main__':
    main()