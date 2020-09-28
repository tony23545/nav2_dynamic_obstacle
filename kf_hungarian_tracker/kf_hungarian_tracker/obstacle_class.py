import numpy as np 
import cv2

class ObstacleClass:
    """wrap a kalman filter and extra information for one single obstacle

    State space is 3D (x, y, z) by default, if you want to work on 2D (for example top-down view), you can simply make z a constant value and independent of x, y.

    Arrtibutes:
        position: 3d position of center point, numpy array with shape (3, 1)
        velocity: 3d velocity of center point, numpy array with shape (3, 1)
        kalman: cv2.KalmanFilter
        dying: count missing frames for this obstacle, if reach threshold, delete this obstacle
    """

    def __init__(self, obstacle_msg, idx, dim, measurementNoiseCov, errorCovPost, a_noise):
        '''Initialize with an Obstacle msg and an assigned id'''
        self.msg = obstacle_msg
        self.msg.id = idx
        position = np.array([[obstacle_msg.position.x, obstacle_msg.position.y, obstacle_msg.position.z]]).T # shape 3*1
        velocity = np.array([[obstacle_msg.velocity.x, obstacle_msg.velocity.y, obstacle_msg.velocity.z]]).T

        # check aganist state space dimension
        if dim == 2:
            measurementNoiseCov[2] = 0.
            errorCovPost[2] = 0.
            errorCovPost[5] = 0.
            a_noise[2] = 0.

        # setup kalman filter
        self.kalman = cv2.KalmanFilter(6,3) # 3d by default, 6d state space and 3d observation space
        self.kalman.measurementMatrix = np.array([[1,0,0,0,0,0], [0,1,0,0,0,0], [0,0,1,0,0,0]], np.float32)
        self.kalman.measurementNoiseCov = np.diag(measurementNoiseCov).astype(np.float32)
        self.kalman.statePost = np.concatenate([position, velocity]).astype(np.float32)
        self.kalman.errorCovPost = np.diag(errorCovPost).astype(np.float32)
        
        self.dying = 0
        self.dim = dim
        self.a_noise = a_noise

    def predict(self, dt):
        '''update F and Q matrices, call KalmanFilter.predict and store position and velocity'''

        # construct new transition matrix
        '''
        F = 1, 0, 0, dt, 0,  0
            0, 1, 0, 0,  dt, 0
            0, 0, 1, 0,  0,  dt
            0, 0, 0, 1,  0,  0
            0, 0, 0, 0,  1,  0
            0, 0, 0, 0,  0,  1
        '''
        F = np.eye(6).astype(np.float32)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # construct new process conv matrix
        '''assume constant velocity, and obstacle's acceleration has noise in x, y, z direction ax, ay, az
        Q = dt4*ax/4, 0,        0,        dt3*ax/2, 0,        0
            0,        dt4*ay/4, 0,        0,        dt3*ay/2, 0
            0,        0,        dt4*az/4, 0,        0,        dt3*az/2
            dt3*ax/2, 0,        0,        dt2*ax,   0,        0
            0,        dt3*ay/2, 0,        0,        dt2*ay,   0
            0,        0,        dt3*az/2, 0,        0,        dt2*az
        '''
        dt2 = dt**2
        dt3 = dt*dt2
        dt4 = dt2**2
        Q = np.array([[dt4*self.a_noise[0]/4, 0, 0, dt3*self.a_noise[0]/2, 0, 0], 
                      [0, dt4*self.a_noise[1]/4, 0, 0, dt3*self.a_noise[1]/2, 0],
                      [0, 0, dt4*self.a_noise[2]/4, 0, 0, dt3*self.a_noise[2]/2],
                      [dt3*self.a_noise[0]/2, 0, 0, dt2*self.a_noise[0], 0, 0],
                      [0, dt3*self.a_noise[1]/2, 0, 0, dt2*self.a_noise[1], 0],
                      [0, 0, dt3*self.a_noise[2]/2, 0, 0, dt2*self.a_noise[2]]]).astype(np.float32)

        self.kalman.transitionMatrix = F
        self.kalman.processNoiseCov = Q
        self.kalman.predict()

    def correct(self, detect_msg):
        '''extract position as measurement and update KalmanFilter'''
        measurement = np.array([[detect_msg.position.x, detect_msg.position.y, detect_msg.position.z]]).T.astype(np.float32)
        self.kalman.correct(measurement)
        self.msg.position.x = np.float(self.kalman.statePost[0][0])
        self.msg.position.y = np.float(self.kalman.statePost[1][0])
        self.msg.position.z = np.float(self.kalman.statePost[2][0])
        self.msg.velocity.x = np.float(self.kalman.statePost[3][0])
        self.msg.velocity.y = np.float(self.kalman.statePost[4][0])
        self.msg.velocity.z = np.float(self.kalman.statePost[5][0])
        self.msg.scale = detect_msg.scale

    def distance(self, other_msg):
        '''measurement distance between two obstacles, dy default it's Euler distance between centers
           you can extent the Obstacle msg to include more features like bounding box or radius and include in the distance function'''
        position = np.array([[self.msg.position.x, self.msg.position.y, self.msg.position.z]]).T
        other_position = np.array([[other_msg.position.x, other_msg.position.y, other_msg.position.z]]).T
        return np.linalg.norm(position - other_position)