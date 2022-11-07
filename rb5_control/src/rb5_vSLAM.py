#!/usr/bin/env python
import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
from rb5_visual_servo_control import PIDcontroller, getCurrentPos, genTwistMsg, coord

class EKF_vSLAM:
    def __init__(self, var_System_noise, var_Sensor_noise) -> None:
        """
        Initialize pose and covariance. 
        
        Parameters
        ----------
        var_System_noise: float
            Variance of the vehicle model/system noise
        var_Sensor_noise: float
            Variance of the sensor noise
        """
        self.var_System_noise = var_System_noise 
        self.var_Sensom_noise = var_Sensor_noise
        
        # Initialization
        self.mu = np.zeros((3, 1))                  # Pose of the vehicle and positions of the landmark. (M - number of landmarks), (3 + 2M, 1)
        self.cov = np.zeros((3, 3))                 # Covariance matrix of the state model, which is the uncertainty in the pose/state estimate, (3 + 2M, 3 + 2M)
        self.observed = []                          # List that stores observed Apriltags' id
    
    def predict_EKF(self, twist, dt, var_System_noise):
        """
        EKF Prediction Step 
        
        Parameters
        ----------
        twist: numpy.ndarray (3 + 2M, 1)
            Control vector, which includes twist vector (i.e., linear velocities, and angular velocity)
        dt: float
            Timestep
        var_system_noise: float
            Variance of the vehicle model/system noise
        """
        vx, vy, w = twist           # Get vehicle linear velocities and angular velocity
        
        # Get the length of the vehicle state vector.
        mu_len = len(self.mu)
        
        # Define the F matrix, which is the autonomous evloution
        F = np.eye(mu_len)
        # Define the G control matrix
        G = np.zeros((mu_len, mu_len))
        G[0, 0], G[1, 1], G[2, 2] = dt, dt, dt
        # Define the u control vector
        u = np.zeros((mu_len, 1))
        u[0,0], u[1,0], u[2,0] = vx, vy, w
        # Define Qt matrix, which is the uncertainty of the model/system noise
        Qt = np.eye(mu_len) * np.random.normal(0, np.sqrt(self.var_System_noise))
        
        # Estimate the state
        self.mu = F @ self.mu + G @ u
        # Estimate the covariance
        self.cov = F @ self.cov @ F.T + Qt
        
    def update_EKF(self, landmarks):
        """
        EKF Update Step 
        
        Parameters
        ----------
        landmarks:
            Detected landmarks
        """
        
        x, y, theta = self.mu[:3, 0]           # Get estimated vehicle pose
        
        # Get the length of the vehicle state vector.
        mu_len = len(self.mu)
        
        for posX_landmark, posY_landmark, tagID in landmarks:
            
            if tagID not in self.observed:
                self.observed.append(tagID)
                j = self.observed.index(tagID)
                

            
                
                
                
                
            
                
            
            
            
            
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        


if __name__ == "__main__":
    import time
    rospy.init_node("vSLAM")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    
    listener = tf.TransformListener()

    # Square Path
    waypoint = np.array([[0.0,0.0,0.0], 
                         [2.0,0.0,0.0],
                         [2.0,2.0,np.pi/2],
                         [0.0,2.0,np.pi],
                         [0.0,0.0,-np.pi/2]])

    # init pid controller
    scale = 1.0
    pid = PIDcontroller(0.03*scale, 0.002*scale, 0.00001*scale)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        found_state, estimated_state = getCurrentPos(listener)
        if found_state: # if the tag is detected, we can use it to update current state.
            current_state = estimated_state
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.30): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state))) 
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            found_state, estimated_state = getCurrentPos(listener)
            if found_state:
                current_state = estimated_state
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
