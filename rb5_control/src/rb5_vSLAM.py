#!/usr/bin/env python
import rospy
import numpy as np
import math
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
        var_Sensor_noise: np.ndarray for r and phi (size 2)
            Variance of the sensor noise
        """
        self.var_System_noise = var_System_noise 
        self.var_Sensor_noise = var_Sensor_noise
        
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

        # Define Sensor Noise
        Rt = np.eye(2)
        var_r, var_phi = self.var_Sensor_noise
        Rt[0,0], Rt[1,1] = var_r, var_phi  
        
        # tag_id, curr_r, curr_z, curr_x
        for posX_landmark, posY_landmark, tagID in landmarks:
            
            if tagID not in self.observed:
                self.observed.append(tagID)         # Append to the observed list
                j = self.observed.index(tagID)      # Get the index of the tagID from the observed list
                idx = 3 + 2 * j                     # Determine the index of the tagID for the state vector
                r = np.linalg.norm(np.array([posX_landmark, posY_landmark])
                phi = math.atan2(posX_landmark, posY_landmark)
                # Landmark position in world frame
                landmark_x = x + math.cos(phi + theta)          
                landmark_y = x + math.cos(phi + theta)
                # Vertically stack to mew
                self.mu = np.vstack((self.mu, landmark_x, landmark_y))
                # Get the length of the vehicle state vector.
                mu_len = len(self.mu)

            # Determine the distance between landmark position and vehicle position
            delta = np.array([[self.mu[id][0] - x], [self.mu[idx+1][0] - y]])
            # Determine q (scalar)
            q = delta.T @ delta

            # Convert the observation data format to range-bearing format
            z_tilde = np.array([[np.sqrt(q)], [math.atan2(delta[0][0], delta[1][0]) - theta]])

            # Create Fxj matrix that map from 2 to 2M + 3
            Fxj = np.array((5, mu_len))
            Fxj[:3,:3] = np.eye(3)
            Fxj[3, idx], Fxj[4, idx+1] = 1, 1

            # Define Jacobian Matrix for the Sensor Measurement
            J = 1 / q * np.array([
                [-np.sqrt(q) * delta[0][0], -np.sqrt(q) * delta[1][0], 0, np.sqrt(q) * delta[0][0], np.sqrt(q) * delta[1][0]],
                [delta[1][0], -delta[0][0], -q, -delta[1][0], delta[0][0]]                
                ])
            
            # Calculate H, which is the measurement prediction p(z |s ) ie a prediction of where features 
            # in the world are in the sensory frame [2, 3+2M]
            H = J @ Fxj 

            # Calculate the Kalman Gain, K [3+2M, 2]
            K = self.cov @ H.T @ np.linalg.inv(H @ self.cov @ H.T + Rt)

            # Define sensor measurement, z
            z = np.array([[r], [phi]])
            # Update mu
            self.mu = self.mu + K @ (z - z_tilde)
            # Update cov
            self.cov = (np.eye(mu_len) - K @ H) @ self.cov

        return self.mu, self.cov


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
