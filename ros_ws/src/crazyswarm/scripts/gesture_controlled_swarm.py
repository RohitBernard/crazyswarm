from pycrazyswarm import Crazyswarm
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance
import sys

## Fetch position and orientation of crazyflie from ROS topic and convert orientation to euler angles
def getTransform(cf):
	cf.tf.waitForTransform("/world", "/cf" + str(cf.id), rospy.Time(0), rospy.Duration(10))
	position, quaternion = cf.tf.lookupTransform("/world", "/cf" + str(cf.id), rospy.Time(0))
	euler = R.from_quat(quaternion).as_euler('xyz', degrees=True)
	return np.array([np.array(position), euler])

## Format messages for logging
def printAlert(string):
    print('')
    print('*'*30)
    print(string)
    print('*'*30)
    print('')

## Parse CLI argument --no-rotate
def noRotate():
    if len(sys.argv)>1:
        if sys.argv[1]=='--no-rotate':
            return True
    return False

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    controller = swarm.allcfs.crazyflies[0]
    
    drones = swarm.allcfs.crazyflies[1:]

    ## Define position of drones [x, y, z] relative to control position (User)
    drone_formation_positions = [
        [1.25,0,0.25],
        [1.75,0.5,-0.25],
        [1.75,-0.5,-0.25]]
    
    gestureRecognized = 0                   # Number of Gestures recognized
    gestureState = 'START'                  # Initial Gesture State
    gesture_start_time = timeHelper.time()
    gesture_max_time = 8                    # Number of seconds within which "Hand Wave" gesture must be completed
    gesture_min_angle = 30                  # Minimum angle of rotation to be recognized as "Hand Wave" gesture
    
    freq = 20                               # Frequency of control loop (also frequency of commands to drones)
    max_horizontal_speed = 8.0              # Set maximum horizontal speed for all drones
    max_vertical_speed = 4.0                # Set maximum vertical speed for all drones
    
    max_horizontal_dist = max_horizontal_speed / freq
    max_vertical_dist = max_vertical_speed / freq
    
    loop_count = 0
    while True:
        controller_transform = getTransform(controller)
        
        ## Implementation of a timed automata used to recognize "Hand Wave" gesture
        if gestureState == 'START':
            if controller_transform[1,0] < -gesture_min_angle:
                gestureState = 'S1'
                printAlert('Gesture state changed to: '+gestureState)
                gesture_start_time = timeHelper.time()
        elif gestureState == 'S1':
            if timeHelper.time()-gesture_start_time > gesture_max_time:
                gestureState = 'START'
                printAlert('Gesture state changed to: '+gestureState)
            elif controller_transform[1,0] > gesture_min_angle:
                gestureState = 'S2'
                printAlert('Gesture state changed to: '+gestureState)
        elif gestureState == 'S2':
            if timeHelper.time()-gesture_start_time > gesture_max_time:
                gestureState = 'START'
                printAlert('Gesture state changed to: '+gestureState)
            elif controller_transform[1,0] < -gesture_min_angle:
                gestureState = 'S3'
                printAlert('Gesture state changed to: '+gestureState)
        elif gestureState == 'S3':
            if timeHelper.time()-gesture_start_time > gesture_max_time:
                gestureState = 'START'
                printAlert('Gesture state changed to: '+gestureState)
            elif controller_transform[1,0] > gesture_min_angle:
                gestureState = 'START'
                printAlert('GESTURE RECOGNIZED !')
                gestureRecognized+=1

        for i in range(len(drones)):
            
            drone = drones[i]
            drone_transform = getTransform(drone)
            
            ## Determine toggle state of "Hand Wave" gesture (on or off)
            ## Odd number of gestures recognized implies swarm must be flying
            ## Even number of gestures recognized implies swarm must be on the ground
            if gestureRecognized%2==0:
                if drone_transform[0,2]<0.1:
                    drone.cmdStop()                 # Turn off rotors if close to the ground
                    continue
                else:
                    target = drone_transform[0]
                    target[2] = 0.04                # Set the target to the ground at current (x,y) position
            else:
                if gestureState in ['S2', 'S3'] or noRotate():
                    controller_transform[1,0]=0                                         # If --no-rotate enabled or gesture is in action, set roll angle to 0
                r = R.from_euler('xyz', controller_transform[1], degrees=True)
                target = r.apply(drone_formation_positions[i])+controller_transform[0]  # Calculate target position for each drone

            ## Calculate setpoint coordinates for each drone based on horizontal and vertical velocity constraints
            horizontal_step_dist = distance.euclidean(drone_transform[0,:2], target[:2])
            vertical_step_dist = abs(drone_transform[0,2] - target[2])
            
            if horizontal_step_dist>max_horizontal_dist:
                h_setpoint = drone_transform[0,:2] + (target[:2] - drone_transform[0,:2])*(max_horizontal_dist/horizontal_step_dist)
            else:
                h_setpoint = target[:2]
            
            if vertical_step_dist>max_vertical_dist:
                v_setpoint = drone_transform[0,2] + (target[2] - drone_transform[0,2])*(max_vertical_dist/vertical_step_dist)
            else:
                v_setpoint = target[2]
                
            setpoint = np.hstack([h_setpoint, v_setpoint])

            ## Command each drone to move to setpoint coordinates with no change in Yaw
            drone.cmdPosition(setpoint,0)
            
            ## Log drone information to console
            if loop_count%30==0:
                print('drone '+str(i)+' h_speed = '+str(h_setpoint-drone_transform[0,:2]))
                print('drone '+str(i)+' v_speed = '+str(v_setpoint-drone_transform[0,2]))
        
        ## Log gesture information to console
        if loop_count%30==0:
            print('Gesture INFO: {} {} {:.2f}'.format(gestureRecognized, gestureState, (timeHelper.time()-gesture_start_time)))
            print('-'*30)	    
            
        loop_count+=1
		
        ## Maintain frequency of control loop
        timeHelper.sleepForRate(freq)
        

if __name__ == "__main__":
    main()

## TODO: Implement https://arxiv.org/pdf/2012.05457.pdf to mitigate effects of downwash