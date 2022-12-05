from pycrazyswarm import Crazyswarm
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
from scipy.spatial import distance

def getTransform(cf):
	cf.tf.waitForTransform("/world", "/cf" + str(cf.id), rospy.Time(0), rospy.Duration(10))
	position, quaternion = cf.tf.lookupTransform("/world", "/cf" + str(cf.id), rospy.Time(0))
	euler = R.from_quat(quaternion).as_euler('xyz', degrees=True)
	return np.array([np.array(position), euler])

def printAlert(string):
    print('')
    print('*'*30)
    print(string)
    print('*'*30)
    print('')

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    controller = swarm.allcfs.crazyflies[0]
    
    drones = swarm.allcfs.crazyflies[1:]

    drone_formation_positions = [
        [1.5,0,0.2],
        [1.5,0.4,-0.2],
        [1.5,-0.4,-0.2]]
    
    guestureRecognized = 0
    guestureState = 'START'
    guesture_start_time = timeHelper.time()
    geusture_max_time = 8
    guesture_min_angle = 30
    
    freq = 20
    max_horizontal_speed = 8.0
    max_vertical_speed = 4.0
    
    max_horizontal_dist = max_horizontal_speed / freq
    max_vertical_dist = max_vertical_speed / freq
    
    loop_count = 0
    while True:
        controller_transform = getTransform(controller)
        
        if guestureState == 'START':
            if controller_transform[1,0] < -guesture_min_angle:
                guestureState = 'S1'
                printAlert('Gesture state changed to: '+guestureState)
                guesture_start_time = timeHelper.time()
        elif guestureState == 'S1':
            if timeHelper.time()-guesture_start_time > geusture_max_time:
                guestureState = 'START'
                printAlert('Gesture state changed to: '+guestureState)
            elif controller_transform[1,0] > guesture_min_angle:
                guestureState = 'S2'
                printAlert('Gesture state changed to: '+guestureState)
        elif guestureState == 'S2':
            if timeHelper.time()-guesture_start_time > geusture_max_time:
                guestureState = 'START'
                printAlert('Gesture state changed to: '+guestureState)
            elif controller_transform[1,0] < -guesture_min_angle:
                guestureState = 'S3'
                printAlert('Gesture state changed to: '+guestureState)
        elif guestureState == 'S3':
            if timeHelper.time()-guesture_start_time > geusture_max_time:
                guestureState = 'START'
                printAlert('Gesture state changed to: '+guestureState)
            elif controller_transform[1,0] > guesture_min_angle:
                guestureState = 'START'
                printAlert('GESTURE RECOGNIZED !')
                guestureRecognized+=1

        for i in range(len(drones)):
            
            drone = drones[i]
            drone_transform = getTransform(drone)
            
            if guestureRecognized%2==0:
                if drone_transform[0,2]<0.1:
                    drone.cmdStop()
                    continue
                else:
                    target = drone_transform[0]
                    target[2] = 0.04
            else:
                if guestureState in ['S2', 'S3']:
                    controller_transform[1,0]=0
                r = R.from_euler('xyz', controller_transform[1], degrees=True)
                target = r.apply(drone_formation_positions[i])+controller_transform[0]

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

            drone.cmdPosition(setpoint,0)
                
            if loop_count%30==0:
                print('drone '+str(i)+' h_speed = '+str(h_setpoint-drone_transform[0,:2]))
                print('drone '+str(i)+' v_speed = '+str(v_setpoint-drone_transform[0,2]))
        

        if loop_count%30==0:
            print('Guesture INFO: {} {} {:.2f}'.format(guestureRecognized, guestureState, (timeHelper.time()-guesture_start_time)))
            print('-'*30)	    
            
        loop_count+=1
		
        timeHelper.sleepForRate(freq)
        

if __name__ == "__main__":
    main()
