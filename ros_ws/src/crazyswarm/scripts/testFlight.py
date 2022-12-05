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

def main():
	swarm = Crazyswarm()
	timeHelper = swarm.timeHelper
	cf = swarm.allcfs.crazyflies[1]
	controller = swarm.allcfs.crazyflies[0]

	
	print("-----TAKEOFF-----")
	cf.takeoff(targetHeight=1.0, duration=3.0)
	timeHelper.sleep(10.0)

	flag = False

	print("-----START FOLLOW-----")

	# while(timeHelper.time() < end_time):
	# for i in range(40):
	# 	controller_transform = getTransform(controller)
	# 	cf_transform = getTransform(cf)

	# 	r = R.from_euler('xyz', controller_transform[1], degrees=True)
	# 	setpoint = r.apply([1,0,0])+controller_transform[0]

	# 	if abs(controller_transform[0,2]-cf_transform[0,2])<0.1:
	# 		flag=True
	# 	if flag:
	# 		cf.goTo(setpoint,0,1.0)
	# 	print('setpoint = '+str(setpoint))
	# 	print('position = '+str(cf_transform[0]))
	# 	timeHelper.sleepForRate(1.0)

	num_secs = 40
	freq = 20
	max_horizontal_speed = 8.0
	max_vertical_speed = 4.0


	start_time = timeHelper.time()
	end_time = start_time + num_secs
	num_iters = num_secs * freq	
	
	max_horizontal_dist = max_horizontal_speed / freq
	max_vertical_dist = max_vertical_speed / freq

	loop_count = 0
	while True:#(timeHelper.time() < end_time):
		controller_transform = getTransform(controller)
		cf_transform = getTransform(cf)

		r = R.from_euler('xyz', controller_transform[1], degrees=True)
		target = r.apply([1,0,0])+controller_transform[0]
		horizontal_step_dist = distance.euclidean(cf_transform[0,:2], target[:2])
		vertical_step_dist = abs(cf_transform[0,2] - target[2])

		if horizontal_step_dist>max_horizontal_dist:
			h_setpoint = cf_transform[0,:2] + (target[:2] - cf_transform[0,:2])*(max_horizontal_dist/horizontal_step_dist)
		else:
			h_setpoint = target[:2]

		if vertical_step_dist>max_vertical_dist:
			v_setpoint = cf_transform[0,2] + (target[2] - cf_transform[0,2])*(max_vertical_dist/vertical_step_dist)
		else:
			v_setpoint = target[2]
		
		setpoint = np.hstack([h_setpoint, v_setpoint])

		if abs(controller_transform[0,2]-cf_transform[0,2])<0.1:
			flag=True
		if flag:
			cf.cmdPosition(setpoint,0)
		
		if loop_count%15==0:
			print('h_speed = '+str(h_setpoint-cf_transform[0,:2]))
			print('v_speed = '+str(v_setpoint-cf_transform[0,2]))
			print('-'*30)
		loop_count+=1
		timeHelper.sleepForRate(freq)

	print("-----LAND-----")
	# cf.land(targetHeight=0.04, duration = 3.0)

if __name__ == "__main__":
	main()
