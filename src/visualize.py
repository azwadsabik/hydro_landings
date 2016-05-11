import rospy
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tf
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
import time

bridge = cv_bridge.CvBridge()

xmax = rospy.get_param("/cage_x")*0.3048/2
ymax = rospy.get_param("/cage_y")*0.3048/2
marker_x_rels = np.array([-3, -1, 1, 3])/8.0
marker_y_rels = np.array([2, 0, -2])/6.0
marker_x = []
marker_y = []

for y_rel in marker_y_rels:
	for x_rel in marker_x_rels:
		x = x_rel*xmax*2
		y = y_rel*ymax*2
		marker_x.append(x)
		marker_y.append(y)

zmax = 4
history = 1000

fig = plt.figure()
planar = fig.add_subplot(2, 2, 1)
image_plot = fig.add_subplot(2, 2, 2)
x_plot = fig.add_subplot(4, 2, 5)
y_plot = fig.add_subplot(4, 2, 6)
z_plot = fig.add_subplot(4, 2, 7)
yaw_plot = fig.add_subplot(4, 2, 8)

estimate_time = [time.time()]
navdata_time = [time.time()]

x = [0]
y = [0]
z = [0]
yaw = [0]
z_truth = [0]
im = np.eye(500, 500)
goal_x = 0
goal_y = 0
goal_z = 0
battery = -1

def update_vis(estimate):
    x.append(estimate.position.x)
    y.append(estimate.position.y)
    z.append(estimate.position.z)
    quaternion = np.zeros((4,))
    quaternion[0] = estimate.orientation.x
    quaternion[1] = estimate.orientation.y
    quaternion[2] = estimate.orientation.z
    quaternion[3] = estimate.orientation.w
    _, _, yaw_estimate = tf.transformations.euler_from_quaternion(quaternion)
    yaw.append(yaw_estimate)
    estimate_time.append(time.time())
    if len(x) > history:
        x.pop(0)
        y.pop(0)
        z.pop(0)
        yaw.pop(0)
        estimate_time.pop(0)
    
def update_goal(goal):
    global goal_x, goal_y, goal_z
    goal_x = goal.position.x
    goal_y = goal.position.y
    goal_z = goal.position.z
    
def update_im(img_msg):
	global im
	im = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")

def update_nav(navdata):
    global battery
    z_truth.append(navdata.altd/1000.0)
    navdata_time.append(time.time())
    if len(z_truth) > history:
        z_truth.pop(0)
        navdata_time.pop(0)
    battery = navdata.batteryPercent    

def animate(i):
    time_min = max(min(estimate_time), min(navdata_time))
    time_max = max(max(estimate_time), max(navdata_time))
    planar.clear()
    image_plot.clear()
    x_plot.clear()
    y_plot.clear()
    z_plot.clear()
    yaw_plot.clear()
    planar.set_xlim((-xmax*1.5, xmax*1.5))
    planar.set_ylim((-ymax*1.5, ymax*1.5))
    x_plot.set_ylim((-xmax, xmax))
    x_plot.set_xlim((time_min, time_max))
    y_plot.set_ylim((-ymax, ymax))
    y_plot.set_xlim((time_min, time_max))
    z_plot.set_ylim((-zmax, zmax))
    z_plot.set_xlim((time_min, time_max))
    yaw_plot.set_ylim((-4, 4))
    yaw_plot.set_xlim((time_min, time_max))
    planar.scatter(x[-1],y[-1])
    planar.scatter(marker_x, marker_y, color='r')
    planar.scatter(x[-1] + np.cos(yaw[-1])/2, y[-1] + np.sin(yaw[-1])/2, color='g')
    planar.scatter(goal_x, goal_y, color='c')
    planar.set_title(battery)
    x_plot.plot(estimate_time, x)
    x_plot.plot(navdata_time, np.ones((len(navdata_time),))*goal_x, 'c')    
    y_plot.plot(estimate_time, y)
    y_plot.plot(navdata_time, np.ones((len(navdata_time),))*goal_y, 'c')    
    z_plot.plot(estimate_time, z)
    z_plot.plot(navdata_time, z_truth, 'g-')
    z_plot.plot(navdata_time, np.ones((len(navdata_time),))*goal_z, 'c')
    yaw_plot.plot(estimate_time, yaw)    
    image_plot.imshow(im)

if __name__ == "__main__":
    rospy.init_node('pose_estimate_visualization')
    pose_estimator = rospy.Subscriber("/pose_estimate", Pose, update_vis)
    goal_publisher = rospy.Subscriber("/goal", Pose, update_goal)
    image_viewer = rospy.Subscriber("/ardrone/image_raw", Image, update_im)
    navdata_sub = rospy.Subscriber('/ardrone/navdata',Navdata,update_nav) 
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()
    rospy.spin()
    pass
