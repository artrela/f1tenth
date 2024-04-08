#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import os
import atexit

from scipy.interpolate import splprep, splev
from transforms3d.euler import quat2euler

# TODO CHECK: include needed ROS msg type headers and libraries
# wps = []
topic = 'ego_racecar/odom'
save_loc = os.path.dirname(os.path.abspath(__file__))
save_loc = os.path.join(save_loc + "/../")
file = open('wp.csv', 'w')

class WaypointLogger(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):

        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(
            Odometry,
            topic,
            self.save_waypoint,
            10)
        self.subscription  # prevent unused variable warning

        
        print(f"Saving Waypoints to: wps.csv")
    
    def save_waypoint(self, data):
        
        roll, pitch, yaw = quat2euler([
            data.pose.pose.orientation.w,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
        ])

        # wps.append((data.pose.pose.position.x,
                        # data.pose.pose.position.y,))   
        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x, data.pose.pose.position.y,
                                        data.twist.twist.linear.x, yaw))
        
def shutdown():
    file.close()

# def log_csv():
#     global wps
    
#     wps = np.array(wps) 

#     # Quick check for duplicate points
#     # if len(np.unique(wps, axis=0)) != len(wps):
#     #     print("There are duplicate points, removing them.")
#     #     wps = np.unique(wps, axis=0)
         
#     # tck, u = splprep([wps[:, 0], wps[:, 1]], s=0)
#     # new_points = splev(u, tck)

#     # fig, ax = plt.subplots()
    
#     # ax.set_title("Waypoints Plotted")
#     # ax.set_aspect('equal')
#     # ax.plot(wps[:, 0], wps[:, 1], 'ro', label="orig wps")
#     # ax.plot(new_points[0], new_points[1], 'b+', label="interp wps") 
#     # ax.legend()

#     # plt.savefig(save_loc + "wps_viz.png")

#     file_orig = open('wp.csv', 'w')
#     # file_interp = open(save_loc + '/wp_interp.csv', 'w')

#     for d1 in wps:#zip(wps, new_points):
#         file_orig.write(f'{d1[0]},{d1[1]}')
#         # file_interp.write(f'{d2[0]},{d2[1]}')

#     file_orig.close()
#     # file_interp.close()

#     if not os.path.exists('wp.csv'):# or not os.path.exists('wp_interp.csv'):
#         print("FILES NOT SAVED!")
#     else:
#         print("Files saved successfully...")

def main(args=None):
    rclpy.init(args=args)

    waypoint_logger = WaypointLogger()
    # atexit.register(log_csv)
    atexit.register(shutdown)

    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()

    # waypoint_logger.log_csv()
    # waypoint_logger.log_csv()

if __name__ == '__main__':
    main()