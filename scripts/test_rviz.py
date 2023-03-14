#!/usr/bin/env python

import rospy
from occupancy_grid_python import OccupancyGridManager
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid

class OGMtester(object):
    def __init__(self):
        rospy.init_node('OGM_tester')

        self.ogm = OccupancyGridManager('/wheelchair/move_base/global_costmap/costmap', subscribe_to_updates=True)

        self.last_costmap = None

        self.point_pub = rospy.Publisher('/safe_point', PointStamped, queue_size = 1)

        rospy.Subscriber('/clicked_point', PointStamped, self.point_cb)

    def point_cb(self, msg):
        #print(self.ogm.origin)

        cost = self.ogm.get_cost_from_world_x_y(msg.point.x, msg.point.y)
        rospy.loginfo("cost {};{} is {}".format(int(msg.point.x), int(msg.point.y), cost))

        if cost > 50:
            costmap_x, costmap_y = self.ogm.get_costmap_x_y(msg.point.x, msg.point.y)

            safe_x, safe_y, safe_cost = self.ogm.get_closest_cell_under_cost(costmap_x, costmap_y, 50, 10)
            rospy.loginfo("{} {} {}".format(safe_x, safe_y, safe_cost))

            if safe_cost != -1 and safe_x != -1 and safe_y != -1:

                safe_point = msg

                safe_point.point.x, safe_point.point.y = self.ogm.get_world_x_y(safe_x, safe_y)
                safe_point.point.z = 0.2

                self.point_pub.publish(safe_point)                          


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ogmt = OGMtester()
    ogmt.run()