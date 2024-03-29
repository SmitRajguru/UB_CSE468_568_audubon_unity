#!/usr/bin/env python3

# import libraries
import rospy
import os
import subprocess
import time
from nav_msgs.msg import OccupancyGrid


class MapServer:
    def __init__(self):
        # initialize node
        rospy.init_node("map_server_node", anonymous=True)

        # load map params from json
        self.isLocalMap = rospy.get_param("isLocalMap")
        self.mapFile = rospy.get_param("mapFile")

        # initialize variables
        self.map = None
        self.mapSaver = None
        self.mapServer = None

        rospy.on_shutdown(self.shutdown)

        if self.isLocalMap:
            # load from mapfile
            print("Map Publiser - loading map from file")
            runCommand = ""
            # check if catkin_ws exists in home directory
            if not os.path.exists("/home/cse4568/catkin_ws"):
                if os.path.exists("/mnt/c/catkin_noetic"):
                    runCommand += "source /mnt/c/catkin_noetic/devel/setup.bash; "
                else:
                    print("Map Publiser - catkin_ws not found")
            else:
                runCommand += "source /home/cse4568/catkin_ws/devel/setup.bash; "

            runCommand += "roscd audubon_unity/maps; "

            runCommand += f"rosrun map_server map_server {self.mapFile}.yaml "
            self.mapServer = subprocess.Popen(
                runCommand, shell=True, executable="/bin/bash"
            )

        else:
            # initialize subscriber
            self.mapPub = rospy.Publisher("/temp_map", OccupancyGrid, queue_size=1)
            self.mapSub = rospy.Subscriber(
                "/unity_map", OccupancyGrid, self.mapCallback
            )

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def shutdown(self):
        if self.mapSaver is not None:
            print("Map Publiser - killing map saver")
            self.mapSaver.kill()
        if self.mapServer is not None:
            print("Map Publiser - killing map server")
            self.mapServer.kill()

        if not self.isLocalMap:
            # remove temp map files
            print("Map Publiser - removing temp map files")
            deleter = subprocess.Popen("rm temp_map*", shell=True)
            deleter.wait()

        print("Map Publiser - shutdown complete")

    def mapCallback(self, msg):
        self.map = msg
        print("Map Publiser - received map")

        # save map to file using map_server on topic temp_map
        print("Map Publiser - saving map to file")
        self.mapSaver = subprocess.Popen(
            "rosrun map_server map_saver -f temp_map map:=/temp_map",
            shell=True,
        )

        # publish map to topic temp_map until map saver is ready
        while self.mapSaver.poll() is None:
            time.sleep(1)
            print("Map Publiser - publishing map")
            self.mapPub.publish(self.map)

        # load map from file using map_server
        if self.mapServer is not None:
            print("Map Publiser - killing map server")
            self.mapServer.kill()
            time.sleep(1)

        print("Map Publiser - loading map from file")
        self.mapServer = subprocess.Popen(
            "rosrun map_server map_server temp_map.yaml", shell=True
        )


if __name__ == "__main__":
    try:
        mapServer = MapServer()
        mapServer.run()
    except rospy.ROSInterruptException:
        pass
