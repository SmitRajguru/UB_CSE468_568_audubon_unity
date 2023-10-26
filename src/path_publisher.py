#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped


class Path_Publisher:
    def __init__(self, car_name):
        self.car_name = car_name

        self.path_publisher = rospy.Publisher(
            f"/{self.car_name}/path", Path, queue_size=1
        )

        self.odom_subscriber = rospy.Subscriber(
            f"/{self.car_name}/odom", Odometry, self.odom_callback, queue_size=1
        )

        self.reset_subscriber = rospy.Subscriber(
            f"/{self.car_name}/reset", Pose, self.reset_callback
        )

        self.path = Path()
        self.last_callback_time = rospy.Time.now()

        self.isRemoved = False
        self.sequenceId = 0

    def odom_callback(self, msg):
        # only publish path every 0.1 seconds
        if (rospy.Time.now() - self.last_callback_time).to_sec() < 0.1:
            return
        self.last_callback_time = rospy.Time.now()

        newpose = PoseStamped()
        newpose.header = msg.header
        newpose.header.seq = self.sequenceId
        newpose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.header.seq = self.sequenceId
        self.path.header.frame_id = "world"
        self.path.poses.append(newpose)

        self.sequenceId += 1

        self.path_publisher.publish(self.path)

    def reset_callback(self, msg):
        # reset path
        self.path.poses.clear()
        self.path_publisher.publish(self.path)
        self.remove()

    def remove(self):
        self.odom_subscriber.unregister()
        self.path_publisher.unregister()
        self.isRemoved = True


def main():
    publishers = dict()

    rospy.init_node("path_publisher", anonymous=True)

    rate = rospy.Rate(1)

    while True:
        # get all topics
        topics = dict(rospy.get_published_topics())

        # check if any publisher is removed
        for car_name in list(publishers.keys()):
            if publishers[car_name].isRemoved:
                del publishers[car_name]

        # get all car odom topics
        odom_topics = [
            topicName
            for topicName in list(topics.keys())
            if topicName.endswith("/odom")
        ]

        # get all car names
        car_names = [odom_topic[1:-5] for odom_topic in odom_topics]

        # create a publisher for each car if it doesn't exist
        for car_name in car_names:
            if car_name not in publishers.keys():
                publishers[car_name] = Path_Publisher(car_name)

        # remove publishers for cars that don't exist anymore
        for car_name in list(publishers.keys()):
            if car_name not in car_names:
                publishers[car_name].remove()

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
