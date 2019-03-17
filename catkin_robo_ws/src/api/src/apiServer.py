#! /usr/bin/env python

from flask import Flask, request, abort

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from src.PersistentStorage import PersistentStorage

app = Flask(__name__)
persistentStorage = PersistentStorage()
manualGoalPublisher = rospy.Publisher('manual_goal_pose', PoseStamped, queue_size=10)
explorationPublisher = rospy.Publisher('exploration_on', String, queue_size=10)
stopMotorsPublisher = rospy.Publisher('stop_motors', String, queue_size=10)


def shutdownHandler():
    persistentStorage.saveLocations()
    raise RuntimeError("Server going down")


rospy.on_shutdown(shutdownHandler)


@app.route('/', methods=["GET"])
def index():
    return "Hello, World!"


@app.route('/manual_pose', methods=['PUT'])
def manual_pose():
    if not request.form:
        abort(400)
    
    goal = PoseStamped()
    goal.header.frame_id = "/map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.z = 0.0
    goal.pose.position.x = float(request.form['x'])
    goal.pose.position.y = float(request.form['y'])
    goal.pose.orientation.w = 1.0
    manualGoalPublisher.publish(goal)

    return "OK"


@app.route('/stop_exploration', methods=['PUT'])
def stop_exploration():
    explorationPublisher.publish("OFF")
    return "OK"


@app.route('/start_exploration', methods=['PUT'])
def start_exploration():
    explorationPublisher.publish("ON")
    return "OK"


@app.route('/stop_motors', methods=['PUT'])
def stop_motors():
    stopMotorsPublisher.publish("")
    return "OK"


@app.route('/save_location', methods=['PUT'])
def stop_motors():
    if not request.form:
        abort(400)

    # catch robot location and add to saved
    # name = request.form['name']
    # persistentStorage.addLocation(pose, name)
    return "OK"


def publisher():
    rospy.init_node('apiServer', anonymous=True, log_level=rospy.DEBUG)
    
    app.run(port=9090, host="0.0.0.0", threaded=True, debug=True)
    rospy.loginfo("Server is running")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
