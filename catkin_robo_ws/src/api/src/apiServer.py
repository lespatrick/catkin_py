#! /usr/bin/env python

from flask import Flask, request, abort

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

# from PersistentStorage import PersistentStorage

app = Flask(__name__)
# persistentStorage = PersistentStorage()
manualGoalPublisher = rospy.Publisher('manual_goal_pose', PoseStamped, queue_size=10)
explorationPublisher = rospy.Publisher('exploration_on', String, queue_size=10)
stopMotorsPublisher = rospy.Publisher('stop_motors', String, queue_size=10)
motorsPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
saveMapPublisher = rospy.Publisher('save_map', String, queue_size=1)


def shutdownHandler():
    # persistentStorage.saveLocations()
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


@app.route('/override_motors', methods=['PUT'])
def override_motors():
    lineX = float(request.form['strength'])
    angleZ = float(request.form['angle'])
    twist = Twist()
    twist.linear.x = lineX * 0.5
    twist.angular.z = angleZ
    motorsPublisher.publish(twist)
    rospy.loginfo(str(angleZ))
    return "OK"


@app.route('/save_map', methods=['PUT'])
def save_map():
    saveMapPublisher.publish("")
    rospy.loginfo('/save_map call received')
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
