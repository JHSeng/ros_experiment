import rospy
import tf
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def teste():
    p = euler_from_quaternion(
        (2.38798784725e-08, -0.0373892758282, 2.07893278392e-06, 0.532975329572))
    print p[2]

    print atan2(10, 10) - p[2]


teste()
