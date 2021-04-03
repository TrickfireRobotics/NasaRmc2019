#! /usr/bin/env python

import rospy
from math import sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Quaternion, Vector3, TransformStamped
from itertools import izip
import tf
import tf2_ros


class OdometryFaker(object):
    """Rough and tumble odometry estimator, based on xsens imu"""
    covariance = [ 1e-1,   0,   0,   0,   0,   0,
                      0,1e-1,   0,   0,   0,   0,
                      0,   0,1e-1,   0,   0,   0,
                      0,   0,   0,1e-1,   0,   0,
                      0,   0,   0,   0,1e-1,   0,
                      0,   0,   0,   0,   0,1e-1]

    def __init__(self):
        self.trans = None
        self.odom = None
        self.position = [10,0,0]
        self.lin_vel = [0,0,0]
        self.lin_acc = [0,0,0]
        self.th = 0
        self.last_time = rospy.get_time()
        self.pub = rospy.Publisher("/odom", Odometry, queue_size=5 )
        self.timer = rospy.Timer(rospy.Duration(0.01), self.odomCallback)
        self.tf_pub = tf2_ros.TransformBroadcaster()


    def getVelocities(self, dt, data):
        acc = vector3ListConversion(data.linear_acceleration)
        out = [(x_1 - x_0)/dt for x_1, x_0 in izip(self.lin_acc, acc)]
        self.lin_acc = acc
        return out

    def updateState(self, data):
        dt = (rospy.get_time() - self.last_time)
        self.last_time += dt
        self.lin_vel = self.getVelocities(dt, data)
        quaternion = (  data.orientation.x,
                        data.orientation.y,
                        data.orientation.z,
                        data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.th = euler[2]
        dx = (self.lin_vel[0]*cos(self.th) - self.lin_vel[1]*sin(self.th))*dt
        dy = (self.lin_vel[0]*sin(self.th) + self.lin_vel[1]*cos(self.th))*dt
        self.position[0] += dx
        self.position[1] += dy

    def packagePose(self, data):
        out = PoseWithCovariance()
        out.pose.position = listVector3Conversion(self.position)
        out.pose.orientation = data.orientation
        out.covariance = OdometryFaker.covariance
        return out

    def packageTwist(self, data):
        out = TwistWithCovariance()
        out.twist.linear = listVector3Conversion(self.lin_vel)
        out.twist.angular = data.angular_velocity
        out.covariance = OdometryFaker.covariance
        return out

    def setupTransform(self,data):
        trans = TransformStamped()
        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = "odom"
        trans.child_frame_id = "base_footprint"
        trans.transform.translation.x = self.position[0]
        trans.transform.translation.y = self.position[1]
        trans.transform.translation.z = self.position[2]
        rotation = tf.transformations.quaternion_from_euler(0,0,self.th)
        trans.transform.rotation.x = rotation[0]
        trans.transform.rotation.y = rotation[1]
        trans.transform.rotation.z = rotation[2]
        trans.transform.rotation.w = rotation[3]
        return trans

    def setupOdometry(self, data):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose =  self.packagePose(data)
        odom.twist = self.packageTwist(data)
        return odom

    def imuCallback(self,data):
        """"Makes a super rough estimate of position based on the imu data"""
        self.updateState(data)
        self.trans = self.setupTransform(data)
        self.odom = self.setupOdometry(data)

    def odomCallback(self,event):
        if self.odom and self.trans:
            self.tf_pub.sendTransform(self.trans)
            self.trans = None
            self.pub.publish(self.odom)
            self.odom = None

    def publish(self):
        trans = self.setupTransform(None)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose =  PoseWithCovariance()
        odom.twist = TwistWithCovariance()
        self.tf_pub.sendTransform(trans)
        self.pub.publish(odom)

def listVector3Conversion(l):
    vec = Vector3()
    vec.x = l[0]
    vec.y = l[1]
    vec.z = l[2]
    return vec

def vector3ListConversion(vec):
    return [vec.x, vec.y, vec.z ]


if __name__ == '__main__':
    rospy.init_node("zero_odom")
    pub = rospy.Publisher("/odom", Odometry, queue_size=5 )
    odom_faker = OdometryFaker()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom_faker.publish()
        rate.sleep()
