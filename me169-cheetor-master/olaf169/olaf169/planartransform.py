#!/usr/bin/env python
#
#   PlanarTransform Class
#
#   Class to hold and process planar transforms, containing the state
#   (x,y,theta).  The angle is stored as the 3D quarternions would be,
#   i.e. as qz = sin(theta/2) and qw = cos(theta/2).  Methods are:
#
#     pt = PlanarTransform(px, py, qz, qw)
#     pt = PlanarTransform.unity()
#     pt = PlanarTransform.basic(x, y, theta)
#     pt = PlanarTransform.fromPose(pose)
#     pt = PlanarTransform.fromTransform(transform)
#
#     pose     = pt.toPose()            Convert to Pose
#     tranform = pt.toTransform()       Convert to Transform
#
#     pt2 = pt1.scale(scale)            Scale is the same
#         = pt1 ** scale                Exponent
#     pt2 = pt1.inv()                   Invert (scale/exponent of -1)
#
#     pt3 = pt1 * pt2                   Concatenate
#
#       for example pt == (pt ** 0.5) * (pt ** 0.5)
#
#     (x,y) = pt.inParent(x,y)          Transform a point to parent frame
#
#     x     = pt.x()                    Extract x
#     y     = pt.y()                    Extract y
#     sin   = pt.sin()                  Extract sin(theta)
#     cos   = pt.cos()                  Extract cos(theta)
#     theta = pt.theta()                Compute theta
#
from math import pi, sin, cos, atan2

from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Transform, Vector3


#
#  PlanarTransform Class Definition
#
class PlanarTransform:
    def __init__(self, px, py, qz, qw):
        self.px = px            # X coordinate
        self.py = py            # Y coordinate
        self.qz = qz            # sin(theta/2)
        self.qw = qw            # cos(theta/2)


    # Processing:
    def inParent(self, x, y):
        (s, c) = (self.sin(), self.cos())
        return (self.px + c * x - s * y,
                self.py + s * x + c * y)

    def __mul__(self, next):
        (x,y) = self.inParent(next.px, next.py)
        return PlanarTransform(x, y,
                               self.qz * next.qw + self.qw * next.qz,
                               self.qw * next.qw - self.qz * next.qz)

    def inv(self):
        (s, c) = (self.sin(), self.cos())
        return PlanarTransform(-c * self.px - s * self.py,
                                s * self.px - c * self.py,
                               -self.qz, self.qw)

    def scale(self, scale):
        return self.__pow__(scale)
    def __pow__(self, scale):
        if self.qz == 0.0:
            return PlanarTransform(self.px*scale, self.py*scale, 0.0, 1.0)
        theta = self.theta()
        u = 0.5 * (self.qz + sin(theta * (scale-0.5))) / self.qz
        v = 0.5 * (self.qw - cos(theta * (scale-0.5))) / self.qz
        return PlanarTransform.basic(self.px*u - self.py*v,
                                     self.py*u + self.px*v,
                                     theta * scale)

    # Extraction:
    def x(self):
        return (self.px)

    def y(self):
        return (self.py)

    def sin(self):
        return (2.0 * self.qz * self.qw)

    def cos(self):
        return (self.qw**2 - self.qz**2)

    def theta(self):
        return atan2(self.sin(), self.cos())

    # Representation:
    def __repr__(self):
        return ("<px:%6.3f, py:%6.3f, qz:%6.3f, qw:%6.3f>"
                % (self.px, self.py, self.qz, self.qw))

    def __str__(self):
        return ("x %6.3fm, y %6.3fm, theta %7.3fdeg"
                % (self.px, self.py, self.theta() * 180.0/pi))

    # Convert to/from Pose and Transform:
    def toPose(self):
        return Pose(
            position    = Point(x=self.px, y=self.py, z=0.0),
            orientation = Quaternion(x=0.0, y=0.0, z=self.qz, w=self.qw))

    def toTransform(self):
        return Transform(
            translation = Vector3(x=self.px, y=self.py, z=0.0),
            rotation    = Quaternion(x=0.0, y=0.0, z=self.qz, w=self.qw))

    @classmethod
    def unity(cls):
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def basic(cls, x, y, theta):
        return cls(x, y, sin(0.5*theta), cos(0.5*theta))

    @classmethod
    def fromPose(cls, pose):
        assert ((abs(pose.orientation.x) < 1e-9) and
                (abs(pose.orientation.y) < 1e-9)), "Pose not planar"
        return cls(pose.position.x, pose.position.y,
                   pose.orientation.z, pose.orientation.w)

    @classmethod
    def fromTransform(cls, transform):
        assert ((abs(transform.rotation.x) < 1e-9) and
                (abs(transform.rotation.y) < 1e-9)), "Transform not planar"
        return cls(transform.translation.x, transform.translation.y,
                   transform.rotation.z, transform.rotation.w)


#
#   Main/Test Function
#
def main(args=None):
    # Pick a planar transform
    pt = PlanarTransform.basic(0, 2, pi)

    # Compute some "fractional" transforms.
    pt2 = pt ** (1/2)
    pt3 = pt ** (1/3)
    pt4 = pt ** (1/4)
    
    # Report.
    print("pt1:              ", pt)
    print("pt2 = pt1 ** 1/2: ", pt2)
    print("pt3 = pt1 ** 1/3: ", pt3)
    print("pt4 = pt1 ** 1/4: ", pt4)
    print("pt2 * pt2:        ", pt2*pt2)
    print("pt3 * pt3 * pt3:  ", pt3*pt3*pt3)
    print("pt4 * pt2 * pt4:  ", pt4*pt2*pt4)

if __name__ == "__main__":
    main()
