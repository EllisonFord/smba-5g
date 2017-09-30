from StringIO import StringIO

import rospy
from smba_core.msg import *

from smba_core._smba_converter_wrapper_cpp import *


class SMBAConverter(object):
    def __init__(self):
        self._smba_converter = SMBAConverterWrapper()

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        return msg.deserialize(str_msg)

    def convert_to_polygon(self, a):
        """Convert two std_mgs/Int64 messages

        Return output std_msgs/smba_polygon_list instance.

        Parameters
        ----------
        - input: input smba_core/smba_object_list instance.
        """
        if not isinstance(a, smba_object_list):
            rospy.ROSException('Input is not a smba_core/Int64')
        str_a = self._to_cpp(a)
        str_out = self._smba_converter.convert_to_polygon(str_a)
        return self._from_cpp(str_out, smba_polygon_list)

    def transform_object_list(self, a, x, y, theta):
        """Convert two std_mgs/Int64 messages

        Return output std_msgs/smba_polygon_list instance.

        Parameters
        ----------
        - input: input smba_core/smba_object_list instance.
        """
        if not isinstance(a, smba_object_list):
            rospy.ROSException('Input is not a smba_core/Int64')
        str_a = self._to_cpp(a)
        str_out = self._smba_converter.transform_object_list(str_a, x, y, theta)
        return self._from_cpp(str_out, smba_object_list)

    def transform_object_list_forward(self, a):
        """Convert two std_mgs/Int64 messages

        Return output std_msgs/smba_polygon_list instance.

        Parameters
        ----------
        - input: input smba_core/smba_object_list instance.
        """
        if not isinstance(a, smba_object_list):
            rospy.ROSException('Input is not a smba_core/Int64')
        str_a = self._to_cpp(a)
        str_out = self._smba_converter.transform_object_list_forward(str_a)
        return self._from_cpp(str_out, smba_object_list)

    def transform_object_list_backward(self, a):
        """Convert two std_mgs/Int64 messages

        Return output std_msgs/smba_polygon_list instance.

        Parameters
        ----------
        - input: input smba_core/smba_object_list instance.
        """
        if not isinstance(a, smba_object_list):
            rospy.ROSException('Input is not a smba_core/Int64')
        str_a = self._to_cpp(a)
        str_out = self._smba_converter.transform_object_list_backward(str_a)
        return self._from_cpp(str_out, smba_object_list)

    def set_transformation_matrix(self, x, y, theta):
        """Set TRmatrix

        Return output std_msgs/smba_polygon_list instance.

        Parameters
        ----------
        - input: input smba_core/smba_object_list instance.
        """
        return self._smba_converter.set_transformation_matrix(float(x), float(y), float(theta))
