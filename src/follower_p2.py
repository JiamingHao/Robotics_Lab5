#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.twist = Twist()
        self.seen_red_stop = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20

        # yellow hue = 29-30
        lower_yellow = numpy.array([20,  10,  10])
        upper_yellow = numpy.array([40, 255, 250])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow[0:search_top, 0:w] = 0
        mask_yellow[search_bot:h, 0:w] = 0

        # red hue = 0
        lower_red = numpy.array([ 170,  10,  10])
        upper_red = numpy.array([180, 255, 250])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        lower_red = numpy.array([ 0,  10,  10])
        upper_red = numpy.array([10, 255, 250])
        mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red, upper_red), mask_red)
        mask_red[0:search_top, 0:w] = 0

        # green hue = 60
        lower_green = numpy.array([50,  10,  10])
        upper_green = numpy.array([70, 255, 250])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_green[0:search_top, 0:w] = 0

        # blue hue = 120
        lower_blue = numpy.array([110,  10,  10])
        upper_blue = numpy.array([130, 255, 250])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_blue[0:search_top, 0:w] = 0

        linear_velocity = 0.3
        turning_angle = 7 / 100
        if numpy.sum(mask_red) > 0:
            self.seen_red_stop = True

            M = cv2.moments(mask_yellow)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                # BEGIN CONTROL
                err = cx - w/2
                self.twist.linear.x = linear_velocity
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
            cv2.imshow("window", mask_yellow)
        elif numpy.sum(mask_blue) > 0:
            self.twist.linear.x = linear_velocity
            self.twist.angular.z = - turning_angle
            self.cmd_vel_pub.publish(self.twist)
            cv2.imshow("window", mask_blue)
        elif numpy.sum(mask_green) > 0:
            self.twist.linear.x = linear_velocity
            self.twist.angular.z = turning_angle
            self.cmd_vel_pub.publish(self.twist)
            cv2.imshow("window", mask_green)
        else:
            if self.seen_red_stop:
                exit()

            M = cv2.moments(mask_yellow)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                # BEGIN CONTROL
                err = cx - w/2
                self.twist.linear.x = linear_velocity
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
            cv2.imshow("window", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
