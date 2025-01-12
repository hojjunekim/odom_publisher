import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub_30hz = self.create_publisher(Odometry, 'odom_30hz', 10)
        self.odom_pub_30hz_rel = self.create_publisher(Odometry, 'odom_30hz_rel', 10)
        self.odom_pub_100hz = self.create_publisher(Odometry, 'odom_100hz', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.hz1 = 30
        self.hz2 = 30
        self.timer_30hz = self.create_timer(1/self.hz1, self.publish_odom_30hz)
        self.timer_100hz = self.create_timer(1/self.hz2, self.publish_odom_100hz)
        self.timer_gt = self.create_timer(1/30, self.publish_gt_tf)

        self.x1 = 0.0
        self.y1 = 0.0

        self.x2 = 0.0
        self.y2 = 0.0
        self.theta1 = -math.pi/2
        self.theta2 = -math.pi/2
        self.theta1_lost = -math.pi/2

        self.radius = 2.0  # Radius of the circular trajectory
        self.angular_velocity = 1.0  # Radians per second
        
        self.noise_t_var = 0.1
        self.noise_r_var = 0.1
        self.noise_covar = 0.5

        self.drift_x = 0.0
        self.drift_y = 0.0
        
        self.drift_var = 0.03
        self.drift_theta = 0.1
        self.drift_covar = 0.1

        self.lost = False
        self.count1 = 0

    def publish_odom_30hz(self):
        self.count1 += 1
        self.publish_odom('30hz', True)

    def publish_odom_100hz(self):
        self.publish_odom('100hz', False)

    def publish_gt_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "gt"
        t.transform.translation.x = self.x2
        t.transform.translation.y = self.y2
        t.transform.rotation.z = math.sin((self.theta2 + math.pi/2) / 2.0)
        t.transform.rotation.w = math.cos((self.theta2 + math.pi/2) / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self, frame_id_suffix, add_gaussian_noise):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = f"odom"
        odom.child_frame_id = f"{frame_id_suffix}"
        
        
        if add_gaussian_noise:
            self.theta1 += self.angular_velocity * 1/self.hz1

            if self.lost:
                self.theta1_lost += self.angular_velocity * 1/self.hz1
                theta = self.theta1_lost
            else:
                theta = self.theta1

            self.x1 = self.radius * math.cos(theta)
            self.y1 = self.radius * (math.sin(theta) + 1)

            noise_x = np.random.normal(0, self.noise_t_var)
            noise_y = np.random.normal(0, self.noise_t_var)
            noise_theta = np.random.normal(0, self.noise_r_var)
            odom.pose.pose.position.x = self.x1 + noise_x
            odom.pose.pose.position.y = self.y1 + noise_y
            odom.pose.pose.orientation.z = math.sin((theta + math.pi/2 + noise_theta) / 2.0)
            odom.pose.pose.orientation.w = math.cos((theta + math.pi/2 + noise_theta) / 2.0)
            
            # Set covariance for 30Hz odom
            odom.pose.covariance = [self.noise_covar, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, self.noise_covar, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
            if self.count1 < 5 * self.hz1:
                if self.lost:
                    self.publish_odom_tf(self.odom_pub_30hz_rel, odom)
                else:
                    self.publish_odom_tf(self.odom_pub_30hz, odom)
            elif self.count1 > 0:
                self.count1 = - self.hz1
                if self.lost:
                    self.lost = False
                else:
                    self.lost = True
                    self.theta1_lost = -math.pi/2
        else:
            # Add small drift noise for 100Hz odom
            self.theta2 += self.angular_velocity * 1/self.hz2
            self.x2 = self.radius * math.cos(self.theta2)
            self.y2 = self.radius * (math.sin(self.theta2) + 1)

            self.drift_x += np.random.normal(0, self.drift_var)
            self.drift_y += np.random.normal(0, self.drift_var)
            odom.pose.pose.position.x = self.x2 + self.drift_x
            odom.pose.pose.position.y = self.y2 + self.drift_y - 4
            odom.pose.pose.orientation.z = math.sin((self.theta2 + math.pi/2 + self.drift_theta) / 2.0)
            odom.pose.pose.orientation.w = math.cos((self.theta2 + math.pi/2 + self.drift_theta) / 2.0)
            
            # Set covariance for 100Hz odom
            odom.pose.covariance = [self.drift_covar, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, self.drift_covar, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01]  # Small variance for orientation
            self.publish_odom_tf(self.odom_pub_100hz, odom)

    def publish_odom_tf(self, publisher, odom):
        publisher.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
