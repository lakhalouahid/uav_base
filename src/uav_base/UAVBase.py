import rospy
import math
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from nav_msgs.msg import Odometry






class UAVBase():
    def __init__(self, cmd_vel_topic, max_speed=Vector3(5, 5, 5)):
        self.position = Point()
        self.orientation = Quaternion()
        self.linear_velocity = Vector3()
        self.angular_velocity = Vector3()
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.max_speed = max_speed

    def update_state(self, state):
        self.position = state.pose.pose.position
        self.orientation = state.pose.pose.orientation
        self.linear_velocity = state.twist.twist.linear
        self.angular_velocity = state.twist.twist.angular

    def move(self, position, orientation=Quaternion(0, 0, 0, 1), interval=1):
        ispositioned = False
        while not ispositioned:
            if self.dist_position(self, position, 3e-2):
                ispositioned = True

            dx, dy, dz = self.position.x - position.x, self.position.y - position.y, self.position.z - position.z
            dx, dy, dz = self.sym_clip(dx, self.max_speed.x), self.sym_clip(dy, self.max_speed.y), self.sym_clip(dz, self.max_speed.z)

            linear_velocity = Vector3(-dx / (1.5*interval), -dy / (1.5*interval), -dz / (1.5*interval))
            print(linear_velocity)

            self.cmd_vel(Twist(linear_velocity, Vector3()), interval)

        self.cmd_vel(Twist(Vector3(), Vector3()), interval)
    
    def pose_cmd(self, position, orientation=Quaternion()):
        self.move(position, orientation)

    def cmd_vel(self, twist, interval):
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(interval)

    @staticmethod
    def dist_position(uav, act_position, delta=1e-2):
        dx = abs(uav.position.x - act_position.x)
        dy = abs(uav.position.y - act_position.y)
        dz = abs(uav.position.z - act_position.z)
        speed = math.sqrt(uav.linear_velocity.x * uav.linear_velocity.x + uav.linear_velocity.y * uav.linear_velocity.y + uav.linear_velocity.z * uav.linear_velocity.z)
        return max([dx, dy, dz]) <= delta and speed <= 0.5

    @staticmethod
    def sym_clip(val, clip):
        return min(max(val, -clip), clip)
