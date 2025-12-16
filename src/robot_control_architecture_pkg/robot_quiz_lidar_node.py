#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def deg2rad(d): return d * math.pi / 180.0
DIR2ANGLE_RAD = {'A': deg2rad(0), 'B': deg2rad(-90), 'C': deg2rad(180), 'D': deg2rad(90)}

class QuizWithLidarNode(Node):
    def __init__(self):
        super().__init__('quiz_with_lidar_node')

        
        self.declare_parameter('laser_topic', '/scan')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('default_distance_m', 1.5)
        self.declare_parameter('stop_wait_s', 5.0)
        self.declare_parameter('sector_width_deg', 30.0)
        self.declare_parameter('person_min_m', 0.3)
        self.declare_parameter('person_max_m', 3.0)
        self.declare_parameter('tts_correct', 'Congratulations, you get the correct answer!')
        self.declare_parameter('tts_incorrect', 'Sorry, no one gets the correct answer.')
        self.declare_parameter('tts_repeat_window_s', 10.0)
        self.declare_parameter('tts_repeat_period_s', 0.8)

        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.say_pub = self.create_publisher(String, 'say_text', 10)
        laser_topic = self.get_parameter('laser_topic').value
        self.scan_sub = self.create_subscription(LaserScan, laser_topic, self.on_scan, qos_profile_sensor_data)
        self.answer_sub = self.create_subscription(String, 'quiz_correct', self.on_answer, 10)

        
        self.state = 'IDLE'
        self.phase_end = self.get_clock().now()
        self.pending_answer = None
        self.last_scan = None
        self.lin_spd = float(self.get_parameter('linear_speed').value)
        self.ang_spd = float(self.get_parameter('angular_speed').value)
        self.turn_sign = 1.0
        self.turn_time = 0.0
        self.forward_time = 0.0

       
        self.tts_end = self.get_clock().now()
        self.next_tts = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.loop)

    def on_scan(self, msg): self.last_scan = msg

    def on_answer(self, msg):
        if msg.data not in DIR2ANGLE_RAD:
            self.get_logger().warn(f"Invalid answer {msg.data}")
            return
        self.pending_answer = msg.data
        if self.state == 'IDLE': self.plan_for_answer()

    def plan_for_answer(self):
        ans = self.pending_answer
        ang = DIR2ANGLE_RAD[ans]
        self.turn_sign = 1.0 if ang >= 0 else -1.0
        self.turn_time = abs(ang) / max(self.ang_spd,1e-3)
        dist = self.estimate_distance(ans) or float(self.get_parameter('default_distance_m').value)
        self.forward_time = dist / max(self.lin_spd,1e-3)
        self.phase_end = self.get_clock().now() + Duration(seconds=self.turn_time)
        self.state = 'ORIENT'
        self.get_logger().info(f"{ans}: turn {self.turn_time:.1f}s, forward {self.forward_time:.1f}s")

    def estimate_distance(self, ans):
        if self.last_scan is None: return None
        center = {'A':0,'B':-90,'C':180,'D':90}[ans]
        half = float(self.get_parameter('sector_width_deg').value)/2
        min_deg,max_deg=center-half,center+half
        scan=self.last_scan
        ang_min,ang_inc=scan.angle_min,scan.angle_increment
        best=None
        for i,r in enumerate(scan.ranges):
            if r<=0 or math.isinf(r): continue
            ang=math.degrees(ang_min+i*ang_inc)
            if min_deg<=ang<=max_deg:
                best=r if best is None or r<best else best
        if not best: return None
        if not(float(self.get_parameter('person_min_m').value)<=best<=float(self.get_parameter('person_max_m').value)):
            return None
        return best

    def loop(self):
        now=self.get_clock().now()
        cmd=Twist()
        stop_wait=float(self.get_parameter('stop_wait_s').value)

        if self.state=='IDLE': pass
        elif self.state=='ORIENT':
            cmd.angular.z=self.turn_sign*self.ang_spd
            if now>self.phase_end:
                self.phase_end=now+Duration(seconds=self.forward_time)
                self.state='ADVANCE'
        elif self.state=='ADVANCE':
            cmd.linear.x=self.lin_spd
            if now>self.phase_end:
                self.tts_end=now+Duration(seconds=float(self.get_parameter('tts_repeat_window_s').value))
                self.next_tts=now
                phrase=self.get_parameter('tts_phrase').value
                self.say_pub.publish(String(data=phrase))
                self.phase_end=now+Duration(seconds=stop_wait)
                self.state='STOP_AT_GOAL'
        elif self.state=='STOP_AT_GOAL':
            if now<=self.tts_end and now>=self.next_tts:
                phrase=self.get_parameter('tts_phrase').value
                self.say_pub.publish(String(data=phrase))
                self.next_tts=now+Duration(seconds=float(self.get_parameter('tts_repeat_period_s').value))
            if now>self.phase_end:
                self.phase_end=now+Duration(seconds=math.pi/self.ang_spd)
                self.state='TURN_BACK'
        elif self.state=='TURN_BACK':
            cmd.angular.z=self.ang_spd
            if now>self.phase_end:
                self.phase_end=now+Duration(seconds=self.forward_time)
                self.state='RETURN'
        elif self.state=='RETURN':
            cmd.linear.x=self.lin_spd
            if now>self.phase_end:
                self.phase_end=now+Duration(seconds=math.pi/self.ang_spd)
                self.state='FACE_FORWARD'
        elif self.state=='FACE_FORWARD':
            cmd.angular.z=self.ang_spd
            if now>self.phase_end:
                self.state='IDLE'; self.pending_answer=None
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node=QuizWithLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()