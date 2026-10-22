#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class JustMove(Node):
    def __init__(self):
        super().__init__('just_move')
        
        self.pub = self.create_publisher(Twist, "cmd_vel", 50)
        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_cb, 10)

        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.first_run = True
        self.ideal_yaw = 0.0 
        self.turn_approx = 89.0

        # --- Configuração da Pista ---
        self.x_dist = 2.04  
        self.y_dist = 2.015
        self.x_desloc = 0.0
        self.curve_radius = 0.68 
        self.clockwise_mode = False 

        self.linear_vel = 0.4     
        self.yaw_vel = 0.3        

        # --- Ganhos do PID ---
        self.kp = 110
        self.ki = 0.15   
        self.kd = 0.1   
        
        # --- Variáveis do PID ---
        self.integral_error = 0.0
        self.last_error = 0.0
        self.max_integral = 2 

        self.movement_thread = threading.Thread(target=self.movement_loop)
        self.movement_thread.daemon = True
        self.movement_thread.start()

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle
        
    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = euler[2]

        if self.first_run:
            self.ideal_yaw = self.current_yaw
            self.first_run = False
    
    def move_straight(self, target_distance):
        cmd = Twist()
        cmd.linear.x = self.linear_vel

        start_x = self.current_x
        start_y = self.current_y

        self.integral_error = 0.0
        self.last_error = 0.0

        target_angle = self.ideal_yaw

        # self.last_error = self.normalize_angle(target_angle - self.current_yaw)

        last_time = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():

            current_time = self.get_clock().now().nanoseconds / 1e9
            dt = current_time - last_time

            if dt == 0:
                time.sleep(0.02)
                continue

            last_time = current_time

            dx = self.current_x - start_x
            dy = self.current_y - start_y

            dist_projected = dx * math.cos(target_angle) + dy * math.sin(target_angle)

            if dist_projected >= target_distance:
                break

            error = self.normalize_angle(self.current_yaw - target_angle)

            P = -self.kp * error

            self.integral_error += error * dt
            self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
            I = self.ki * self.integral_error

            derivative = (error - self.last_error) / dt
            D = self.kd * derivative

            self.last_error = error

            cmd.angular.z = P + I

            self.pub.publish(cmd)
            time.sleep(0.02) 
        
        self.stop(0.5)

    def move_turn_approx_90(self):
        cmd = Twist()
        rotation_rad = (math.pi / 180.0) * self.turn_approx
        
        if self.clockwise_mode:
            self.ideal_yaw -= rotation_rad
            target_omega = -abs(self.yaw_vel)
        else:
            self.ideal_yaw += rotation_rad
            target_omega = abs(self.yaw_vel)
            
        self.ideal_yaw = self.normalize_angle(self.ideal_yaw)

        target_linear = abs(target_omega * self.curve_radius)
        cmd.linear.x = target_linear
        cmd.angular.z = target_omega
        
        while rclpy.ok():
            error = self.normalize_angle(self.current_yaw - self.ideal_yaw)
            
            if abs(error) < 0.03: 
                break
                
            self.pub.publish(cmd)
            time.sleep(0.02)
            
        self.stop(1.0)

    def move_to_point_x(self, target_x):

        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = 0.0

        start_x = self.current_x

        while rclpy.ok():

            dist_moved_x = abs(self.current_x - start_x)

            if dist_moved_x >= target_x:
                break

            self.pub.publish(cmd)
            time.sleep(0.02)

        cmd.linear.x = 0.0
        self.pub.publish(cmd)

    def move_to_point_y(self, target_y):

        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = 0.0

        start_y = self.current_y

        while rclpy.ok():

            dist_moved_y = abs(self.current_y - start_y)

            if dist_moved_y >= target_y:
                break

            self.pub.publish(cmd)
            time.sleep(0.02)

        cmd.linear.x = 0.0
        self.pub.publish(cmd)  

    def stop(self, duration):
        cmd = Twist()
        self.pub.publish(cmd)
        time.sleep(duration)

    def movement_loop(self):
        self.get_logger().info("Aguardando odometria...")
        while self.first_run and rclpy.ok():
            time.sleep(0.1)
        
        self.get_logger().info("Iniciando circuito!")
        start_dist = self.x_dist - self.x_desloc

        while rclpy.ok():

            # self.move_to_point_x(3)
            # time.sleep(2)

            # self.move_turn_approx_90()

            # self.move_to_point_y(3)
            # time.sleep(2)

            # self.move_turn_approx_90()

            # self.move_to_point_x(3)
            # time.sleep(2)

            # self.move_turn_approx_90()

            # self.move_to_point_y(3)
            # time.sleep(2)

            # self.move_turn_approx_90()

            self.get_logger().info("Primeira reta!")
            self.move_straight(start_dist)
            self.move_turn_approx_90()
            
            self.get_logger().info("Segunda reta!")
            self.move_straight(self.y_dist)
            self.move_turn_approx_90()

            self.get_logger().info("Terceira reta!")
            self.move_straight(self.x_dist)
            self.move_turn_approx_90()
            
            self.get_logger().info("Quarta reta!")
            self.move_straight(self.y_dist)
            self.move_turn_approx_90()

            self.get_logger().info("Finalizando!")
            self.move_straight(self.x_dist)
            self.move_turn_approx_90()

def main(args=None):
    rclpy.init(args=args)
    just_move_node = JustMove()
    try:
        rclpy.spin(just_move_node)
    except KeyboardInterrupt:
        pass
    finally:
        just_move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()