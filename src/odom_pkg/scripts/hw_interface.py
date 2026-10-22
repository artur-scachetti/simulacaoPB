#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import serial
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class HWInterface(Node):
    def __init__(self):
        super().__init__('hw_interface_node')
    #Definimos os Subscrips para receber oque chega
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, 10)

        # ADICIONEI ESSE SUBSCRIBER QUE OUVE O ""ENCODER DO GAZEBO""
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.cb_joint_states, 10)
    
        self.encoder_sub = self.create_subscription(String, 'encoder', self.cb_encoder, 10)
    
        #Definimos em qual topico vamos acessar 
        self.odom_pub = self.create_publisher(Odometry, "odom", 50)
        self.tf_broadcaster = TransformBroadcaster(self)

        #Variaveis que viam do arquivo yaml, não sei como sera no projeto
        self.raio_roda = 0.1   
        self.largura_roda = 0.35 
        self.max_velocidade = 100
        self.min_velocidade = 0


        #Parametros usados agora
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
    
        self.tempo_anterior = self.get_clock().now()

        self.velocidade_direita_comando = 0.0
        self.velocidade_esquerda_comando = 0.0

        self.velocidade_linear_atual = 0.0
        self.velocidade_angular_atual = 0.0

        self.watchdog_timer = self.create_timer(0.2, self.cb_cmd_timeout)

        # try:
            
        #     self.porta_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        #     self.get_logger().info("ESP32 conectado com sucesso na Serial!")

        # except serial.SerialException as e:
             
        #     self.get_logger().error(f"Erro ao conectar no ESP32: {e}")
        #     self.porta_serial = None


    #Aplica a cinemática inversa
    def cb_cmd_vel(self, msg):
        vx = msg.linear.x
        omega = msg.angular.z

        self.velocidade_direita_comando = (vx + (omega * self.largura_roda) / 2.0) / self.raio_roda
        self.velocidade_esquerda_comando = (vx - (omega * self.largura_roda) / 2.0) / self.raio_roda

        self.watchdog_timer.reset()

        self.enviar_comandos_motores() #envia para a esp

    def cb_joint_states(self, msg):
        try:

            idx_left = next(i for i, name in enumerate(msg.name) if 'left_wheel_joint' in name)
            idx_right = next(i for i, name in enumerate(msg.name) if 'right_wheel_joint' in name)
            
            omega_left = msg.velocity[idx_left]
            omega_right = msg.velocity[idx_right]
            
            v_left_ms = omega_left * self.raio_roda
            v_right_ms = omega_right * self.raio_roda

            self.velocidade_linear_atual = (v_right_ms + v_left_ms) / 2.0
            self.velocidade_angular_atual = (v_right_ms - v_left_ms) / self.largura_roda
            
            if (self.velocidade_linear_atual >= self.max_velocidade) or (self.velocidade_linear_atual <= -self.max_velocidade):
                self.velocidade_linear_atual = 0.0

            
            self.update_odometry()
            
        except StopIteration:
            self.get_logger().error(f"Juntas não encontradas! O simulador enviou: {msg.name}")
        except IndexError:
            self.get_logger().error("Juntas encontradas, mas a velocidade não foi fornecida pelo simulador!")

    def cb_encoder(self, msg):
        # Transforma a rotação (rad/s) em velocidade linear da roda (m/s)
        v_right_ms = msg.encoder_right * self.raio_roda
        v_left_ms = msg.encoder_left * self.raio_roda

        # Cinemática Direta: Velocidade real do chassi
        self.velocidade_linear_atual = (v_right_ms + v_left_ms) / 2.0
        self.velocidade_angular_atual = (v_right_ms - v_left_ms) / self.largura_roda

        # Filtro de segurança (Glitch do sensor)
        if (self.velocidade_linear_atual >= self.max_velocidade) or (self.velocidade_linear_atual <= self.min_velocidade):
            self.velocidade_linear_atual = 0.0
        
        self.update_odometry() #atualiza a odometria
    
    def enviar_comandos_motores(self):
            # # Só tenta enviar se a porta serial estiver aberta e funcionando
            # if self.porta_serial is not None and self.porta_serial.is_open:
                
            #     # Formata a string: "vel_esq,vel_dir\n" (Limitado a 3 casas decimais)
            #     # Exemplo de saída: "12.500,12.500\n"
            #     comando_str = f"{self.velocidade_esquerda_comando:.3f},{self.velocidade_direita_comando:.3f}\n"
                
            #     # Converte o texto para bytes e envia pelo cabo
            #     self.porta_serial.write(comando_str.encode('utf-8'))
            pass

    def update_odometry(self):
        tempo_agora = self.get_clock().now()
        dt = (tempo_agora - self.tempo_anterior).nanoseconds / 1e9
        self.tempo_anterior = tempo_agora

        delta_x = self.velocidade_linear_atual * math.cos(self.odom_yaw) * dt
        delta_y = self.velocidade_linear_atual * math.sin(self.odom_yaw) * dt
        delta_yaw = self.velocidade_angular_atual * dt

        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_yaw += delta_yaw

        q_z = math.sin(self.odom_yaw / 2.0)
        q_w = math.cos(self.odom_yaw / 2.0)

        t = TransformStamped()
        t.header.stamp = tempo_agora.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = tempo_agora.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        
        odom.twist.twist.linear.x = self.velocidade_linear_atual
        odom.twist.twist.angular.z = self.velocidade_angular_atual
        
        self.odom_pub.publish(odom)

    def cb_cmd_timeout(self):
        # self.get_logger().warn("Sinal de controle perdido! Parando motores...")
        # self.velocidade_direita_comando = 0.0
        # self.velocidade_esquerda_comando = 0.0
        # self.enviar_comandos_motores()

        pass

def main(args=None):
    rclpy.init(args=args)
    interface = HWInterface()
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()