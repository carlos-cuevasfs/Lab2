#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi, sqrt

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_x_y_theta(self, desired_x, desired_y, desired_theta):
        # Constantes de proporcionalidad del controlador (ajustables)
        Kp_linear = 0.8  # Ganancia proporcional para la velocidad lineal
        Kp_angular = 0.3 # Reducida para evitar oscilaciones

        # 1. Primero, mover hacia la posición deseada
        while not rospy.is_shutdown():
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y

            distance = sqrt(error_x**2 + error_y**2)
            vel_x = Kp_linear * error_x
            vel_y = Kp_linear * error_y

            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y

            self.velocity_publisher.publish(twist_msg)

            rospy.loginfo("Moviendo a la posición: x = %f, y = %f, Error x = %f, Error y = %f, Velocidad x = %f, Velocidad y = %f", 
                          self.current_x, self.current_y, error_x, error_y, vel_x, vel_y)

            if distance < 0.05:
                rospy.loginfo("Posición alcanzada")
                break

            self.rate.sleep()

        # 2. Luego, orientar la tortuga hacia el ángulo deseado
        while not rospy.is_shutdown():
            error_theta = desired_theta - self.current_theta

            while error_theta > pi:
                error_theta -= 2 * pi
            while error_theta < -pi:
                error_theta += 2 * pi

            angular_z = Kp_angular * error_theta

            twist_msg = Twist()
            twist_msg.angular.z = angular_z

            self.velocity_publisher.publish(twist_msg)

            rospy.loginfo("Corrigiendo orientación: θ = %f, Error θ = %f, Velocidad angular = %f", 
                          self.current_theta, error_theta, angular_z)

            if abs(error_theta) < 0.05:  # Si el ángulo es suficientemente pequeño, detener la rotación
                rospy.loginfo("Orientación alcanzada")
                break

            self.rate.sleep()

    def get_desired_x_from_user(self):
        print("Ingrese la posición deseada en el eje x:")
        return float(input("Coordenada x: "))
        
    def get_desired_y_from_user(self):
        print("Ingrese la posición deseada en el eje y:")
        return float(input("Coordenada y: "))
    
    def get_desired_theta_from_user(self):
        print("Ingrese la orientación deseada en theta (radianes):")
        return float(input("Theta: "))
        
    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x = self.get_desired_x_from_user()
            desired_y = self.get_desired_y_from_user()
            desired_theta = self.get_desired_theta_from_user()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_x_y_theta(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
