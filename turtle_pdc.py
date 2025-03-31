#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi, sqrt

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al tópico de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el tópico de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Establecer la tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # Esta función se llama cada vez que se actualiza la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_x_y_theta(self, desired_x, desired_y, desired_theta):
        # Ganancias Proporcionales y Derivativas para el controlador (ajustables)
        Kp_linear = 1.0  # Ganancia proporcional para la velocidad lineal
        Kd_linear = 0.1  # Ganancia derivativa para la velocidad lineal
        Kp_angular = 0.8  # Ganancia proporcional para la velocidad angular
        Kd_angular = 0.1  # Ganancia derivativa para la velocidad angular

        # 1. Primero, moverse hacia la posición deseada (traslación)
        while not rospy.is_shutdown():
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y

            # Calcular la distancia a la posición deseada
            distance = sqrt(error_x**2 + error_y**2)

            # Calcular las velocidades considerando el control proporcional y derivativo
            vel_x = Kp_linear * error_x + Kd_linear * (error_x - self.last_error_x)
            vel_y = Kp_linear * error_y + Kd_linear * (error_y - self.last_error_y)

            # Publicar comandos de velocidad
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            self.velocity_publisher.publish(twist_msg)

            # Mostrar información en la consola
            rospy.loginfo("Moviéndose a la posición: x = %f, y = %f, Error x = %f, Error y = %f, Vel x = %f, Vel y = %f", 
                          self.current_x, self.current_y, error_x, error_y, vel_x, vel_y)

            if distance < 0.1:
                rospy.loginfo("Posición alcanzada")
                break

            # Actualizar el error anterior para la siguiente iteración
            self.last_error_x = error_x
            self.last_error_y = error_y

            self.rate.sleep()

        # 2. Luego, rotar la tortuga hasta el ángulo deseado (rotación en z)
        while not rospy.is_shutdown():
            error_theta = desired_theta - self.current_theta

            # Asegurar que el error angular esté dentro del rango [-pi, pi]
            while error_theta > pi:
                error_theta -= 2 * pi
            while error_theta < -pi:
                error_theta += 2 * pi

            # Calcular la velocidad angular considerando el control proporcional y derivativo
            angular_z = Kp_angular * error_theta + Kd_angular * (error_theta - self.last_error_theta)

            # Publicar el comando de velocidad angular
            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            self.velocity_publisher.publish(twist_msg)

            # Mostrar información en la consola
            rospy.loginfo("Corrigiendo orientación: θ = %f, Error θ = %f, Velocidad angular = %f", 
                          self.current_theta, error_theta, angular_z)

            if abs(error_theta) < 0.05:  # Si el error de ángulo es lo suficientemente pequeño, detener la rotación
                rospy.loginfo("Orientación alcanzada")
                break

            # Actualizar el error anterior para la siguiente iteración
            self.last_error_theta = error_theta

            self.rate.sleep()

    def get_desired_x_from_user(self):
        print("Ingrese la posición deseada en x:")
        return float(input("Coordenada x: "))
        
    def get_desired_y_from_user(self):
        print("Ingrese la posición deseada en y:")
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

            # Mover la tortuga a la posición y orientación deseadas
            self.move_turtle_to_desired_x_y_theta(desired_x, desired_y, desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
