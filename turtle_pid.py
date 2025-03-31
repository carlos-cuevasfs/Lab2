#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi, radians

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al tópico de posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el tópico de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Configurar la frecuencia de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.last_error_x = 0
        self.error_accumulation_x = 0
        self.last_error_y = 0
        self.error_accumulation_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # Esta función se ejecuta cada vez que se actualiza la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_x_y(self, desired_x, desired_y):
        # Constantes PID para el movimiento lineal (X e Y)
        Kp = 1
        Ki = 0.01
        Kd = 0.1

        while not rospy.is_shutdown():
            # Calcular el error de posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Acumular errores para el término integral
            self.error_accumulation_x += error_x
            self.error_accumulation_y += error_y
            
            # Calcular las velocidades lineales usando PID
            vel_x = Kp * error_x + Ki * self.error_accumulation_x + Kd * (error_x - self.last_error_x)
            vel_y = Kp * error_y + Ki * self.error_accumulation_y + Kd * (error_y - self.last_error_y)
            
            # Guardar el error actual para la siguiente iteración
            self.last_error_x = error_x
            self.last_error_y = error_y
            
            # Crear un mensaje Twist para enviar comandos de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            twist_msg.linear.y = vel_y
            
            # Publicar el comando de movimiento
            self.velocity_publisher.publish(twist_msg)
            
            # Registrar la posición actual, el error y la velocidad
            rospy.loginfo("Posición actual x: %f, Error: %f, Velocidad lineal: %f", self.current_x, error_x, vel_x)
            rospy.loginfo("Posición actual y: %f, Error: %f, Velocidad lineal: %f", self.current_y, error_y, vel_y)
            
            # Verificar si se ha alcanzado la posición deseada
            if abs(error_x) < 0.1 and abs(error_y) < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar para la siguiente iteración
            self.rate.sleep()

    def move_turtle_to_desired_theta(self, desired_theta):
        # Constantes PD para el movimiento angular (rotación)
        Kp_theta = 0.3  # Ganancia proporcional pequeña
        Kd_theta = 0.1  # Ganancia derivativa pequeña

        while not rospy.is_shutdown():
            # Calcular el error de ángulo
            error_theta = desired_theta - self.current_theta
            
            # Normalizar el error de ángulo en el rango [-pi, pi]
            while error_theta > pi:
                error_theta -= 2 * pi
            while error_theta < -pi:
                error_theta += 2 * pi

            # Calcular la velocidad angular usando control PD (sin término integral)
            angular_z = Kp_theta * error_theta + Kd_theta * (error_theta - self.last_error_theta)
            
            # Crear un mensaje Twist para el movimiento angular
            twist_msg = Twist()
            twist_msg.angular.z = angular_z
            
            # Publicar el comando de velocidad angular
            self.velocity_publisher.publish(twist_msg)
            
            # Registrar la orientación actual, el error y la velocidad angular
            rospy.loginfo("Orientación actual: %f, Error: %f, Velocidad angular: %f", self.current_theta, error_theta, angular_z)

            # Verificar si se ha alcanzado la orientación deseada
            if abs(error_theta) < 0.05:  # Tolerancia mayor para evitar sobrepaso
                rospy.loginfo("Orientación deseada alcanzada")
                break

            # Guardar el error de ángulo actual para la siguiente iteración
            self.last_error_theta = error_theta
            
            # Esperar para la siguiente iteración
            self.rate.sleep()

    def get_desired_x_from_user(self):
        print("Ingrese la posición x deseada:")
        return float(input("Coordenada x: "))

    def get_desired_y_from_user(self):
        print("Ingrese la posición y deseada:")
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
            self.move_turtle_to_desired_x_y(desired_x, desired_y)

            # Rotar la tortuga a la orientación deseada
            self.move_turtle_to_desired_theta(desired_theta)

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
