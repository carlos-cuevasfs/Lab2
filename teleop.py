#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import time

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controla la tortuga con las teclas:")
    print("  W -> Mover hacia arriba")
    print("  D -> Mover hacia la derecha")
    print("  S -> Mover hacia abajo")
    print("  A -> Mover hacia la izquierda")
    print("  R -> Rotar")
    print("  C -> Dibujar cuadrado")
    print("  T -> Dibujar triángulo")
    print("  Q -> Salir")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
       
        if key == 'd':
            msg.linear.x = 2.0  # Avanza en X
        elif key == 'w':
            msg.linear.y = 2.0  # Avanza en Y
        elif key == 'a':
            msg.linear.x = -2.0 # Avanza en reversa en X
        elif key == 's':
            msg.linear.y = -2.0 # Avanza en reversa en Y
        elif key == 'r':
            msg.angular.z = 2.0
            
        # Hacer un cuadrado con una tecla
        elif key == 'c':
            msg.linear.x = 2.0
            msg.linear.y = 0.0
            pub.publish(msg)
            time.sleep(2)

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            pub.publish(msg)
            time.sleep(0.5)

            msg.linear.x = 0.0
            msg.linear.y = 2.0
            pub.publish(msg)
            time.sleep(2)

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            pub.publish(msg)
            time.sleep(0.5)

            msg.linear.x = -2.0
            msg.linear.y = 0.0
            pub.publish(msg)
            time.sleep(2)

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            pub.publish(msg)
            time.sleep(0.5)

            msg.linear.x = 0.0
            msg.linear.y = -2.0
            pub.publish(msg)
            time.sleep(2)

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            pub.publish(msg)

        elif key == 't':
            # Lado 1 del triángulo
            msg.linear.x = 2.0
            msg.angular.z = 0.0
            pub.publish(msg)
            time.sleep(2)

            # Detener
            msg.linear.x = 0.0
            pub.publish(msg)
            time.sleep(0.5)

            # Girar 120°
            msg.angular.z = 1.0472 * 2  # Velocidad angular positiva
            pub.publish(msg)
            time.sleep(2.1)  # Ajuste de tiempo para 120°

            # Detener giro
            msg.angular.z = 0.0
            pub.publish(msg)
            time.sleep(0.5)

            # Lado 2 del triángulo
            msg.linear.x = 2.0
            pub.publish(msg)
            time.sleep(2)

            # Detener
            msg.linear.x = 0.0
            pub.publish(msg)
            time.sleep(0.5)

            # Girar 120°
            msg.angular.z = 1.055 * 2
            pub.publish(msg)
            time.sleep(2.1)

            # Detener giro
            msg.angular.z = 0.0
            pub.publish(msg)
            time.sleep(0.5)

            # Lado 3 del triángulo
            msg.linear.x = 2.0
            pub.publish(msg)
            time.sleep(2)

            # Detener movimiento
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            pub.publish(msg)

        elif key == 'q':  
            print("Saliendo...")
            break  # Sale del loop
        
        pub.publish(msg)

if __name__ == '__main__':
    main()
