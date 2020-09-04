#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import os
import sys
import threading
from image_detection import main
from Main import chama


class Thread_Terminal(threading.Thread):
    def __init__(self, threadID, name, sleep, cust_type):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sleep = sleep
        self.cust_type = cust_type

        # chama os metodos que precisas...

        self.main = terminal()

    def returnTheData(self):
        return self.main


class Hardware():
    class Motors():
        class AllMotors():

            def power_on_all_motors(self):
                print(" os motores foram ligados")

        class Motor1():

            def power_motor(self):
                print("o motor 1 foi ligado!!")

        class Motor2():

            def power_motor(self):
                print("o motor 2 foi ligado!!")

        class Motor3():

            def power_motor(self):
                print("o motor 3 foi ligado!!")

        class Motor4():

            def power_motor(self):
                print("o motor 4 foi ligado!!")

    class Sensors():
        class Sonars():
            class AllSonars():

                def read_sonars(self):
                    print("os sonares estao a ser lidos")

            class Sonar1():

                def read_sonar(self):
                    print("o sonar 1 esta a ser lido")

            class Sonar2():

                def read_sonar(self):
                    print("o sonar 2 esta a ser lido")

            class Sonar3():

                def read_sonar(self):
                    print("o sonar 3 esta a ser lido")

            class Sonar4():

                def read_sonar(self):
                    print("o sonar 4 esta a ser lido")


class Software():
    class Road():
        class Color_Calibrate():
            class Color_Upper():
                print("Aqui sao alterados os valores da cor alta")

            class Color_Lower():
                print("Aqui sao alterados os valores da cor baixa")


def terminal():
    while True:
        print('\033[32m' + '\033[1m' + "TitoLagarto" + '\033[1m' + '@MacBookPro' +
              '\033[0;0m' + ":" + '\033[34m' + "~" + '\033[34m' + '\033[37m' + "$" + '\033[37m')
        user_input = input()

        if user_input == "imagem start":

            main()



        elif user_input == "clear":
            os.system('cls' if os.name == 'nt' else 'clear')

        elif user_input == "exit":
            sys.exit()

        else:
            print(user_input, ":comando não encontrado")


if __name__ == '__main__':
    thread_terminal = Thread_Terminal(1, "THT", 1, 'T')

    thread_terminal.start()
