'''
one line to give the program's name and a brief idea of what it does.>
Copyright (C) <2020>  <Tito Diogo Lagarto da Costa Correia>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''
import cv2
from image_detection import main
import threading
import sys


############################# processamento de imagem ######################################

class Thread_Processamento_De_Imagem(threading.Thread):
    def __init__(self, threadID, name, sleep, cust_type,captura):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sleep = sleep
        self.cust_type = cust_type

        # chama os metodos que precisas...



        self.main = main(captura)


    def returnTheData(self):
        return self.main

def chama():
    # captura = cv2.VideoCapture(0) # Aqui é aberta a coneção com a camera á escolha.
    input = 'Cities Skylines - First Person Rural Mountain Highway Drive.mp4'

    captura = cv2.VideoCapture(0)


    while True:
        thread_processamento_de_imagem = Thread_Processamento_De_Imagem(4, "THPDI", 4, 'PDI', captura)

        thread_processamento_de_imagem.start()

        thread_processamento_de_imagem.join()

        retoma_de_main = thread_processamento_de_imagem.returnTheData()

        if cv2.waitKey(25) & 0xFF == ord('q'):

            break


    captura.release()
    cv2.destroyAllWindows()
