"""
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
"""

# -*- coding: utf-8 -*-
import cv2
import math
import numpy as np
import threading
import time
from simple_pid import PID


def filtro_de_imagem_branca(principal_frame):
    rgb_image_converter = cv2.cvtColor(principal_frame,
                                       cv2.COLOR_BGR2RGB)  # filtra a imagem vinda da variavel "frame" parara a
    # variavel "rgb"

    # filtra a imagem branca com RGB

    lowerRGB = np.uint8([[[200, 200, 200]]])  # valor minimo de RGB para a cor branca
    upperRGB = np.uint8([[[255, 255, 255]]])  # valor maximo de RGB para a cor branca

    lower_yellow = np.array(cv2.cvtColor(lowerRGB, cv2.COLOR_BGR2RGB))
    upper_yellow = np.array(cv2.cvtColor(upperRGB, cv2.COLOR_BGR2RGB))

    mascara = cv2.inRange(rgb_image_converter, lower_yellow,
                          upper_yellow)  # calcula o alcance entre os dois valores RGB inceridos
    # nas variaveis "upperRGB" e "upperRGB" em relaçao á imagem "rgb_image_converter"

    return mascara


def filtro_de_imagem_amarela(principal_frame):
    hsv = cv2.cvtColor(principal_frame, cv2.COLOR_BGR2HSV)

    # defining the range of Yellow color
    yellow_lower = np.array([13, 115, 130], np.uint8)
    yellow_upper = np.array([60, 255, 255], np.uint8)

    # finding the range yellow colour in the image
    yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
    # nas variaveis "upperRGB" e "upperRGB" em relaçao á imagem "rgb_image_converter"

    return yellow


def roi(img, vertices):
    # blank mask:
    mask = np.zeros_like(img)
    # fill the mask
    cv2.fillPoly(mask, np.int32([vertices]), 255)
    # now only show the area that is the mask
    masked = cv2.bitwise_and(img, mask)
    return masked


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, line_segments):
    """
    Esta função combina segmentos de linha em uma ou duas linhas de pista
    Se todas as inclinações da linha forem < 0: então detectamos apenas a faixa esquerda
    Se todas as inclinações de linha forem > 0: então detectamos apenas a faixa direita
    """
    lane_lines = []
    if line_segments is None:
        # print('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary  # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                # print('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # print('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def calculo_de_angulo_de_viragem(lane_lines, img, width, height):
    pid = PID(1, 0.5, 0.05, setpoint=1)

    switcher = {
        0: 'go left',
        1: 'go right'

    }

    # Variavel para o "switch case"

    try:
        _, _, left_x2, _ = lane_lines[0][0]
        l = left_x2
    except:
        switcher = 0

    '''
            Este "try" esta a verificar se a posicao [0][0] da variavel lane_lines esta vazia pois se estiver ele 
            ira parar a exepcao. Senao o programa continuara a sua normal execucao.
            '''

    try:
        _, _, right_x2, _ = lane_lines[1][0]
        r = right_x2
    except:
        switcher = 1

    '''
            Este "try" esta a verificar se a posicao [1][0] da variavel lane_lines esta vazia pois se estiver ele 
            ira parar a exepcao. Senao o programa continuara a sua normal execucao.
            '''

    if switcher == 0:
        print('go left!!!')
    elif switcher == 1:
        print('go right!!')
    else:
        '''
                Este if statement faz com que so se as variaveis "l" e "r" contiverem valores o programa continuara 
                a sua execucao caso contrario o carro ira ser desviado para o lado onde a linha foi perdida podendo 
                assim encontra-la
                '''
        mid = int(width / 2)
        x_offset = (l + r) / 2 - mid
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

        # print(steering_angle)

        steering_angle = pid(steering_angle)

        hi = display_heading_line(img, steering_angle)
        cv2.imshow('Imagem Com Ponto de Referencia', hi)


########################################################################################################################
# CLASSES

class Thread_Filtro_de_Imagem_Branca(threading.Thread):
    def __init__(self, threadID, name, sleep, cust_type, img):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sleep = sleep
        self.cust_type = cust_type

        # call the methods you need on your data...

        self.mascara_branca = filtro_de_imagem_branca(img)

    def returnTheData(self):
        return self.mascara_branca


class Thread_Filtro_de_Imagem_Amarela(threading.Thread):
    def __init__(self, threadID, name, sleep, cust_type, img):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sleep = sleep
        self.cust_type = cust_type

        # call the methods you need on your data...

        self.mascara_amarela = filtro_de_imagem_amarela(img)

    def returnTheData(self):
        return self.mascara_amarela


class Thread_Average_Slope_Intercept(threading.Thread):
    def __init__(self, threadID, name, sleep, cust_type, img, lines):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.sleep = sleep
        self.cust_type = cust_type

        # call the methods you need on your data...

        self.mascara_amarela = average_slope_intercept(img, lines)

    def returnTheData(self):
        return self.mascara_amarela


########################################################################################################################

def main():
    # captura = cv2.VideoCapture(0) # Aqui é aberta a coneção com a camera á escolha.
    input = 'Cities Skylines - First Person Rural Mountain Highway Drive.mp4'

    captura = cv2.VideoCapture(0)

    global r
    global l
    while True:
        inicio = time.time()

        # Aqui começa a contagem do tempo para no fim de cada loop ser calculada a duraçao do mesmo.

        ret, img = captura.read()
        # Aqui a captura é transformada numa array para poder ser processada.

        img = cv2.resize(img, (480, 360))
        # Aqui a imagem é redimencionada.

        height, width, _ = img.shape
        # Aqui são recolhidas a altura e a largura da imagem redimencionada.

        cv2.imshow('Imagem Original', img)
        # Aqui é mostrada a imagem original já redimencionada.

        ############# AQUI É COMEÇADA A THREAD DA FUNÇÃO "filtro_de_imagem_branca" #############

        thread_filtro_de_imagem_branca = Thread_Filtro_de_Imagem_Branca(2, "THIMGB", 2, 'IMGB', img)

        thread_filtro_de_imagem_branca.start()

        thread_filtro_de_imagem_branca.join()

        mascara_branca = thread_filtro_de_imagem_branca.returnTheData()

        ########################################################################################

        cv2.imshow("imagem branca", mascara_branca)

        ############# AQUI É COMEÇADA A THREAD DA FUNÇÃO "filtro_de_imagem_amarela" #############

        thread_filtro_de_imagem_amarela = Thread_Filtro_de_Imagem_Amarela(3, "THIMGA", 3, 'IMGA', img)

        thread_filtro_de_imagem_amarela.start()

        thread_filtro_de_imagem_amarela.join()

        mascara_amarela = thread_filtro_de_imagem_amarela.returnTheData()

        #########################################################################################

        '''
                Nestas duas threads anteriores foram aplicados os filtros para serem apenas detetadas as cores branca e 
                amarela para assim so detetar linhas dessas cores.
                
                Tudo o que esta a branco é o que é capturado e o que esta a preto é ignorado.
                
                NOTA!!!:  ESSAS THREADS PODEM SER COSTOMIZADAS COMO SE PRETENDER. PODEM SER....
                
                    - ADICIONADAS NOVAS PARA FILTRAGEM DE MAIS CORES
                    - REMOVIDAS PARA FILTRAGEM DE MENOS CORES
                    - ALTERADAS PARA DETETAR OUTRAS CORES (ao alterar as cores que elas processam por favor alterar os
                    - respetivos nomes para nao haver futuros enganos).
                '''

        cv2.imshow("imagem amarela", mascara_amarela)

        imagens_misturadas = cv2.addWeighted(mascara_branca, 1, mascara_amarela, 1, 0)
        '''
                aqui é feita a junçao das imagens os parametros que estao em "1" é o valor da opacidade, e o parametro 
                a 0 é a gama da imagem.
                Tudo o que esta a branco é o que é capturado e o que esta a preto é ignorado.
                '''

        vertices = np.array([[0, 478], [70, 280], [520, 270], [639, 470]],
                            np.int32)  # aqui são defenidos o s vertices para a regiaop de interece "roi - Region of Interest"

        mascara_final = roi(imagens_misturadas, vertices)
        cv2.imshow("blended image", mascara_final)
        #########################################################

        edges = cv2.Canny(mascara_final, 220, 400)
        cv2.imshow('edges', edges)

        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 15  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 50  # minimum number of pixels making up a line
        max_line_gap = 200  # maximum gap in pixels between connectable line segments
        line_image = np.copy(mascara_final) * 0  # creating a blank to draw lines on

        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)

        ############# AQUI É COMEÇADA A THREAD DA FUNÇÃO "average_slope_intercept" #############

        thread_average_slope_intercept = Thread_Average_Slope_Intercept(10, "THASI", 10, 'ASI', img, lines)

        thread_average_slope_intercept.start()

        thread_average_slope_intercept.join()

        lane_lines = thread_average_slope_intercept.returnTheData()

        ########################################################################################

        # print(lane_lines)
        lane_lines_image = display_lines(img, lane_lines)
        cv2.imshow('Imagem Com Linhas', lane_lines_image)

        calculo_de_angulo_de_viragem(lane_lines, img, width, height)

        print("FPS: ", 1.0 / (time.time() - inicio))  # FPS = 1 / time to process loop
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    captura.release()
    cv2.destroyAllWindows()