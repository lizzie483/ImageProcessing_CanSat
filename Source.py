import cv2  # Importamos libreria OpenCV 3.4
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import time


# Definimos limite inferior y superior de color en HSV
cotaInf = np.array([33, 80, 40])
cotaSup = np.array([102, 255, 255])

# Kernel de apertura y clausura
kernelAp = np.ones((5, 5))
kernelCl = np.ones((20, 20))

# Parte de la dirección 0, no hay giro
direc = 0
cond = 0  # Condicion inicial de BUSQUEDA
u = 0  # Inicializamos condicion de movimiento del rover

# Inicializa la camara
camera = PiCamera()
camera.resolution = (640, 480)
camera.rotation = 90
camera.framerate = 30
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

ancho = 640
largo = 480

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv2.imshow("Frame", image)

    key = cv2.waitKey(1) & 0xFF

    # Convierte de RGB a HSV
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Mascara que contiene el rango de colores en sus pixeles
    mask = cv2.inRange(imgHSV, cotaInf, cotaSup)
    # Operaciones de morfologia
    maskAp = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelAp)
    maskCl = cv2.morphologyEx(maskAp, cv2.MORPH_CLOSE, kernelCl)
    # Seleccion de mascara final
    maskFinal = maskCl

    _, conts, h = cv2.findContours(maskFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cv2.line(image, (int(7 * ancho / 16), 0), (int(7 * ancho / 16), largo), (80, 100, 80), 3)
    cv2.line(image, (int(9 * ancho / 16), 0), (int(9 * ancho / 16), largo), (80, 100, 80), 3)

    if bool(conts):

        # Condicion para iniciar el movimiento
        # Una vez detectada por primera vez se cancela la funcion de busqueda
        cond = 1

        # Parametros del rectangulo mas grande
        # x,y,w,h = cv2.boundingRect(conts[lista.index(max(lista))])
        mcont = max(conts, key=cv2.contourArea)
        cv2.drawContours(image, mcont, -1, (255, 0, 0), 3)
        x, y, w, h = cv2.boundingRect(mcont)

        # Dibujo del rectangulo
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        lineThickness = 2
        xm = int(x + w / 2)

        # Trazado de linea central
        cv2.line(image, (xm, y), (xm, y + h), (0, 255, 0), 2)

        sp = ancho / 2
        ep = xm - sp
        d1 = abs(ep)

        # Navegacion
        if (xm > 7 * ancho / 16 and xm < 9 * ancho / 16):
            u = 1  # Directo

        elif (xm > 9 * ancho / 16):
            u = 2  # Derecha

        else:
            u = 3  # Izquierda
        # Detener
        if (h >= 0.6 * largo):
            u = 4  # Parar

    # Limpia el stream para poder trabajar con el siguiente frame
    rawCapture.truncate(0)

    if bool(cond):
        if (u == 1):
           
            print("Directo")

        if (u == 3):
           
            print("Derecha")

        if (u == 2):
           
            print("Izquierda")
        if (u == 4):
            print("Parar")

            
            break
    else:
       
        print("Buscando")
