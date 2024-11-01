import cv2
import RPi.GPIO as GPIO
import numpy as np
from simple_pid import PID
from filter import KalmanFilter
from i2c_itg3205 import *
from i2c_adxl345 import *
from time import sleep
import atexit

#  ------------------------------------------Close function----------------------------------
def on_esc():
    print("end")
    MotorStop()


def ClearPins():
    GPIO.cleanup()
    exit()
    return

atexit.register(on_esc)

#  ------------------------------------------Constants----------------------------------


#  Camera init
car_cascade = cv2.CascadeClassifier('./cascade.xml')
cap = cv2.VideoCapture(0, cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
frame_width = 640
frame_height = 480
image_size = (frame_width, frame_height)

ENA = 13
ENB = 12
IN1 = 26
IN2 = 21
IN3 = 16
IN4 = 19

# Set the type of GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# Motor initialized to LOW
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)
pwmA.start(0)
pwmB.start(0)

pid = PID(1, 0.1, 0.05, setpoint=0)
pid.output_limits = (-40, 40)

T = 1/15

EPS = 0.25

VECTOR_SIZE = 8
MEASUREMENT_SIZE = 4
x_prev = np.zeros(VECTOR_SIZE)
D_n = np.diag([4.283398238658887, 2.227185341333505, 4.896227345745329, 5.3289875509286775])
D_ksi = np.diag([463.01033005794915, 398.73519778281695, 236.91962711010333, 292.40866717057196])

F = np.array([[1, T, 0, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 1, T, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, T, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 1, T],
              [0, 0, 0, 0, 0, 0, 0, 1]])

G = np.array([[0, 0, 0, 0],
              [T, 0, 0, 0],
              [0, 0, 0, 0],
              [0, T, 0, 0],
              [0, 0, 0, 0],
              [0, 0, T, 0],
              [0, 0, 0, 0],
              [0, 0, 0, T]])

H = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 1, 0]])

x1 = x2 = frame_width / 2
y1 = y2 = frame_height / 2
x_vector = np.array((x1, 0, y1, 0, x2, 0, y2, 0))
kfilter = KalmanFilter(VECTOR_SIZE, T, D_n, D_ksi, F, G, H, x_vector, 20)


#  ------------------------------------------Base function----------------------------------
def haar_pipeline(ret,frames):
    stop, x, w, n_cars = haar(ret, frames)
    if n_cars >= 1:
        angle = ((x + w / 2) - frame_width / 2)
        MotorTurnOnRidePID(angle, 35)
    else:
        MotorStop()


def haar(ret, frames):
    if ret:
        gray = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)
        cars = car_cascade.detectMultiScale(gray, minSize=(90, 90))
        n_cars = 0
        x_out = 0
        w_out = 0
        y_min = 1000
        for (x, y, w, h) in cars:
            n_cars = n_cars + 1
            x_vector = np.array((x, y, x + w, y + h))
            x1f, _, y1f, _, x2f, _, y2f, _ = map(int, kfilter(x_vector))
            cv2.rectangle(frames, (x1f, y1f), (x2f, y2f), (0, 255, 0), 2)
            cv2.rectangle(frames, (x, y), (x + w, y + h), (255, 0, 0), 2)
            x_out = x
            w_out = w
            if y_min < y:
                x_out = x
                w_out = w
                y_min = y

        if n_cars == 0:
            x1f, _, y1f, _, x2f, _, y2f, _ = map(int, kfilter())
            cv2.rectangle(frames, (x1f, y1f), (x2f, y2f), (0, 255, 0), 2)
            x_out = x1f
            w_out = x2f - x1f
            n_cars = 1

        return stop, x_out, w_out, n_cars
    else:
        return None, None, None, None


def MotorTurnOnRidePID(param, speed):
    pid.setpoint = 0
    pid_output = pid(param)

    speed_left = speed - pid_output
    speed_right = speed + pid_output
    speed_left = max(0, min(60, speed_left))
    speed_right = max(0, min(60, speed_right))

    pwmB.ChangeDutyCycle(abs(speed_right))
    pwmA.ChangeDutyCycle(abs(speed_left))

    if speed_left >= 0:
        GPIO.output(IN1, False)
        GPIO.output(IN2, True)
    else:
        GPIO.output(IN1, True)
        GPIO.output(IN2, False)

    if speed_right >= 0:
        GPIO.output(IN3, False)
        GPIO.output(IN4, True)
    else:
        GPIO.output(IN3, True)
        GPIO.output(IN4, False)


def GetPosition():
    with open("state.txt", "r") as f:
        string = str(f.read())
        if not (string == ''):
            temp_str = string.split(' ')
            if not (temp_str[0] == ''):
                x = float(temp_str[0])
                y = float(temp_str[1])
                z = 0
                location = [x, y, z]
                f.close()
                return location
            else:
                return None
        else:
            return None


def turn_by_angle(angle_g):
    itg3205 = i2c_itg3205(1)
    angle = angle_g*math.pi/180
    dt = 0.2
    spe = 60
    pwmA.ChangeDutyCycle(spe)
    pwmB.ChangeDutyCycle(spe)
    x, y, z = itg3205.getDegPerSecAxes()
    start_error = z
    this_angle = 0
    while abs(angle - this_angle) > 0.3:
        if angle - this_angle > 0:
            GPIO.output(IN4, True)
            GPIO.output(IN3, False)
            GPIO.output(IN1, True)
            GPIO.output(IN2, False)
        else:
            GPIO.output(IN3, True)
            GPIO.output(IN4, False)
            GPIO.output(IN2, True)
            GPIO.output(IN1, False)
        sleep(dt)
        try:
            itgready, dataready = itg3205.getInterruptStatus()
            if dataready:
                x, y, z = itg3205.getDegPerSecAxes()
                this_angle += -(z - start_error) / 180 * math.pi * dt
        except OSError:
            pass
    MotorStop()


def MotorStop():
    print('motor stop')
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, False)


def MotorForward():
    print('motor forward')
    pwmA.ChangeDutyCycle(50)
    pwmB.ChangeDutyCycle(50)
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)


#  ------------------------------------------Start programm----------------------------------
MotorStop()
sleep(2)

#  ------------------------------------------Path----------------------------------
path = [
  [4.2, 2],
  [4.41, 0.55],
  [0.45, 0.6],
  [0.45, 4.5],
  [4.5, 4.5],
  [4.41, 0.4],
]

#  ------------------------------------------Exercise----------------------------------
stops_ = True
while stops_:
    MotorForward()
    curr_location = GetPosition()
    if curr_location is not None:
        if abs(curr_location[0] - path[0][0]) < EPS:
            stops_ = False
MotorStop()

turn_by_angle(-95)

#  ------------------------------------------Exercise----------------------------------
stops_ = True
while stops_:
    ret, frame = cap.read()
    haar_pipeline(ret, frame)
    curr_location = GetPosition()
    if curr_location is not None:
        if abs(curr_location[1] - path[1][1]) < EPS:
            stops_ = False
MotorStop()

turn_by_angle(-85)

stops_ = True
while stops_:
    ret, frame = cap.read()
    haar_pipeline(ret, frame)
    curr_location = GetPosition()
    if curr_location is not None:
        if abs(curr_location[0] - path[2][0]) < EPS:
            stops_ = False
MotorStop()

turn_by_angle(-95)

stops_ = True
while stops_:
    ret, frame = cap.read()
    haar_pipeline(ret, frame)
    curr_location = GetPosition()
    if curr_location is not None:
        if abs(curr_location[1] - path[3][1]) < EPS:
            stops_ = False
MotorStop()

turn_by_angle(-95)

stops_ = True
while stops_:
    ret, frame = cap.read()
    haar_pipeline(ret, frame)
    curr_location = GetPosition()
    if curr_location is not None:
        if abs(curr_location[0] - path[4][0]) < EPS:
            stops_ = False
MotorStop()

turn_by_angle(-95)

stops_ = True
while stops_:
    ret, frame = cap.read()
    haar_pipeline(ret, frame)
    curr_location = GetPosition()
    if curr_location is not None:
        if abs(curr_location[1] - path[5][1]) < EPS:
            stops_ = False

#  ------------------------------------------End programm----------------------------------
MotorStop()
cap.release()
cv2.destroyAllWindows()
