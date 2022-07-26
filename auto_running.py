#library import
import numpy as np
import serial
import time
import sklearn

import joblib
import RPi.GPIO as rp
import tensorflow as tf

from collections import deque
from tensorflow.keras.models import load_model
from tensorflow.keras.metrics import MeanSquaredError
from time import time as t
from threading import Thread

print("start")

#communicate arduino
ser = serial.Serial('/dev/ttyACM0', 9600)
ser.flush()

#Define constant
BUFFER_SIZE = 20
ACCIDENT_THRESHOLD = 35 
_hit = 8
_return = 23

#pin setting
buzzer_pin = 13
servo_pin = 19
red_pin = 21
blue_pin = 20
green_pin = 16

#input data tensor deque
buffer_deque = deque(maxlen=BUFFER_SIZE)

#tensor model loading
tflite_model = tf.lite.Interpreter("./auto.tflite")
tflite_model.allocate_tensors()
input_index = tflite_model.get_input_details()[0]["index"]
output_index = tflite_model.get_output_details()[0]["index"]

#sklearn scaler loading
scaler = joblib.load('scaler.pkl')

#GPIO setting
rp.setwarnings(False)
rp.setmode(rp.BCM)
rp.setup(buzzer_pin, rp.OUT)
rp.setup(servo_pin, rp.OUT)
rp.setup(red_pin, rp.OUT)
rp.setup(blue_pin, rp.OUT)
rp.setup(green_pin, rp.OUT)

pwm1=rp.PWM(buzzer_pin, 100)
pwm2=rp.PWM(servo_pin, 100)
pwm1.start(0)
pwm2.start(0)

#motor reset
pwm2.ChangeDutyCycle(_return)
time.sleep(0.5)
pwm2.stop()
print("motor start")
time.sleep(0.1)
pwm2.start(0)

#green pin HIGH
rp.output(green_pin, rp.HIGH)

def get_data():
    while True:
        #Get data
        line = ser.readline().decode('utf-8').rstrip().split(",")
        line = list(map(float, line))
        line = np.array(line, dtype=np.float32)
            
        buffer_deque.append(line)

def pymain():
    while True:
        #tflite inference
        t0 = t()
        dq_tensor = np.array(buffer_deque)
        dq_tensor = scaler.transform(dq_tensor)
        
        in_tensor = tf.convert_to_tensor([dq_tensor], dtype=np.float32)
        tflite_model.set_tensor(input_index, in_tensor)
        
        tflite_model.invoke()
        recon = tflite_model.get_tensor(output_index)[0]
        
        MSE = MeanSquaredError()
        MSE.update_state(dq_tensor, recon)
        mse = MSE.result().numpy()
        t1 = t()
        
        print(f"mse : {round(float(mse), 4)}, time : {round(t1-t0, 4)}")
        time.sleep(0.001)
        
        if mse > ACCIDENT_THRESHOLD:
            print("accident")
            rp.output(green_pin, rp.LOW)
            rp.output(red_pin, rp.HIGH)
            pwm1.ChangeDutyCycle(100)
            pwm2.ChangeDutyCycle(_hit) #8:hit, 23:return 
            time.sleep(2)
            
            pwm2.ChangeDutyCycle(_return)
            time.sleep(0.5)
            pwm1.stop()
            pwm2.stop()
            rp.cleanup()
            break
        
#main      
data_thread = Thread(target=get_data)
main_thread = Thread(target=pymain)

data_thread.start()
time.sleep(1)
main_thread.start()
