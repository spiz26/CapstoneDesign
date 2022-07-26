# CapstoneDesign
CapstoneDesign
  
Data are in cap_data directory.  
cap_data 폴더 안에 normal, abnormal data가 있습니다.  
  
CapstoneDesign/  
  cap_data/  
    accident  
    ordinary  
  
Autoencoder_v5.ipynb는 cap_data안의 데이터를 가지고 LSTM(or GRU) autodencoder를 학습시킵니다.  
normal data로만 학습시키고, 학습이 잘 되었는지 확인하기 위해 accident data를 abnormal data로 사용합니다.  
라즈베리파이에서 사용해야하기에 용량도 크고, 판단 시간도 긴 h5파일을 tflite로 변환시켜주어야 합니다. 맨 마지막 cell에 h5 to tflite 코드가 적혀있습니다.  
  
1. Data gathering
이 시스템은 라즈베리파이가 아두이노 IMU센서의 데이터를 실시간으로 받아 실행됩니다.  
data_gathering.ino파일을 실행시키고, 시리얼 모니터를 끈 채로, data_gathering_rasp.py파일을 실행시키면 됩니다.   
data_gathering_rasp.py파일에서 file_name을 마음대로 바꿔서 사용하시면 됩니다.  
생성된 csv파일을 cap_data에 카테고리에 맞게 넣습니다.  
  
  
2. Training model
GPU가 있는 컴퓨터에 Autoencoder_v5.ipynb파일과 cap_data 폴더를 다운받아 실행시킵니다.  
Autoencoder_v5.ipynb은 auto.h5, auto.tflite, scaler.pkl, history.pkl 총 4개의 파일을 생성합니다.  
라즈베리파이에서 실행할 때는 auto.tflite, scaler.pkl 2개의 파일만 있으면 됩니다.  

3. Use system
먼저 라즈베리파이에 auto.tflite, auto_running.py, scaler.pkl 총 3개의 파일을 다운받습니다.  
data_gathering.ino파일을 실행시켜 아두이노가 데이터를 실시간으로 측정하게 하고, auto_running.py파일을 실행시킵니다.


