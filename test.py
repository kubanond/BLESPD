import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

csv_filename = 'sensor_data2.csv'
data = pd.read_csv(csv_filename)

data['timestamp'] = pd.to_datetime(data['timestamp'])
data['acc_magnitude'] = np.sqrt(data['X']**2 + data['Y']**2 + data['Z']**2)/128.0
data['acc_detrended'] = data['acc_magnitude'] - data['acc_magnitude'].rolling(window=100, min_periods=1).mean()

fs = 50
cutoff = 3

b, a = signal.butter(N=3, Wn=cutoff/(0.5*fs), btype='low', analog=False)
data['filtered'] = signal.filtfilt(b, a, data['acc_detrended'])

peak_height = 0.15
min_distance = int(0.5 * fs)

peaks, properties = signal.find_peaks(data['filtered'], height=peak_height, distance=min_distance)

step_count = len(peaks)
print("Number of detected steps:", step_count)

plt.figure(figsize=(12, 6))
plt.plot(data['timestamp'], data['filtered'])
plt.plot(data['timestamp'][peaks], data['filtered'][peaks], "ro")
plt.xlabel('Time')
plt.ylabel('Acceleration (m/s^2)')
plt.title('Step Detection from Accelerometer Data')
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()