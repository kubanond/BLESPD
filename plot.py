import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the CSV file
csv_filename = 'sensor_data2.csv'
data = pd.read_csv(csv_filename)

# Convert the timestamp column to datetime
data['timestamp'] = pd.to_datetime(data['timestamp'])

# Define the specific timestamp or range
# For a single timestamp, you could find the closest row, but typically you'll want a range.
# Here we filter by a date range. Adjust the start and end values as needed.
start_time = pd.Timestamp('2025-04-13 15:40:50')

# Filter the DataFrame for the selected time range
filtered_data = data[(data['timestamp'] >= start_time)]

# Calculate the magnitude of acceleration for the filtered data: sqrt(X^2 + Y^2 + Z^2)
filtered_data['acc_magnitude'] = np.sqrt(filtered_data['X']**2 +
                                         filtered_data['Y']**2 +
                                         filtered_data['Z']**2)

# Plot the acceleration magnitude over time for the filtered period
plt.figure(figsize=(10, 6))
plt.plot(filtered_data['timestamp'], filtered_data['acc_magnitude'], marker='o', linestyle='-')
plt.xlabel('Time')
plt.ylabel('Acceleration Magnitude')

plt.xticks(rotation=45)
plt.tight_layout()
plt.show()