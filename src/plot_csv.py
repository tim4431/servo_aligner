import pandas as pd, numpy as np
import matplotlib.pyplot as plt

# File path
csv_file = 'mcp3424_readings_2.csv'

# Read the CSV file
data = pd.read_csv(csv_file)

# Ensure the CSV has 'Timestamp' and 'Reading' columns
if 'Timestamp' not in data.columns or 'Reading' not in data.columns:
    raise ValueError("CSV file must contain 'Timestamp' and 'Reading' columns.")
data['Timestamp'] = pd.to_datetime(data['Timestamp'], errors='raise')
# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(data['Timestamp'], data['Reading'], label='Reading vs Timestamp', color='blue')
plt.xlabel('Timestamp')
plt.ylabel('Reading')
plt.title('MCP3424 Readings vs Timestamp')
plt.legend()
plt.grid(True)
plt.show()
plt.savefig('mcp3424_readings_plot_2.png', dpi=300, bbox_inches='tight')

# # Uniform resampling
# # Define a uniform time grid
# time_sec = (data['Timestamp'] - data['Timestamp'].iloc[0]).dt.total_seconds().values
# readings = data['Reading'].values
# # get rid of DC
# readings -= np.mean(readings)
# uniform_time = np.linspace(time_sec.min(), time_sec.max(), len(time_sec))

# # Interpolate readings onto the uniform grid
# uniform_readings = np.interp(uniform_time, time_sec, readings)

# # Do FFT
# fft_vals = np.fft.fft(uniform_readings)
# fft_freqs = np.fft.fftfreq(len(uniform_time), d=(uniform_time[1] - uniform_time[0]))  # d is time step

# # Only keep positive frequencies
# positive_freqs = fft_freqs[fft_freqs >= 0]
# positive_fft = np.abs(fft_vals[fft_freqs >= 0])

# # Plot the spectrum
# plt.figure(figsize=(10, 6))
# plt.plot(positive_freqs, positive_fft, color='red')
# plt.xlabel('Frequency (Hz)')
# plt.ylabel('Amplitude')
# plt.title('FFT of MCP3424 Readings')
# plt.xlim(0,1/60)
# plt.grid(True)
# # plt.show()
# plt.savefig('mcp3424_readings_fft_2.png', dpi=300, bbox_inches='tight')