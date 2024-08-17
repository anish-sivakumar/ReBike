import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import csv

# Load the CSV file
df = pd.read_csv('sampleDataLog29.csv')

# Initialize variables for calculations
regen_requests = 0
total_regen_time = 0
total_positive_energy = 0
total_positive_energy_efficiency = 0
total_regen_energy = 0
total_regen_energy_efficiency = 0
starting_voltage = df['vDC'].iloc[:3].mean()
ending_voltage = df['vDC'].iloc[-3:].mean()

# Loop through the data to perform calculations
for i in range(len(df)):
    # Check for regen request
    if df['iqFdb'].iloc[i] < 0:
        regen_requests += 1

    # Calculate total regen time
    if df['iqFdb'].iloc[i] < 0:
        total_regen_time += 0.1012  # Assuming each row is one second

    # Calculate positive energy and efficiency
    if df['throttle'].iloc[i] > 0:
        total_positive_energy += df['power'].iloc[i]
        total_positive_energy_efficiency += df['iqFdb'].iloc[i] / df['iqRef'].iloc[i]

    # Calculate regen energy and efficiency
    if df['iqFdb'].iloc[i] < 0:
        total_regen_energy += df['power'].iloc[i]
        total_regen_energy_efficiency += abs(df['iqFdb'].iloc[i]) / df['iqRef'].iloc[i]

# Calculate average efficiencies
positive_throttle_count = len(df[df['throttle'] > 0])
if positive_throttle_count > 0:
    total_positive_energy_efficiency /= positive_throttle_count

regen_count = len(df[df['iqFdb'] < 0])
if regen_count > 0:
    total_regen_energy_efficiency /= regen_count

# Calculate battery SoC change
battery_soc_change = starting_voltage - ending_voltage

# Estimate effect on range
estimated_range_effect = (total_positive_energy - total_regen_energy) / 15  # Wh/km

# Results dictionary
results = {
    "Regen Requests": regen_requests,
    "Total Regen Time (s)": total_regen_time,
    "Total Positive Energy (Wh)": total_positive_energy,
    "Total Positive Energy Efficiency (%)": total_positive_energy_efficiency * 100 if positive_throttle_count > 0 else "N/A",
    "Total Regen Energy (Wh)": total_regen_energy,
    "Total Regen Energy Efficiency (%)": total_regen_energy_efficiency * 100 if regen_count > 0 else "N/A",
    "Starting Battery SoC (V)": starting_voltage,
    "Ending Battery SoC (V)": ending_voltage,
    "Battery SoC Change (V)": battery_soc_change,
    "Estimated Effect on Range (km)": estimated_range_effect
}

# Write results to CSV
output_file = 'regen_analysis_results.csv'
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Metric", "Value"])
    for key, value in results.items():
        writer.writerow([key, value])

# Create plots
plt.figure(figsize=(14, 10))

# Plot 1: Power over Time
plt.subplot(2, 2, 1)
plt.plot(df['timestamp'], df['power'], label='Power (W)')
plt.xlabel('Timestamp (s)')
plt.ylabel('Power (W)')
plt.title('Power over Time')
plt.legend()

# Plot 2: Battery Voltage over Time
plt.subplot(2, 2, 2)
plt.plot(df['timestamp'], df['vDC'], label='Battery Voltage (V)', color='orange')
plt.xlabel('Timestamp (s)')
plt.ylabel('Voltage (V)')
plt.title('Battery Voltage over Time')
plt.legend()

# Plot 3: Current over Time
plt.subplot(2, 2, 3)
plt.plot(df['timestamp'], df['iDC'], label='iDC (A)')
plt.plot(df['timestamp'], df['iA'], label='iA (A)')
plt.plot(df['timestamp'], df['iB'], label='iB (A)')
plt.xlabel('Timestamp (s)')
plt.ylabel('Current (A)')
plt.title('Current over Time')
plt.legend()

# Plot 4: Throttle Input over Time
plt.subplot(2, 2, 4)
plt.plot(df['timestamp'], df['throttle'], label='Throttle Input (%)', color='green')
plt.xlabel('Timestamp (s)')
plt.ylabel('Throttle Input (%)')
plt.title('Throttle Input over Time')
plt.legend()

plt.tight_layout()
plt.show()

# Create a heatmap to show the correlation between variables
# plt.figure(figsize=(12, 8))
# correlation_matrix = df.corr()
# sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', linewidths=0.5)
# plt.title('Correlation Matrix')
# plt.show()
