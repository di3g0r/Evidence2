import matplotlib.pyplot as plt
import numpy as np

# Data
data = {
    'run': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
    'num_robbers': [2, 2, 3, 3, 4, 4, 4, 5, 5, 5],
    'spotted': [2, 1, 2, 1, 4, 2, 3, 3, 2, 3],
    'detection_rate': [100, 50, 66.67, 33.33, 100, 50, 75, 60, 40, 60]
}

# Create figure and axis objects with a single subplot
fig, ax1 = plt.subplots(figsize=(12, 6))

# Create second y-axis that shares same x-axis
ax2 = ax1.twinx()

# Plot number of robbers and spotted robbers on left axis
line1 = ax1.plot(data['run'], data['num_robbers'], color='blue', marker='o', label='Number of Robbers')
line2 = ax1.plot(data['run'], data['spotted'], color='green', marker='s', label='Robbers Spotted')

# Plot detection rate on right axis
line3 = ax2.plot(data['run'], data['detection_rate'], color='red', marker='^', label='Detection Rate')

# Set labels and title
ax1.set_xlabel('Run Number')
ax1.set_ylabel('Number of Robbers')
ax2.set_ylabel('Detection Rate (%)')
plt.title('Threat Detection Analysis')

# Add grid
ax1.grid(True, linestyle='--', alpha=0.7)

# Combine legends
lines = line1 + line2 + line3
labels = [line.get_label() for line in lines]
ax1.legend(lines, labels, loc='upper left')

# Set y-axis ranges
ax1.set_ylim(0, max(data['num_robbers']) + 1)
ax2.set_ylim(0, 110)  # Setting to 110 to give some space above 100%

# Set x-axis to show all run numbers
ax1.set_xticks(data['run'])

# Adjust layout to prevent label cutoff
plt.tight_layout()

plt.show()