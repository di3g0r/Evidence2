import matplotlib.pyplot as plt
import numpy as np

# Data
runs = np.arange(1, 11)
cells_covered = [100, 120, 109, 123, 103, 135, 141, 117, 131, 114]
coverage_percentage = [25, 30, 27.25, 30.75, 25.75, 33.75, 35.25, 29.25, 32.75, 28.5]
average_coverage = 29.825

# Create figure
plt.figure(figsize=(12, 8))

# Create bar plot for coverage percentage
bars = plt.bar(runs, coverage_percentage, color='skyblue', alpha=0.7)

# Add horizontal line for average
plt.axhline(y=average_coverage, color='red', linestyle='--', label=f'Average Coverage ({average_coverage}%)')

# Customize the plot
plt.title('Area Coverage Analysis (200 Steps)', fontsize=14, pad=20)
plt.xlabel('Run Number', fontsize=12)
plt.ylabel('Area Coverage (%)', fontsize=12)

# Add value labels on top of each bar
for i, bar in enumerate(bars):
    height = bar.get_height()
    plt.text(bar.get_x() + bar.get_width()/2., height,
             f'{height}%',
             ha='center', va='bottom')

# Add value labels for cells covered inside each bar
for i, cells in enumerate(cells_covered):
    plt.text(i+1, coverage_percentage[i]/2,
             f'{cells} cells',
             ha='center', va='center',
             color='white', fontweight='bold')

# Customize grid
plt.grid(True, axis='y', linestyle='--', alpha=0.7)

# Customize ticks
plt.xticks(runs)
plt.yticks(np.arange(0, max(coverage_percentage) + 5, 5))

# Add legend
plt.legend()

# Adjust layout
plt.tight_layout()

# Show plot
plt.show()