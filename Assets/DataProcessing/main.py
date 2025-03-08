import csv
import matplotlib.pyplot as plt

# File path to your CSV file
csv_file_path = r"C:\Users\UserAdmin\Terrain Generation\Assets\DataProcessing\PID Drone.csv"

# Lists to store data
timesteps = []
scores = []

# Read the CSV file line by line
with open(csv_file_path, mode='r') as file:
    reader = csv.reader(file)
    # Skip the header
    next(reader)
    
    # Read each row
    for row in reader:
        timesteps.append(float(row[0]))  # Convert timestep to float
        scores.append(float(row[1]))    # Convert score to float

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(timesteps, scores, label="Score over Time", color="blue")

# Add labels and title
plt.xlabel("Time Step")
plt.ylabel("Score")
plt.title("Score vs. Time Step")
plt.legend()
plt.grid()

# Show the plot
plt.show()