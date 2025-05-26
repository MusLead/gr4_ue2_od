import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Read data from a text file
def read_data(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
        return [list(map(float, line.strip().split())) for line in lines]

# Load data
data = read_data('odometry.txt')
max_index = len(data)

# Extract all x and y values
all_x, all_y = zip(*data)

# Get min and max with a fixed buffer of 1 unit
min_x, max_x = min(all_x), max(all_x)
min_y, max_y = min(all_y), max(all_y)

# Create plot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)

# Plot dynamic trajectory line
line, = ax.plot([], [], 'bo-', label='Trajectory')

# Static start and end points
start_x, start_y = data[0]
end_x, end_y = data[-1]
start_point, = ax.plot(start_x, start_y, 'go', label='Start')  # green
end_point, = ax.plot(end_x, end_y, 'saddlebrown', marker='o', label='End')  # brown

# Set axis limits with fixed buffer of 1
ax.set_xlim(min_x - 0.5, max_x + 0.5)
ax.set_ylim(min_y - 0.5, max_y + 0.5)

# Labels and title
ax.set_title("Odometry Data Viewer with Slider")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.legend()

# Slider setup
slider_ax = plt.axes([0.2, 0.1, 0.6, 0.03])
frame_slider = Slider(slider_ax, 'Frame', 1, max_index, valinit=1, valstep=1)

# Update function for the slider
def update(val):
    idx = int(frame_slider.val)
    current_data = data[:idx]
    x, y = zip(*current_data)
    line.set_data(x, y)
    fig.canvas.draw_idle()

# Connect the slider to the update function
frame_slider.on_changed(update)

# Initial plot
update(1)
plt.show()
