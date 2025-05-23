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

# Create plot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)  # Make space for the slider
line, = ax.plot([], [], 'bo-')
ax.set_xlim(min(d[0] for d in data), max(d[0] for d in data))
ax.set_ylim(min(d[1] for d in data), max(d[1] for d in data))
ax.set_title("Odometry Data Viewer with Slider")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Slider axis and widget
slider_ax = plt.axes([0.2, 0.1, 0.6, 0.03])
frame_slider = Slider(slider_ax, 'Frame', 1, max_index, valinit=1, valstep=1)

# Update function for slider
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
