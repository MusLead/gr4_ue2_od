import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# Read data from a text file
def read_data(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
        return [list(map(float, line.strip().split())) for line in lines]

# Load your data
data = read_data('odometry.txt')  # Make sure this file exists
max_frames = len(data)
frame = [1]  # Use list to make it mutable in button callbacks

# Prepare figure and plot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)  # Leave space for buttons
line, = ax.plot([], [], 'bo-')
ax.set_xlim(min(d[0] for d in data), max(d[0] for d in data))
ax.set_ylim(min(d[1] for d in data), max(d[1] for d in data))

# Update plot based on current frame
def update_plot():
    current_data = data[:frame[0]]
    if current_data:
        x, y = zip(*current_data)
        line.set_data(x, y)
    else:
        line.set_data([], [])
    fig.canvas.draw_idle()

# Button callback: Increase frame
def next_frame(event):
    if frame[0] < max_frames:
        frame[0] += 1
        update_plot()

# Button callback: Decrease frame
def prev_frame(event):
    if frame[0] > 1:
        frame[0] -= 1
        update_plot()

# Add buttons
axprev = plt.axes([0.3, 0.05, 0.1, 0.075])
axnext = plt.axes([0.6, 0.05, 0.1, 0.075])
bnext = Button(axnext, 'Next')
bprev = Button(axprev, 'Previous')
bnext.on_clicked(next_frame)
bprev.on_clicked(prev_frame)

# Initial plot
update_plot()
plt.show()
