import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# Read data from a text file
def read_data(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
        return [list(map(float, line.strip().split())) for line in lines]

# Load data
data = read_data('odometry.txt')
max_index = len(data)

# Frame index (how many points to show)
index = [1]  # Using a list to keep it mutable in callbacks

# Create plot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
line, = ax.plot([], [], 'bo-')
ax.set_xlim(min(d[0] for d in data), max(d[0] for d in data))
ax.set_ylim(min(d[1] for d in data), max(d[1] for d in data))
ax.set_title("Odometry Data Viewer")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Update function
def update_plot():
    current_data = data[:index[0]]
    x, y = zip(*current_data)
    line.set_data(x, y)
    fig.canvas.draw_idle()

# Button callbacks
def next_point(event):
    if index[0] < max_index:
        index[0] += 1
        update_plot()

def prev_point(event):
    if index[0] > 1:
        index[0] -= 1
        update_plot()

# Create buttons
axprev = plt.axes([0.3, 0.05, 0.1, 0.075])
axnext = plt.axes([0.6, 0.05, 0.1, 0.075])
bnext = Button(axnext, 'Next')
bprev = Button(axprev, 'Previous')
bnext.on_clicked(next_point)
bprev.on_clicked(prev_point)

# Initial plot
update_plot()
plt.show()
