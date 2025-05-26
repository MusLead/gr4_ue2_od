import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Read data from a text file
def read_data(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
        return [list(map(float, line.strip().split())) for line in lines]

# Load data
odometry_data = read_data('odometry.txt')
imu_data = read_data('imu.txt')

# Make sure both datasets have the same number of entries
max_index = min(len(odometry_data), len(imu_data))

# Extract all x and y from both datasets for setting plot limits
all_x = [x for x, _ in odometry_data] + [x for x, _ in imu_data]
all_y = [y for _, y in odometry_data] + [y for _, y in imu_data]

# Get min and max with a buffer of 1 unit
min_x, max_x = min(all_x), max(all_x)
min_y, max_y = min(all_y), max(all_y)

# Create figure and subplots
fig, (ax_odom, ax_imu, ax_combined) = plt.subplots(3, 1, figsize=(8, 10), sharex=True, sharey=True)
plt.subplots_adjust(bottom=0.2, hspace=0.4)

# ---------- ODOMETRY PLOT ----------
odom_line, = ax_odom.plot([], [], 'bo-', label='Odometry')
odom_start, = ax_odom.plot(*odometry_data[0], 'go', label='Start')
odom_end, = ax_odom.plot(*odometry_data[-1], 'saddlebrown', marker='o', label='End')
ax_odom.set_title("Odometry Data")
ax_odom.legend()

# ---------- IMU PLOT ----------
imu_line, = ax_imu.plot([], [], 'ro-', label='IMU')
imu_start, = ax_imu.plot(*imu_data[0], 'go', label='Start')
imu_end, = ax_imu.plot(*imu_data[-1], 'saddlebrown', marker='o', label='End')
ax_imu.set_title("IMU Data")
ax_imu.legend()

# ---------- COMBINED PLOT ----------
combined_odom_line, = ax_combined.plot([], [], 'bo-', label='Odometry')
combined_imu_line, = ax_combined.plot([], [], 'ro-', label='IMU')
combined_start_odom, = ax_combined.plot(*odometry_data[0], 'go', label='Odo Start')
combined_start_imu, = ax_combined.plot(*imu_data[0], 'g^', label='IMU Start')
combined_end_odom, = ax_combined.plot(*odometry_data[-1], 'saddlebrown', marker='o', label='Odo End')
combined_end_imu, = ax_combined.plot(*imu_data[-1], 'sienna', marker='^', label='IMU End')
ax_combined.set_title("Combined Odometry and IMU")
ax_combined.legend()

# Set axis limits for all
for ax in [ax_odom, ax_imu, ax_combined]:
    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

# ---------- SLIDER ----------
slider_ax = plt.axes([0.2, 0.05, 0.6, 0.03])
frame_slider = Slider(slider_ax, 'Frame', 1, max_index, valinit=1, valstep=1)

# ---------- UPDATE FUNCTION ----------
def update(val):
    idx = int(frame_slider.val)
    odo_slice = odometry_data[:idx]
    imu_slice = imu_data[:idx]
    
    # Update odometry plot
    x_odo, y_odo = zip(*odo_slice)
    odom_line.set_data(x_odo, y_odo)
    
    # Update imu plot
    x_imu, y_imu = zip(*imu_slice)
    imu_line.set_data(x_imu, y_imu)
    
    # Update combined plot
    combined_odom_line.set_data(x_odo, y_odo)
    combined_imu_line.set_data(x_imu, y_imu)

    fig.canvas.draw_idle()

# Connect slider
frame_slider.on_changed(update)

# Initial draw
update(1)
plt.show()
