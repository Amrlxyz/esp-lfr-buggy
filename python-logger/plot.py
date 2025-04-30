import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

display_points = 10000

data_labels = [
    'time_index',
    'set_point', 
    'measurement', 
    'error', 
    'proportional', 
    'integrator', 
    'differentiator', 
    'output'
]

plot1_ylim_offset = 0.2
plot1_labels = [
    'set_point', 
    'measurement', 
    'error', 
] 

plot2_ylim_offset = 0.1
plot2_labels = [
    'proportional', 
    'integrator', 
    'differentiator', 
    'output'
]

plot1_xlim = [0, 3]
plot1_ylim = [-2, 2]

plot2_xlim = [0, 3]
plot2_ylim = [-2, 2]


fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True)

# Data Processing
data = pd.read_csv("data.csv")
data_values = {}
for idx in range(len(data_labels)):
    if (data_labels[idx] == 'error'):
        data_values[data_labels[idx]] = [(meas-set) for meas, set in zip(data_values['measurement'], data_values['set_point'])]
    elif (data_labels[idx] == 'output'):
        data_values[data_labels[idx]] = [sum(x) for x in zip(data_values['proportional'], data_values['integrator'], data_values['differentiator'])]
    else:
        data_values[data_labels[idx]] = data[data_labels[idx]]

time_index = data_values[data_labels[0]]

for label in plot1_labels:
    ax1.plot(time_index, data_values[label], label=label)

for label in plot2_labels:
    ax2.plot(time_index, data_values[label], label=label)


# # time_index = data_values[data_labels[0]].tail(display_points)

# time_index = data_values[data_labels[0]]
# display_points = len(time_index)

# # for idx in range(len(data_labels) - 1):
# #     ax1.plot(time_index, data_values[data_labels[idx+1]], label=data_labels[idx+1])

# x_min = 0
# x_max = 0
# y1_min = 0
# y1_max = 0
# y2_min = 0
# y2_max = 0

# for line in ax1.lines:
#     x = time_index
#     y = data[line.get_label()].tail(display_points)
#     line.set_data(x, y)
#     y1_min = y.min() if (y.min() < y1_min) else y1_min
#     y1_max = y.max() if (y.max() > y1_max) else y1_max

# for line in ax2.lines:
#     x = time_index
#     y = data[line.get_label()].tail(display_points)
#     line.set_data(x, y) 

#     y2_min = y.min() if (y.min() < y2_min) else y2_min
#     y2_max = y.max() if (y.max() > y2_max) else y2_max

# x_min = time_index.min()
# x_max = time_index.max()

# x_min = x_min if not pd.isna(x_min) else 0
# x_max = x_max if not pd.isna(x_max) else 1
# # y1_min = y1_min if y1_min else plot1_ylim_offset
# # y1_max = y1_max if y1_max else plot1_ylim_offset
# # y2_min = y2_min if y2_min else plot2_ylim_offset
# # y2_max = y2_max if y2_max else plot2_ylim_offset

# ax1.set_xlim(x_min, x_max)
# ax1.set_ylim(y1_min - plot1_ylim_offset, y1_max + plot1_ylim_offset)
# ax2.set_ylim(y2_min - plot2_ylim_offset, y2_max + plot2_ylim_offset)

ax1.legend()
ax1.grid(True)
# ax1.set_xlim(plot1_xlim)
# ax1.set_ylim(plot1_ylim)

ax2.legend()
ax2.grid(True)
# ax2.set_xlim(plot2_xlim)
# ax2.set_ylim(plot2_ylim)

# animate(0)

plt.tight_layout()
plt.show()