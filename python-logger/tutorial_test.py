import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# plt.style.use('fivethirtyeight')

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

plot1_xlim = [0, 1000]
plot1_ylim = [-10, 10]
plot1_labels = [
    'set_point', 
    'measurement', 
    'error', 
] 

plot2_xlim = [0, 1000]
plot2_ylim = [-10, 10]
plot2_labels = [
    'proportional', 
    'integrator', 
    'differentiator', 
    'output'
]

data = pd.read_csv('data.csv')
data_values = {}
for idx in range(len(data_labels)):
    data_values[data_labels[idx]] = data[data_labels[idx]]
time_index = data_values[data_labels[0]]

fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, sharex=True)


for label in plot1_labels:
    ax1.plot(time_index, data_values[label], label=label)

for label in plot2_labels:
    ax2.plot(time_index, data_values[label], label=label)


def animate(i):

    for line in ax1.lines:
        x = time_index
        y = data[line.get_label()]
        line.set_data(x, y)

    for line in ax2.lines:
        x = time_index
        y = data[line.get_label()]
        line.set_data(x, y) 


#     y_min = 0
#     y_max = 0

#     # y_min = y.min() if (y.min() < y_min) else y_min
#     # y_max = y.max() if (y.max() > y_max) else y_max

#     # xlim_low, xlim_high = ax.get_xlim()
#     # ylim_low, ylim_high = ax.get_ylim()

#     # ax.set_xlim(xlim_low, (x.max() + 5))
#     # ax.set_ylim((y_min - 5), (y_max + 5))


ax1.legend()
ax1.grid(True)
ax1.set_xlim(plot1_xlim)
ax1.set_ylim(plot1_ylim)

ax2.legend()
ax2.grid(True)
ax2.set_xlim(plot2_xlim)
ax2.set_ylim(plot2_ylim)

ani = FuncAnimation(fig, animate, interval=200, frames=50)

plt.tight_layout()
plt.show()