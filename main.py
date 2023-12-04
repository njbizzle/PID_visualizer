import math, time
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import numpy as np


class PID_Controller:
    def __init__(self, p, i, d):
        self.kP = p
        self.kI = i
        self.kD = d

        self.error_sum = 0

        self.current_error = 0
        self.last_error = 0

    def set_pid(self, p, i, d):
        self.kP = p
        self.kI = i
        self.kD = d

        self.reset_error_sum()

    def reset_error_sum(self):
        self.error_sum = 0

    def calculate(self, error):
        self.current_error = error
        self.error_sum += error

        output = sum([
            self.kP * self.calculate_proportional(),
            self.kI * self.calculate_integral(),
            self.kD * self.calculate_derivative()
        ])

        self.last_error = error
        return output

    def calculate_proportional(self):
        return self.current_error

    def calculate_integral(self):
        return self.error_sum

    def calculate_derivative(self):
        if self.last_error == 0:
            return 0  # GD REFERENCE

        return self.current_error - self.last_error


class Simulation():
    def __init__(self, damping):
        self.position = 0
        self.velocity = 0

        self.damping = damping

    def reset(self):
        self.position = 0
        self.velocity = 0

    def simulation_update(self, value, time_step):
        self.velocity += value
        self.velocity *= self.damping

        self.position += self.velocity * time_step


simulation = Simulation(0.99)
pid = PID_Controller(0, 0, 0)

target_position = 0
speed = 5
amps = 10

time_step = 0.01
graph_size = 1000

plt.ion()

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.3, left=0.2)


y_span = [2*i-graph_size/2 for i in range(graph_size)]

simulation_position_line, = ax.plot(
    range(graph_size),
    y_span,
    label="Simulation Position"
)

target_line, = ax.plot(
    range(graph_size),
    [target_position for _ in range(graph_size)],
    label="Simulation Position"
)

ax_p = fig.add_axes([0.05, 0.15, 0.9, 0.03])
ax_i = fig.add_axes([0.05, 0.1, 0.9, 0.03])
ax_d = fig.add_axes([0.05, 0.05, 0.9, 0.03])

p_slider = Slider(ax_p, "p", 0, 10, valinit=0)
i_slider = Slider(ax_i, "i", 0, 1, valinit=0)
d_slider = Slider(ax_d, "d", 0, 10, valinit=0)

ax_speed = fig.add_axes([0.025, 0.25, 0.03, 0.65])
speed_slider = Slider(
    ax_speed, "speed", 0, max(graph_size/100, 10), valinit=max(graph_size/200, 1), orientation="vertical"
)

ax_target = fig.add_axes([0.075, 0.25, 0.03, 0.65])
target_slider = Slider(
    ax_target, "target", min(y_span), max(y_span), valinit=graph_size/2, orientation="vertical"
)

simulation_positions = [0 for _ in range(graph_size)]


def update():
    global simulation_positions, target_position, speed

    speed = speed_slider.val

    if target_position != target_slider.val:
        pid.reset_error_sum()
        target_position = target_slider.val
        target_line.set_ydata([target_position for _ in range(graph_size)])

    if pid.kP != p_slider.val or pid.kI != i_slider.val or pid.kD != d_slider.val:
        pid.set_pid(
            p_slider.val,
            i_slider.val,
            d_slider.val
        )

    for i in range(int(speed)):
        pid_out = pid.calculate(target_position - simulation.position)

        simulation.simulation_update(pid_out, time_step)

        simulation_positions = simulation_positions[1:]
        simulation_positions.append(simulation.position)

    simulation_position_line.set_ydata(simulation_positions)

    return True # might return false later to quit


while update():
    fig.canvas.draw()
    fig.canvas.flush_events()

    time.sleep(time_step)
