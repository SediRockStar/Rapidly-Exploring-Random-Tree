import matplotlib.pyplot as plt
import numpy as np
# Sample data
path_length_simple= [1286.53, 1284.42, 1282, 1246.88, 1220.66, 1246.79, 1210.52, 1156.46, 1200.53, 1140.7, 1171.19]
path_length= [1329.81, 1262.47, 1254.03, 1234.44, 1234.25, 1208.02, 1145.3, 1216.05, 1204.15, 1182.78, 1171.45]
step_size= [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55]

# Set the bar width
bar_width = 0.35
plt.figure(figsize=(10,5))
# Create bar plots for both path lengths
index = np.arange(len(step_size))
bars_path_length = plt.bar(index, path_length, color='blue', width=bar_width, label='Path Length')
bars_path_length_simple = plt.bar(index + bar_width, path_length_simple, color='orange', width=bar_width, label='Simple Path Length')

# Display values on top of each bar for path_length
for bar, value in zip(bars_path_length, path_length):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5, str(value), ha='center')

# Display values on top of each bar for path_length_simple
for bar, value in zip(bars_path_length_simple, path_length_simple):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5, str(value), ha='center')

# Add title, labels, and legend
plt.title('RRT Path Length vs Step Size')
plt.xlabel('Step Size')
plt.ylabel('Path Length')
plt.xticks(index + bar_width / 2, step_size)
plt.legend()

# Save the plot to a file (you can change the format if needed)
plt.savefig('rrt_path_length_vs_step_size.png')
# Show the plot
plt.show()

iterations_simple= [6289, 2339, 1219, 658, 431, 289, 200, 148, 163, 130, 97]
iterations= [ 8520, 3543, 1949, 1336, 894, 663, 499, 462, 406, 325, 299]
bar_width= 0.4
# Create bar plots for both iterations
index = np.arange(len(step_size))
bars_iterations = plt.bar(index, iterations, color='red', width=bar_width, label='Iterations')
bars_iterations_simple = plt.bar(index + bar_width, iterations_simple, color='green', width=bar_width, label='Simple Iterations')

# Display values on top of each bar for iterations
for bar, value in zip(bars_iterations, iterations):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5, str(value), ha='center')

# Display values on top of each bar for iterations_simple
for bar, value in zip(bars_iterations_simple, iterations_simple):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5, str(value), ha='center')

# Add title, labels, and legend
plt.title('Num-rrt-Iterations vs Step Size')
plt.xlabel('Step Size')
plt.ylabel('Number of Iterations')
plt.xticks(index + bar_width / 2, step_size)
plt.legend()
# Save the plot to a file (you can change the format if needed)
plt.savefig('num_rrt_iterations_vs_step_size.png')
# Show the plot
plt.show()

# Sample data
line_robot_iteration = [4524, 5849, 6433, 5134, 5125, 6402, 4016, 5479, 6367, 5499]
line_robot_simple_iteration = [4299, 4900, 4175, 3939, 4637, 5076, 3766, 3479, 4663, 5161]
line_robot_length = [5, 10, 15, 20, 25, 30, 35, 40, 45, 50]

# Set the bar width
bar_width = 0.35

# Create bar plots for both line_robot_iteration and line_robot_simple_iteration
index = np.arange(len(line_robot_length))
bars_line_robot_iteration = plt.bar(index, line_robot_iteration, color='green', width=bar_width, label='RRT Iterations')
bars_line_robot_simple_iteration = plt.bar(index + bar_width, line_robot_simple_iteration, color='blue', width=bar_width, label='Simple RRT Iterations')

# Display values on top of each bar for line_robot_iteration
for bar, value in zip(bars_line_robot_iteration, line_robot_iteration):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5, str(value), ha='center')

# Display values on top of each bar for line_robot_simple_iteration
for bar, value in zip(bars_line_robot_simple_iteration, line_robot_simple_iteration):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5, str(value), ha='center')

# Add title, labels, and legend
plt.title('Num-rrt-Iterations vs Robot Length')
plt.xlabel('Robot Length')
plt.ylabel('Number of RRT Iterations')
plt.xticks(index + bar_width / 2, line_robot_length)
plt.legend()
# Save the plot to a file (you can change the format if needed)
plt.savefig('num_rrt_iterations_vs_robot_length.png')
# Show the plot
plt.show()
