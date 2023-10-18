#!/bin/bash


# Define the CSV file
output_csv1="rrt_point_shot_output.csv"

for step_size in 5 10 15 20 25 30 35 40 45 50 55
 do
    echo "Running with step size: $step_size"
    total_iterations=0
    total_path_length=0.0
    for i in 1 2 3 4 5 6 7 8 9 10
    do
      output=$(python3 rrt_planner_point_robot.py --start_pos_x 10 --start_pos_y 270 --target_pos_x 900 --target_pos_y 30 --step_size $step_size --rrt_sampling_policy gaussian --seed $i)

      # Extract path length from the output (assuming it's in the format a,b,c)

      path_length=$(echo $output | cut -d ',' -f 1)
      iterations=$(echo $output | cut -d ',' -f 3)
      total_iterations=$((total_iterations + iterations))
      total_path_length=$(echo "$total_path_length + $path_length" | bc)
    done
    average_iterations=$((total_iterations / 10))
    average_path_length=$(awk "BEGIN {print $total_path_length / 10}")
    echo "$average_iterations, $average_path_length" >> $output_csv1
done

# Define the CSV file
output_csv2="rrt_point_simple_output.csv"

for step_size in 5 10 15 20 25 30 35 40 45 50 55
 do
    echo "Running with step size: $step_size"
    total_iterations=0
    total_path_length=0.0
    for i in 1 2 3 4 5 6 7 8 9 10
    do
      output=$(python3 rrt_planner_point_robot.py --start_pos_x 10 --start_pos_y 270 --target_pos_x 900 --target_pos_y 30 --world simple.png --step_size $step_size --rrt_sampling_policy gaussian --seed $i)


      path_length=$(echo $output | cut -d ',' -f 1)
      iterations=$(echo $output | cut -d ',' -f 3)
      total_iterations=$((total_iterations + iterations))
      total_path_length=$(echo "$total_path_length + $path_length" | bc)
    done
    average_iterations=$((total_iterations / 10))
    average_path_length=$(awk "BEGIN {print $total_path_length / 10}")
    echo "$average_iterations, $average_path_length" >> $output_csv2
done