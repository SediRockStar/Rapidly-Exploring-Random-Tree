# Define the CSV file
output_csv1="rrt_line_shot_output.csv"

for robot_length in 5 10 15 20 25 30 35 40 45 50
 do
    echo "Running with robot length: $robot_length"
    total_iterations=0
    for i in 1 2 3 4 5 6 7 8 9 10
    do
      output=$(python3 rrt_planner_line_robot.py --start_pos_x 50 --start_pos_y 630 --start_pos_theta 5 --robot_length $robot_length --target_pos_x 800 --target_pos_y 150 --seed $i --visualize False)

      # Extract path length from the output (assuming it's in the format a,b,c)
      iterations=$(echo $output)
      total_iterations=$((total_iterations + iterations))

      #echo "$iterations" >> $output_csv
    done
    average_iterations=$((total_iterations / 10))
    echo "$average_iterations" >> $output_csv1
done

output_csv2="rrt_line_simple_output.csv"

for robot_length in 5 10 15 20 25 30 35 40 45 50
 do
    echo "Running with robot length: $robot_length"
    total_iterations=0
    for i in 1 2 3 4 5 6 7 8 9 10
    do
      output=$(python3 rrt_planner_line_robot.py --start_pos_x 50 --start_pos_y 630 --start_pos_theta 5 --robot_length $robot_length --target_pos_x 800 --target_pos_y 150 --seed $i --visualize False --world simple.png)

      # Extract path length from the output (assuming it's in the format a,b,c)
      iterations=$(echo $output)
      total_iterations=$((total_iterations + iterations))
      #echo "$iterations" >> $output_csv
    done
    average_iterations=$((total_iterations / 10))
    echo "$average_iterations" >> $output_csv2
done