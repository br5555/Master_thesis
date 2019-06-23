# script for saving ROS topics

folder=$1 # define the save folder
mkdir -p $folder  # create a save folder
# mkdir $(date '+%d-%b-%Y') # save the folder with current date

# define the save topics which are goind to be saved


rostopic echo -p /my_debug/traget_input_const> "$folder/traget_input_const.txt" & 
rostopic echo -p /my_debug/traget_input_disk> "$folder/traget_input_disk.txt" & 
rostopic echo -p /my_debug/traget_state_const> "$folder/traget_state_const.txt" & 
rostopic echo -p /my_debug/traget_state_disk> "$folder/traget_state_disk.txt" & 

rostopic echo -p /my_debug/x1_cons> "$folder/x1_cons.txt" & 
rostopic echo -p /my_debug/x2_cons> "$folder/x2_cons.txt" &
rostopic echo -p /my_debug/x3_cons> "$folder/x3_cons.txt" &
rostopic echo -p /my_debug/x4_cons> "$folder/x4_cons.txt" &
rostopic echo -p /my_debug/x5_cons> "$folder/x5_cons.txt" &
rostopic echo -p /my_debug/x6_cons> "$folder/x6_cons.txt" &

rostopic echo -p /my_debug/x1_disk> "$folder/x1_disk.txt" &
rostopic echo -p /my_debug/x2_disk> "$folder/x2_disk.txt" &
rostopic echo -p /my_debug/x3_disk> "$folder/x3_disk.txt" &
rostopic echo -p /my_debug/x4_disk> "$folder/x4_disk.txt" &
rostopic echo -p /my_debug/x5_disk> "$folder/x5_disk.txt" &
rostopic echo -p /my_debug/x6_disk> "$folder/x6_disk.txt" &

rostopic echo -p /my_debug/residuum_state_1_disk> "$folder/residuum_state_1_disk.txt" & 
rostopic echo -p /my_debug/residuum_state_2_disk> "$folder/residuum_state_2_disk.txt" & 
rostopic echo -p /my_debug/residuum_state_3_disk> "$folder/residuum_state_3_disk.txt" & 
rostopic echo -p /my_debug/residuum_state_4_disk> "$folder/residuum_state_4_disk.txt" & 
rostopic echo -p /my_debug/residuum_state_5_disk> "$folder/residuum_state_5_disk.txt" & 
rostopic echo -p /my_debug/residuum_state_6_disk> "$folder/residuum_state_6_disk.txt" & 

rostopic echo -p /my_debug/residuum_state_1_const> "$folder/residuum_state_1_const.txt" & 
rostopic echo -p /my_debug/residuum_state_2_const> "$folder/residuum_state_2_const.txt" & 
rostopic echo -p /my_debug/residuum_state_3_const> "$folder/residuum_state_3_const.txt" & 
rostopic echo -p /my_debug/residuum_state_4_const> "$folder/residuum_state_4_const.txt" & 
rostopic echo -p /my_debug/residuum_state_5_const> "$folder/residuum_state_5_const.txt" & 
rostopic echo -p /my_debug/residuum_state_6_const> "$folder/residuum_state_6_const.txt" & 

rostopic echo -p /my_debug/residuum_input_1_disk> "$folder/residuum_input_1_disk.txt" & 
rostopic echo -p /my_debug/residuum_input_2_disk> "$folder/residuum_input_2_disk.txt" & 
rostopic echo -p /my_debug/residuum_input_3_disk> "$folder/residuum_input_3_disk.txt" & 
rostopic echo -p /my_debug/residuum_input_4_disk> "$folder/residuum_input_4_disk.txt" & 
rostopic echo -p /my_debug/residuum_input_5_disk> "$folder/residuum_input_5_disk.txt" & 
rostopic echo -p /my_debug/residuum_input_6_disk> "$folder/residuum_input_6_disk.txt" & 



rostopic echo -p /my_debug/residuum_input_1_const> "$folder/residuum_input_1_const.txt" & 
rostopic echo -p /my_debug/residuum_input_2_const> "$folder/residuum_input_2_const.txt" & 
rostopic echo -p /my_debug/residuum_input_3_const> "$folder/residuum_input_3_const.txt" & 
rostopic echo -p /my_debug/residuum_input_4_const> "$folder/residuum_input_4_const.txt" & 
rostopic echo -p /my_debug/residuum_input_5_const> "$folder/residuum_input_5_const.txt" & 
rostopic echo -p /my_debug/residuum_input_6_const> "$folder/residuum_input_6_const.txt" & 

rostopic echo -p /my_debug/iteration1_const> "$folder/iteration1_const.txt" & 
rostopic echo -p /my_debug/iteration2_const> "$folder/iteration2_const.txt" & 
rostopic echo -p /my_debug/iteration3_const> "$folder/iteration3_const.txt" & 
rostopic echo -p /my_debug/iteration4_const> "$folder/iteration4_const.txt" & 
rostopic echo -p /my_debug/iteration5_const> "$folder/iteration5_const.txt" & 
rostopic echo -p /my_debug/iteration6_const> "$folder/iteration6_const.txt" & 

rostopic echo -p /my_debug/iteration1_disk> "$folder/iteration1_disk.txt" & 
rostopic echo -p /my_debug/iteration2_disk> "$folder/iteration2_disk.txt" & 
rostopic echo -p /my_debug/iteration3_disk> "$folder/iteration3_disk.txt" & 
rostopic echo -p /my_debug/iteration4_disk> "$folder/iteration4_disk.txt" & 
rostopic echo -p /my_debug/iteration5_disk> "$folder/iteration5_disk.txt" & 
rostopic echo -p /my_debug/iteration6_disk> "$folder/iteration6_disk.txt" & 

# current state of the UAV - pose
rostopic echo -p /my_debug/delta_omega_i> "$folder/delta_omega_i.txt" &  
rostopic echo -p /my_debug/delta_omega_i_2> "$folder/delta_omega_i_2.txt" & 
rostopic echo -p /my_debug/dx_i> "$folder/dx_i.txt" & 
rostopic echo -p /my_debug/dx_i_2> "$folder/dx_i_2.txt" &

rostopic echo -p /my_debug/m_i> "$folder/m_i.txt" &  
rostopic echo -p /my_debug/m_i_2> "$folder/m_i_2.txt" & 
rostopic echo -p /my_debug/omega_i> "$folder/omega_i.txt" & 
rostopic echo -p /my_debug/omega_i_2> "$folder/omega_i_2.txt" &

rostopic echo -p /my_debug/referenca> "$folder/referenca.txt" &  
rostopic echo -p /my_debug/theta> "$folder/theta.txt" & 
rostopic echo -p /my_debug/x_i> "$folder/x_i.txt" & 
rostopic echo -p /my_debug/x_i_2> "$folder/x_i_2.txt" &

wait
kill $(jobs -p)
wait
