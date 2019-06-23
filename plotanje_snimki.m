clc
clear all
mass_0_command = importdata('mass_0_command.txt');
mass_1_command = importdata('mass_1_command.txt');
mass_2_command = importdata('mass_2_command.txt');
mass_3_command = importdata('mass_3_command.txt');

mass_0_pos = importdata('mass_0_position.txt');
mass_1_pos = importdata('mass_1_position.txt');
mass_2_pos = importdata('mass_2_position.txt');
mass_3_pos = importdata('mass_3_position.txt');


mass_0_vel = importdata('mass_0_velocity.txt');
mass_1_vel = importdata('mass_1_velocity.txt');
mass_2_vel = importdata('mass_2_velocity.txt');
mass_3_vel = importdata('mass_3_velocity.txt');


figure();
plot(1:length(mass_0_command.data(:,2)), mass_0_command.data(:,2), 'LineWidth', 3);
hold on;
plot(1:length(mass_0_pos.data(:,2)), mass_0_pos.data(:,2), 'LineWidth', 3);
plot(1:length(mass_0_vel.data(:,2)), mass_0_vel.data(:,2), 'LineWidth', 3);
legend('mass_0 command', 'mass_0 position', 'mass_0 vel');

figure();
plot(1:length(mass_1_command.data(:,2)), mass_1_command.data(:,2), 'LineWidth', 3);
hold on;
plot(1:length(mass_1_pos.data(:,2)), mass_1_pos.data(:,2), 'LineWidth', 3);
plot(1:length(mass_1_vel.data(:,2)), mass_1_vel.data(:,2), 'LineWidth', 3);
legend('mass_1 command', 'mass_1 position', 'mass_1 vel');


figure();
plot(1:length(mass_2_command.data(:,2)), mass_2_command.data(:,2), 'LineWidth', 3);
hold on;
plot(1:length(mass_2_pos.data(:,2)), mass_2_pos.data(:,2), 'LineWidth', 3);
plot(1:length(mass_2_vel.data(:,2)), mass_2_vel.data(:,2), 'LineWidth', 3);
legend('mass_2 command', 'mass_2 position', 'mass_2 vel');

figure();
plot(1:length(mass_3_command.data(:,2)), mass_3_command.data(:,2), 'LineWidth', 3);
hold on;
plot(1:length(mass_3_pos.data(:,2)), mass_3_pos.data(:,2), 'LineWidth', 3);
plot(1:length(mass_3_vel.data(:,2)), mass_3_vel.data(:,2), 'LineWidth', 3);
legend('mass_3 command', 'mass_3 position', 'mass_3 vel');

angles = importdata('angles.txt');
euler_ref = importdata('euler_ref.txt');

figure();
plot(1:length(angles.data(:,5)), angles.data(:,5), 'LineWidth', 3);
hold on;
plot(1:length(angles.data(:,8)), angles.data(:,8), 'LineWidth', 3);

plot(1:length(euler_ref.data(:,2)), euler_ref.data(:,2), 'LineWidth', 3);
legend('roll', 'roll rate', 'roll ref');

figure();
plot(1:length(angles.data(:,6)), angles.data(:,6), 'LineWidth', 3);
hold on;
plot(1:length(angles.data(9)), angles.data(9), 'LineWidth', 3);

plot(1:length(euler_ref.data(:,3)), euler_ref.data(:,3), 'LineWidth', 3);
legend('pitch', 'pitch rate', 'pitch ref');

motor_speed_attitude_command = importdata('motor_speed_attitude_command.txt');
motor_speed_command = importdata('motor_speed_command.txt');
motor_speed_height_command = importdata('motor_speed_height_command.txt');
motor_speed_state = importdata('motor_speed_state.txt');

figure();
plot(1:length(motor_speed_attitude_command.data(:,5)), motor_speed_attitude_command.data(:,5), 'LineWidth', 3);
hold on;
plot(1:length(motor_speed_command.data(:,5)), motor_speed_command.data(:,5), 'LineWidth', 3);
plot(1:length(motor_speed_height_command.data(:,5)), motor_speed_height_command.data(:,5), 'LineWidth', 3);
plot(1:length(motor_speed_state.data(1)), motor_speed_state.data(1), 'LineWidth', 3);

legend('motor_0 speed attitude command', 'motor_0 speed command','motor_0 speed height command', 'motor_0 speed state' )


figure();
plot(1:length(motor_speed_attitude_command.data(:,6)), motor_speed_attitude_command.data(:,6), 'LineWidth', 3);
hold on;
plot(1:length(motor_speed_command.data(:,6)), motor_speed_command.data(:,6), 'LineWidth', 3);
plot(1:length(motor_speed_height_command.data(:,6)), motor_speed_height_command.data(:,6), 'LineWidth', 3);
plot(1:length(motor_speed_state.data(:,2)), motor_speed_state.data(:,2), 'LineWidth', 3);

legend('motor_1 speed attitude command', 'motor_1 speed command','motor_1 speed height command', 'motor_1 speed state' )


figure();
plot(1:length(motor_speed_attitude_command.data(:,7)), motor_speed_attitude_command.data(:,7), 'LineWidth', 3);
hold on;
plot(1:length(motor_speed_command.data(:,7)), motor_speed_command.data(:,7), 'LineWidth', 3);
plot(1:length(motor_speed_height_command.data(:,7)), motor_speed_height_command.data(:,7), 'LineWidth', 3);
plot(1:length(motor_speed_state.data(:,3)), motor_speed_state.data(:,3), 'LineWidth', 3);

legend('motor_2 speed attitude command', 'motor_2 speed command','motor_2 speed height command', 'motor_2 speed state' )


figure();
plot(1:length(motor_speed_attitude_command.data(:,8)), motor_speed_attitude_command.data(:,8), 'LineWidth', 3);
hold on;
plot(1:length(motor_speed_command.data(:,8)), motor_speed_command.data(:,8), 'LineWidth', 3);
plot(1:length(motor_speed_height_command.data(:,8)), motor_speed_height_command.data(:,8), 'LineWidth', 3);
plot(1:length(motor_speed_state.data(:,4)), motor_speed_state.data(:,4), 'LineWidth', 3);

legend('motor_3 speed attitude command', 'motor_3 speed command','motor_3 speed height command', 'motor_3 speed state' )


pose = importdata('pose.txt');
pose_ref = importdata('pose_ref.txt');
position = importdata('position.txt');

figure();
plot(1:length(pose_ref.data(:,2)), pose_ref.data(:,2), 'LineWidth', 3);
hold on;
plot(1:length(position.data(:,1)), position.data(:,1), 'LineWidth', 3);
legend('position ref x', 'position  x' )


figure();
plot(1:length(pose_ref.data(:,3)), pose_ref.data(:,3), 'LineWidth', 3);
hold on;
plot(1:length(position.data(:,2)), position.data(:,2), 'LineWidth', 3);
legend('position ref y', 'position  y' )

figure();
plot(1:length(pose_ref.data(:,4)), pose_ref.data(:,4), 'LineWidth', 3);
hold on;
plot(1:length(position.data(:,3)), position.data(:,3), 'LineWidth', 3);
legend('position ref z', 'position  z' )

target_input = importdata('target_input.txt');
target_states = importdata('target_states.txt');
velocity = importdata('velocity.txt');

figure();
plot(1:length(velocity.data(:,5)), velocity.data(:,5), 'LineWidth', 3);
hold on;
plot(1:length(velocity.data(:,6)), velocity.data(:,6), 'LineWidth', 3);
plot(1:length(velocity.data(:,7)), velocity.data(:,7), 'LineWidth', 3);

legend('linear x', 'linear y','linear z' )

figure();
plot(1:length(velocity.data(:,8)), velocity.data(:,8), 'LineWidth', 3);
hold on;
plot(1:length(velocity.data(:,9)), velocity.data(:,9), 'LineWidth', 3);
plot(1:length(velocity.data(:,10)), velocity.data(:,10), 'LineWidth', 3);

legend('angular x', 'angular y','angular z' )