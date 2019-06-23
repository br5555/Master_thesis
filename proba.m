format long
delimiterIn = ',';

filename = 'delta_omega_i.txt';
delta_omega_i = importdata(filename, delimiterIn);

filename = 'delta_omega_i_2.txt';
delta_omega_i_2 = importdata(filename, delimiterIn);

filename = 'dx_i.txt';
dx_i = importdata(filename, delimiterIn);

filename = 'dx_i_2.txt';
dx_i_2 = importdata(filename, delimiterIn);

filename = 'iteration1_const.txt';
iteration1_const = importdata(filename, delimiterIn);

filename = 'iteration2_const.txt';
iteration2_const = importdata(filename, delimiterIn);

filename = 'iteration3_const.txt';
iteration3_const = importdata(filename, delimiterIn);

filename = 'iteration4_const.txt';
iteration4_const = importdata(filename, delimiterIn);

filename = 'iteration5_const.txt';
iteration5_const = importdata(filename, delimiterIn);

filename = 'iteration6_const.txt';
iteration6_const = importdata(filename, delimiterIn);

filename = 'iteration1_disk.txt';
iteration1_disk = importdata(filename, delimiterIn);

filename = 'iteration2_disk.txt';
iteration2_disk = importdata(filename, delimiterIn);

filename = 'iteration3_disk.txt';
iteration3_disk = importdata(filename, delimiterIn);

filename = 'iteration4_disk.txt';
iteration4_disk = importdata(filename, delimiterIn);

filename = 'iteration5_disk.txt';
iteration5_disk = importdata(filename, delimiterIn);

filename = 'iteration6_disk.txt';
iteration6_disk = importdata(filename, delimiterIn);

filename = 'residuum_state_1_const.txt';
residuum_state_1_const = importdata(filename, delimiterIn);

filename = 'residuum_state_2_const.txt';
residuum_state_2_const = importdata(filename, delimiterIn);

filename = 'residuum_state_3_const.txt';
residuum_state_3_const = importdata(filename, delimiterIn);

filename = 'residuum_state_4_const.txt';
residuum_state_4_const = importdata(filename, delimiterIn);

filename = 'residuum_state_5_const.txt';
residuum_state_5_const = importdata(filename, delimiterIn);

filename = 'residuum_state_6_const.txt';
residuum_state_6_const = importdata(filename, delimiterIn);

filename = 'residuum_state_1_disk.txt';
residuum_state_1_disk = importdata(filename, delimiterIn);

filename = 'residuum_state_2_disk.txt';
residuum_state_2_disk = importdata(filename, delimiterIn);

filename = 'residuum_state_3_disk.txt';
residuum_state_3_disk = importdata(filename, delimiterIn);

filename = 'residuum_state_4_disk.txt';
residuum_state_4_disk = importdata(filename, delimiterIn);

filename = 'residuum_state_5_disk.txt';
residuum_state_5_disk = importdata(filename, delimiterIn);

filename = 'residuum_state_6_disk.txt';
residuum_state_6_disk = importdata(filename, delimiterIn);

filename = 'residuum_input_1_const.txt';
residuum_input_1_const = importdata(filename, delimiterIn);

filename = 'residuum_input_2_const.txt';
residuum_input_2_const = importdata(filename, delimiterIn);

filename = 'residuum_input_3_const.txt';
residuum_input_3_const = importdata(filename, delimiterIn);

filename = 'residuum_input_4_const.txt';
residuum_input_4_const = importdata(filename, delimiterIn);

filename = 'residuum_input_5_const.txt';
residuum_input_5_const = importdata(filename, delimiterIn);

filename = 'residuum_input_6_const.txt';
residuum_input_6_const = importdata(filename, delimiterIn);

filename = 'residuum_input_1_disk.txt';
residuum_input_1_disk = importdata(filename, delimiterIn);


filename = 'residuum_input_2_disk.txt';
residuum_input_2_disk = importdata(filename, delimiterIn);


filename = 'residuum_input_3_disk.txt';
residuum_input_3_disk = importdata(filename, delimiterIn);


filename = 'residuum_input_4_disk.txt';
residuum_input_4_disk = importdata(filename, delimiterIn);


filename = 'residuum_input_5_disk.txt';
residuum_input_5_disk = importdata(filename, delimiterIn);


filename = 'residuum_input_6_disk.txt';
residuum_input_6_disk = importdata(filename, delimiterIn);

filename = 'traget_input_const.txt';
traget_input_const = importdata(filename, delimiterIn);

filename = 'traget_input_disk.txt';
traget_input_disk = importdata(filename, delimiterIn);

filename = 'traget_state_const.txt';
traget_state_const = importdata(filename, delimiterIn);

filename = 'traget_state_disk.txt';
traget_state_disk = importdata(filename, delimiterIn);

filename = 'x1_cons.txt';
x1_cons = importdata(filename, delimiterIn);

filename = 'x2_cons.txt';
x2_cons = importdata(filename, delimiterIn);

filename = 'x3_cons.txt';
x3_cons = importdata(filename, delimiterIn);

filename = 'x4_cons.txt';
x4_cons = importdata(filename, delimiterIn);

filename = 'x5_cons.txt';
x5_cons = importdata(filename, delimiterIn);

filename = 'x6_cons.txt';
x6_cons = importdata(filename, delimiterIn);

filename = 'x1_disk.txt';
x1_disk = importdata(filename, delimiterIn);

filename = 'x2_disk.txt';
x2_disk = importdata(filename, delimiterIn);

filename = 'x3_disk.txt';
x3_disk = importdata(filename, delimiterIn);

filename = 'x4_disk.txt';
x4_disk = importdata(filename, delimiterIn);

filename = 'x5_disk.txt';
x5_disk = importdata(filename, delimiterIn);

filename = 'x6_disk.txt';
x6_disk = importdata(filename, delimiterIn);
%%
%figure(); plot(iteration1_const.data(:,1)-iteration1_const.data(1,1), iteration1_const.data(:,2),'Linewidth',6); legend('iteration1_const'); title('iterations_{const}');

figure(); plot(residuum_input_3_const.data(:,1)-residuum_input_3_const.data(1,1), residuum_input_3_const.data(:,2),'Linewidth',3)
hold on;
plot(residuum_state_3_const.data(:,1)-residuum_state_3_const.data(1,1), residuum_state_3_const.data(:,2),'Linewidth',3); legend('residuum_{input}', 'residuum_{state} '); title('residuum');

%%
figure(); plot(traget_state_const.data(:,1)-traget_state_const.data(1,1), traget_state_const.data(:,2),'Linewidth',3)
hold on;
plot(traget_state_const.data(:,1)-traget_state_const.data(1,1), traget_state_const.data(:,4),'Linewidth',3)
plot(traget_state_const.data(:,1)-traget_state_const.data(1,1), traget_state_const.data(:,6),'Linewidth',3)
plot(traget_state_const.data(:,1)-traget_state_const.data(1,1), traget_state_const.data(:,7),'Linewidth',3)
plot(traget_state_const.data(:,1)-traget_state_const.data(1,1), traget_state_const.data(:,8),'Linewidth',3)
legend('x_i', 'x_{i_2} ','omega_i', 'omega_{i_2} ', 'referenca'); title('target state');

%%
figure(); plot(x1_cons.data(:,1)-x1_cons.data(1,1), x1_cons.data(:,2),'Linewidth',3)
hold on;
plot(x1_cons.data(:,1)-x1_cons.data(1,1), x1_cons.data(:,1+12),'Linewidth',3)
plot(x1_cons.data(:,1)-x1_cons.data(1,1), x1_cons.data(:,1+2*12),'Linewidth',3)
plot(x1_cons.data(:,1)-x1_cons.data(1,1), x1_cons.data(:,1+3*12),'Linewidth',3)
legend('x_i', 'x_{i_2} ','delta omega_i', 'delta omega_{i_2} '); title('target state');