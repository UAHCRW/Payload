%% initialization, find biases
fs=100;%sampling frequency

gyro_offset=0;
acc_offset=125*9.81e-3;%given in mg
g_bias=9.81;%m/s, gravity offset

%% initialization of matrices
%UPDATE THESE WITH MAG
pos_old=[0,0,0];
vel_old=[0,0,0];
C_old=[1,0,0; 0,1,0; 0,0,1];
dt=1;%dummy time step value, I don't know what it is


%% algorithim proper
%NOTE probably need some loop here, whether it reads values from an 
%array or or as we go is up to you
%also probably needs filtering and integration with pitot system

%assumes two input vectors, both x y z, for gyro and acc
%first remove bias
acc_vals=input_acc-acc_offset;
gyro_vals=input_gyro-gyro_offset;

%find rotation matrix
C_new=Find_Rat_Mat(C_old,gyro_vals,dt);

%rotate the acceleration
dummy=C_new*acc_vals';%need to use transpose so matrix math works
acceleration=dummy';

%remove gravity
acceleration(3)=acceleration(3)+g_bias;%z axis is vertical and points in direction of ground

velocity=Euler_integration(vel_old,acceleration,dt);
position=Euler_integration(pos_old,velocity,dt);

%end loop

%% Compare to grid square 
%going to work on this