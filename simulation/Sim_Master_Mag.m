%% Organization
%Last updated: November 9th 2021, 2:28pm. By Declan Brick

%This sim is organized into a few subsections. Each section likely could
%be split into a seperate file, but by having it in one place I think it
%helps readability the sections are 

%1. Defining the Measurement Parameters
%2. Defining the Model
%3. Defining the Trajectory
%4. Running the Simulation
    %4.1 Simulation Output Processing
%5. Determining Error
%6. Plotting

%The description of each section is below

%1. Defining Mesurement Parameters 
    %This section takes the error parameters of the gyroscope,
    %accelerometer, and magnetometer. I pulled what I could off the
    %datasheets but there likely needs to be more defined values from the
    %allen tests
%2. Defining the Model
    %This section is pretty basic, it just feeds the measurement parameters
    %into the model to create the model object. It also defines a reference
    %magnetic field for the magnetometer
%3. Defining the Trajectory
    %This section is defining the reference trajectory. As the trajectory
    %is pulled from data, it will need to be loaded in through a CSV file.
    %THIS DATA NEEDS TO BE DEFINED IN THE GLOBAL FRAME!
    %Currently, the code treats the accelerometer and gyro values as a
    %single loaded array, but this will need to be changed to 1 column per
    %dimension. Additionally, the velocity and position is calculated by
    %integrating the acceleration. This integration is done via euler
    %method
%4. Running the Simulation
    %This section is a loop to allow for multiple runs. During each run,
    %the raw output and calculated velocity/position are found and stored
%4.1 Simulation Output Processing
    %When the sim outputs data, it is in the body frame and needs to be
    %transformed. Bias is subtracted in the body frame where it is defined.
    %But otherwise ahrs filter is used to find the rotation matrices. The
    %acceleration is then rotated and integrated using euler's method to
    %find the velocity/position
%5. Determining Error
    %This section finds the error components of the position with regards
    %to the reference trajectory and the total error which is just the 
    %magnitude of the components. The max, mean, and standard deviation of
    %the total error is found to characterize the error.
%6. Plotting
    %Read the title, not much other documentation needed. Only plot right
    %now is the total error distribution

%% Defining Measurement Parameters
clear

%gyroscope
noise_den=0.005*(pi/180); %value given in dps/sqrt(Hz) so convert to rad
instab=10*(pi/180)*(1/3600);%value given in deg/h need rad/s
g_parameters=gyroparams('NoiseDensity', noise_den,...
    'BiasInstability', instab);

%accelerometer
offset=125*9.81e-3;%given in mg
white_noise=75*9.81e-6;%given in micro g, correct units for the NoiseDensity
Temp_noise=0.2*9.81e-3;%given in mg
a_instab_x=7.2*9.81e-6;%given in micro mg
a_instab_y=6.5*9.81e-6;%given in micro mg
a_instab_z=9.7*9.81e-6;%given in micro mg

a_parameters=accelparams('ConstantBias',offset,'NoiseDensity',white_noise,...
    'TemperatureBias',Temp_noise,'BiasInstability',[a_instab_x,a_instab_y,a_instab_z]);

%magnetometer
noise_xy=3.2e-3*1e-4;%convert from mGauss to Tesla
noise_z=4.1e-3*1e-4;%convert from mGauss to Tesla

m_parameters=magparams('BiasInstability',[noise_xy,noise_xy,noise_z]);

%% defining the model
IMU=imuSensor('accel-gyro-mag','Gyroscope',g_parameters,'Accelerometer'...
    ,a_parameters,'Magnetometer',m_parameters);

%redefine reference field
%from https://www.geomatrix.co.uk/tools/magnetic-field-calculator/
%with HSV at 34.729542, -86.585297 and 0.183 km alt
%note that the order is North, East, Vertical
IMU.MagneticField=[22.30043,-1.59566,44.8554];%in microT

fs=100;%sampling frequency, in Hz
%% defining the trajectory
runtime=120;%runtime in s
N=(runtime*fs)+1;%number of readings

%define acceleration, 1 m/s^2 in x direction
acc=ones(N,3);
acc(:,2:3)=0;%remove y and z values
%define angular velocity
rpm=60;%rotating once every second
total_rad=rpm*runtime/60;%find total distance traversed
angVel=ones(N,3)*(total_rad/runtime);
angVel(:,1)=0;
angVel(:,3)=0;

times=0:1/fs:runtime;

%load in csv option
%{
import_file='File';
traj_data=importdata(import_file);


%once CSV is loaded, seperate into orientation and acceleration in
%the global frame

%these will need to be changed to pull all axes
acc=traj_data.data(:,find(traj_data.colheaders=="Accelerometer Values"));

angVel=traj_data.data(:,find(traj_data.colheaders=="Gyro Values"));
%}
%based on the trajectory, calculate the reference velocity and
%position
ref_velocity=zeros(size(acc));
ref_position=zeros(size(acc));

for i=1:size(acc,1)-1
   ref_velocity(i+1,:)=ref_velocity(i,:)+acc(i,:)*(times(i+1)-times(i));
end

for i=1:size(acc,1)-1
   ref_position(i+1,:)=ref_position(i,:)+ref_velocity(i,:)*(times(i+1)-times(i));
end

%% Running the Simulation
num_runs=1; 

%save the raw outputs
acc_out=cell(num_runs,1);%initialize storage array
gyro_out=cell(num_runs,1);%initialize storage array

acc_data=cell(num_runs,1);%initialize storage array
vel_data=cell(num_runs,1);%
pos_data=cell(num_runs,1);%

for ii=1:num_runs
    %output of each run is Nx3 matrix 
    [accelerometer_output,gyro_output, mag_output]=IMU(acc,angVel);
    
    %save the raw outputs
    acc_out{ii}=accelerometer_output;
    gyro_out{ii}=gyro_output;
    
    %% Simulation Output Processing
    %remove bias
    accelerometer_output=accelerometer_output-offset;

    %why does it invert?
    %accelerometer_output=-1*accelerometer_output;%I don't know why if outputs as -

    %find rotation matrix to rotate accel output to global frame
    FUSE=ahrsfilter('SampleRate',fs,'OrientationFormat','Rotation matrix');
    %need to put in noise characteristics, but I don't know them

    Rat_Mats=FUSE(accelerometer_output,gyro_output,mag_output);
    %integrate to find position, need to factor in magnetometer

    acceleration=zeros(size(accelerometer_output));


    for i=1:size(accelerometer_output,1)%for each row in the output, rotate
        dummy=(Rat_Mats(:,:,i)')*accelerometer_output(i,:)';%multiply by rotation matrix
        acceleration(i,:)=dummy';%store
    end

    %remove gravity, might need to check the dimension
    acceleration(:,3)=acceleration(:,3)-9.81;


    %integrate using Euler
    velocity=zeros(size(acceleration));
    position=zeros(size(acceleration));

    for i=1:size(acceleration,1)-1
        try 
            velocity(i+1,:)=velocity(i,:)+acceleration(i,:).*(times(i+1)-times(i));
        catch 
            keyboard
        end
    end

    for i=1:size(acceleration,1)-1
        position(i+1,:)=position(i,:)+velocity(i,:).*(times(i+1)-times(i));
    end
    
    %save global outputs
    acc_data{ii}=acceleration;
    vel_data{ii}=velocity;
    pos_data{ii}=position;
    
end
%% Determining error
%define storage arrays
error_comp=zeros(num_runs,3); %error in each dimension
error_tot=zeros(num_runs,1); %total position error

for ii=1:num_runs
   %pull position
   position=pos_data{ii};
   %find the error 
   error=position(end,:)-ref_position(end,:);
   %store error comp
   error_comp(ii,:)=error;
   
   error_tot(ii)=norm(error);%find the total error distance
end

%some interesting stats:
max_err=max(error_tot);
avg_err=mean(error_tot);
std_err=std(error_tot);
%% plotting
%error distribution
%{
figure
title('Error Distribution')
histogram(error_tot);
%}

%accelerometer
figure
hold on
%plot(times',acc(:,1))
plot(times',-1*accelerometer_output(:,1))
title('X Axis of Accelerometer')

figure
hold on
%plot(times',acc(:,2))
plot(times',-1*accelerometer_output(:,2))
title('Y Axis of Accelerometer')

figure
hold on
%plot(times',acc(:,3))
plot(times',-1*accelerometer_output(:,3))
title('Z Axis of Accelerometer')

%gyro
figure
hold on
%plot(times',angVel(:,1))
plot(times',gyro_output(:,1))
title('X Axis of Gyro')

figure
hold on
%plot(times',angVel(:,2))
plot(times',gyro_output(:,2))
title('Y Axis of Gyro')

figure
hold on
%plot(times',angVel(:,3))
plot(times',gyro_output(:,3))
title('Z Axis of Gyro')
