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
    %This section takes the error parameters of the gyroscope and
    %accelerometer. I pulled what I could off the
    %datasheets but there likely needs to be more defined values from the
    %allen tests
%2. Defining the Model
    %This section is pretty basic, it just feeds the measurement parameters
    %into the model to create the model object.
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

%To do: Figure out how to rotated from NED back to OG
    
%% Defining Measurement Parameters
clear
close all

fs=100;%sampling frequency, in Hz
%gyroscope
noise_den=0.005*(pi/180); %value given in dps/sqrt(Hz) so convert to rad
noise_var=(noise_den*sqrt(fs))^2; %square std dev, from 
%https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs
instab=10*(pi/180)*(1/3600);%value given in deg/h need rad/s
g_parameters=gyroparams('NoiseDensity', noise_den,...
    'BiasInstability', instab);

%accelerometer
offset=125*9.81e-3;%given in mg
white_noise=75*9.81e-6;%given in micro g, correct units for the NoiseDensity
a_noise_var=(white_noise*sqrt(fs))^2;
Temp_noise=0.2*9.81e-3;%given in mg
a_instab_x=7.2*9.81e-6;%given in micro mg
a_instab_y=6.5*9.81e-6;%given in micro mg
a_instab_z=9.7*9.81e-6;%given in micro mg

a_parameters=accelparams('ConstantBias',offset,'NoiseDensity',white_noise,...
    'TemperatureBias',Temp_noise,'BiasInstability',[a_instab_x,a_instab_y,a_instab_z]);

%% defining the model

FUSE=imufilter('SampleRate',fs,'AccelerometerNoise',a_noise_var,'GyroscopeNoise',noise_var,'OrientationFormat','Rotation matrix');
%FUSE=imufilter('SampleRate',fs,'AccelerometerNoise',a_noise_var,'GyroscopeNoise',noise_var);
%need to put in noise characteristics, but I don't know them

IMU=imuSensor('accel-gyro','Gyroscope',g_parameters,'Accelerometer'...
    ,a_parameters);

%% defining the trajectory
runtime=120;%runtime in s
N=(runtime*fs);%number of readings

%define acceleration, 1 m/s^2 in x direction
acc=1*ones(N,3);
acc(:,2:3)=0;%remove y and z values


%define angular velocity
rpm=60;%rotating once every second
total_rad=2*pi*rpm*runtime/60;%find total distance traversed
angVel=ones(N,3)*(total_rad/runtime);
angVel(:,1:2)=0;%want to rotate about z only , so remove 

%convert to NED frame
traj = kinematicTrajectory('SampleRate',fs);
[posNED,orientationNED,velNED,accNED,angVelNED] = traj(acc,angVel);

%define the time array
times=(0:N-1)'/fs;


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

C=cell(num_runs,1);%initialize storage array
orientation_data=cell(num_runs,1);%initialize storage array
acc_data=cell(num_runs,1);%initialize storage array
vel_data=cell(num_runs,1);%
pos_data=cell(num_runs,1);%

for ii=1:num_runs
    %output of each run is Nx3 matrix 
    %note to self, these reference inputs are in the body frame, regardless
    %of what the IMU documentation says 
    %[accelerometer_output,gyro_output]=IMU(accNED,angVelNED,orientationNED);
    [accelerometer_output,gyro_output]=IMU(accNED,angVelNED);
    %[accelerometer_output,gyro_output]=IMU(acc,angVel);
    
    %save the raw outputs
    acc_out{ii}=accelerometer_output;
    gyro_out{ii}=gyro_output;
    
    %% Simulation Output Processing

    %remove bias
    accelerometer_output=accelerometer_output-offset;
    %gyro offset is 0, thus there is nothing to remove
    
    %remove gravity, don't ask why it's removed before rotation, IMU
    %seems to output gravity as a constant in the z axis
    accelerometer_output(:,3)=accelerometer_output(:,3)-9.81;
    
    accelerometer_output(:,:)=-1*accelerometer_output(:,:);
    
    %initialize acceleration array
    acceleration=zeros(size(accelerometer_output));

    %integrate the angular velocity using Euler 
    C{1}=[1,0,0;0,1,0;0,0,1];
    for i=1:size(accelerometer_output,1)%for each row in the output, rotate
        %create matrix
        if i==1 %if using initial array 
            C_current=C{i};
        else 
            %find dt 
            dt=times(i)-times(i-1);
            %pull angular velocities, in x,y,z format
            w=gyro_output(i,:);
            %find sigma and construct the B matrix, Equation 37 in Woodman
            sigma=norm(w*dt);
%             B=[0, -w(3)*dt, w(2)*dt; ...
%                 w(3)*dt, 0,-w(1)*dt;...
%                 -w(2)*dt,w(1)*dt,0];
            % used negative angles instead of transposing in the interest
            % of time
            B=[0, w(3)*dt, -w(2)*dt; ...
                -w(3)*dt, 0,w(1)*dt;...
                w(2)*dt,-w(1)*dt,0];
            %then can find the matrix to multiply C_old by
            Update_Mat=eye(3)+(sin(sigma)/sigma)*B+((1-cos(sigma))/sigma^2)*B^2;%check if B^2 is elementwise or not
            %now find the updated C
            C_current=C{i-1}*Update_Mat;
            C{i}=C_current;
        end
        %rotate
        dummy=C_current*accelerometer_output(i,:)';
        % dummy=C_current'*accelerometer_output(i,:)';
        acceleration(i,:)=dummy';
    end
    %for some reason it insists on flipping the sign, hard correcting here
    %acceleration(:,1)=-1*acceleration(:,1);

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
% figure
% title('Error Distribution')
% histogram(error_tot);




% %accelerometer
% figure
% hold on
% %plot(times',acc(:,1))
% plot(times,accelerometer_output(:,1))
% title('X Axis of Accelerometer')
% xlabel('Time (s)')
% ylabel('Output (m/s^2)')
% 
% figure
% hold on
% %plot(times',acc(:,2))
% plot(times,accelerometer_output(:,2))
% title('Y Axis of Accelerometer')
% xlabel('Time (s)')
% ylabel('Output (m/s^2)')
% 
% figure
% hold on
% %plot(times',acc(:,3))
% plot(times,accelerometer_output(:,3))
% title('Z Axis of Accelerometer')
% xlabel('Time (s)')
% ylabel('Output (m/s^2)')
% 
% %gyro
% figure
% hold on
% %plot(times',angVel(:,1))
% plot(times,gyro_output(:,1))
% title('X Axis of Gyro')
% xlabel('Time (s)')
% ylabel('Output (rad/s)')
% 
% figure
% hold on
% %plot(times',angVel(:,2))
% plot(times,gyro_output(:,2))
% title('Y Axis of Gyro')
% xlabel('Time (s)')
% ylabel('Output (rad/s)')
% 
% figure
% hold on
% %plot(times',angVel(:,3))
% plot(times,gyro_output(:,3))
% title('Z Axis of Gyro')
% xlabel('Time (s)')
% ylabel('Output (rad/s)')


%ref
figure 
hold on
plot(times,acc(:,1), times, acc(:,2), times, acc(:,3))
title('Reference Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend('X axis','Y axis','Z axis','location','best')


% figure 
% hold on
% plot(times,angVel(:,1), times, angVel(:,2), times, angVel(:,3))
% title('Reference Angular Velocity')
% xlabel('Time (s)')
% ylabel('Angular Velocity (rad/s)')
% legend('X axis','Y axis','Z axis','location','best')

%actual
figure 
hold on
plot(times,accelerometer_output(:,1), times, accelerometer_output(:,2), times, accelerometer_output(:,3))
%plot(times,acceleration(:,1), times, acceleration(:,2), times, acceleration(:,3))
title('Measured Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend('X axis','Y axis','Z axis','location','best')

figure 
hold on
%plot(times,accelerometer_output(:,1), times, accelerometer_output(:,2), times, accelerometer_output(:,3))
plot(times,acceleration(:,1), times, acceleration(:,2), times, acceleration(:,3))
title('Rotated Acceleration')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
legend('X axis','Y axis','Z axis','location','best')

% figure 
% hold on
% plot(times,gyro_output(:,1), times, gyro_output(:,2), times, gyro_output(:,3))
% title('Measured Angular Velocity')
% xlabel('Time (s)')
% ylabel('Angular Velocity (rad/s)')
% legend('X axis','Y axis','Z axis','location','best')
% 
% figure 
% hold on
% plot(times,orientation(:,1), times, orientation(:,2), times, orientation(:,3))
% title('Measured Orientation')
% xlabel('Time (s)')
% ylabel('Orientation (rad)')
% legend('\gamma','\beta','\alpha','location','best')

% index_of_interst=zeros(size(Rat_Mats));
% for i=1:size(Rat_Mats,2)
%     index_of_interst(i)=Rat_Mats{i}(9);
% end
% index_of_interst=index_of_interst';
% figure
% plot(times,index_of_interst)
