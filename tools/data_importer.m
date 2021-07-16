clear all %#ok<CLALL>
clc

simultanius_log_rate = 100; % hz

wd = dir;
wd = wd(~[wd.isdir]);

for file = 1:length(wd)
    if contains(wd(file).name,"DFTI_") && ~contains(wd(file).name,".mat")

        file_name = erase(convertCharsToStrings(wd(file).name),".csv");
        
        table = readmatrix(file_name+".csv",'OutputType','string');

        ID   = str2double(table(:,1));
        type = table(:,2);
        time = str2double(table(:,3));
        data = str2double(table(:,4));

        time = time - min(time);

        types = unique(type);

        for i = 1:length(types)
            results.(types(i)).raw.time = time(strcmp(types(i),type)); % sort
            results.(types(i)).raw.hz = 1./(results.(types(i)).raw.time(2:end)-results.(types(i)).raw.time(1:end-1));
            results.(types(i)).raw.data = data(strcmp(types(i),type));

            results.(types(i)).zoh.time = (1:1/simultanius_log_rate:max(results.(types(i)).raw.time)).'; 
            results.(types(i)).zoh.hz = ones(size(results.(types(i)).zoh.time))*simultanius_log_rate;
            nni = knnsearch(results.(types(i)).raw.time,results.(types(i)).zoh.time);
            results.(types(i)).zoh.data = results.(types(i)).raw.data(nni);
            results.(types(i)).zoh.data((results.(types(i)).zoh.time-results.(types(i)).raw.time(nni))<0) = results.(types(i)).raw.data(nni((results.(types(i)).zoh.time-results.(types(i)).raw.time(nni(:,1)))<0)-1);

            results.(types(i)).interpolate.time = (1:1/simultanius_log_rate:max(results.(types(i)).raw.time)).';
            results.(types(i)).interpolate.hz = ones(size(results.(types(i)).interpolate.time))*simultanius_log_rate;
            results.(types(i)).interpolate.data = interp1(results.(types(i)).raw.time,results.(types(i)).raw.data,results.(types(i)).interpolate.time);


        %     plot(results.(types(i)).rawtime,results.(types(i)).hz)
        %     disp(types(i));
        %     disp(length(results.(types(i)).rawdata)/(max(results.(types(i)).rawtime)-min(results.(types(i)).rawtime)));
        %     input("Press ENTER to continue:")
        end

        euler_angles = ["psi";"theta";"phi"];

        raw_index = min([length(results.("q0").raw.data),length(results.("q1").raw.data),length(results.("q2").raw.data),length(results.("q3").raw.data)]);
        zoh_index = min([length(results.("q0").zoh.data),length(results.("q1").zoh.data),length(results.("q2").zoh.data),length(results.("q3").zoh.data)]);
        interp_index = min([length(results.("q0").interpolate.data),length(results.("q1").interpolate.data),length(results.("q2").interpolate.data),length(results.("q3").interpolate.data)]);

        raw_euler = zeros(raw_index,3);
        for i = 1:raw_index
            [~,roll,pitch,yaw] = enu_to_ned(quaternion([results.("q0").raw.data(i),results.("q1").raw.data(i),results.("q2").raw.data(i),results.("q3").raw.data(i)]));
            raw_euler(i,:) = [yaw,pitch,roll];
        end
        zoh_euler = zeros(zoh_index,3);
        for i = 1:zoh_index
            [~,roll,pitch,yaw] = enu_to_ned(quaternion([results.("q0").zoh.data(i),results.("q1").zoh.data(i),results.("q2").zoh.data(i),results.("q3").zoh.data(i)]));
            zoh_euler(i,:) = [yaw,pitch,roll];
        end
        interp_euler = zeros(interp_index,3);
        for i = 1:interp_index
            [~,roll,pitch,yaw] = enu_to_ned(quaternion([results.("q0").interpolate.data(i),results.("q1").interpolate.data(i),results.("q2").interpolate.data(i),results.("q3").interpolate.data(i)]));
            interp_euler(i,:) = [yaw,pitch,roll];
        end
        
        for i = 1:length(euler_angles)
            results.(euler_angles(i)).raw.time = results.("q0").raw.time(1:raw_index);
            results.(euler_angles(i)).raw.hz = results.("q0").raw.hz(1:raw_index-1);
            results.(euler_angles(i)).raw.data = raw_euler(:,i);

            results.(euler_angles(i)).zoh.time = results.("q0").zoh.time(1:zoh_index);
            results.(euler_angles(i)).zoh.hz = results.("q0").zoh.hz(1:zoh_index-1);
            results.(euler_angles(i)).zoh.data = zoh_euler(:,i);

            results.(euler_angles(i)).interpolate.time = results.("q0").interpolate.time(1:interp_index);
            results.(euler_angles(i)).interpolate.hz = results.("q0").interpolate.hz(1:interp_index-1);
            results.(euler_angles(i)).interpolate.data = interp_euler(:,i);
        end
%         figure(1)
%         clf
%         hold on
%         plot(results.("phi").raw.time,results.("phi").raw.data*180/pi,"-r")
%         plot(results.("theta").raw.time,results.("theta").raw.data*180/pi,"--r")
%         plot(results.("psi").raw.time,(results.("psi").raw.data-results.("psi").raw.data(1))*180/pi,":r")
%         plot(results.("phi").zoh.time,results.("phi").zoh.data*180/pi,"-b")
%         plot(results.("theta").zoh.time,results.("theta").zoh.data*180/pi,"--b")
%         plot(results.("psi").zoh.time,(results.("psi").zoh.data-results.("psi").zoh.data(1))*180/pi,":b")
%         plot(results.("phi").interpolate.time,results.("phi").interpolate.data*180/pi,"-g")
%         plot(results.("theta").interpolate.time,results.("theta").interpolate.data*180/pi,"--g")
%         plot(results.("psi").interpolate.time,(results.("psi").interpolate.data-results.("psi").interpolate.data(1))*180/pi,":g")

        % hold on
        % plot(results.("u").raw.time,results.("u").raw.data)

        types = [types;euler_angles]; %#ok<AGROW>

        save(file_name+".mat","results","types");
        
        clear file_name table ID type time data types results nni euler_angles raw_index zoh_index raw_euler zoh_euler interp_euler
    end
end

function [q_ned,roll,pitch,yaw] = enu_to_ned(q_enu)

    % Example from today
    % q1 = quaternion([pi/2,0,0],'euler','ZYX','frame');

    % NED -> ENU -> FLU -> FRD
    % Inertial -> ROS Inertial -> ROS Body (MAVROS Calls it Baselink) -> Body
    q_ned = quaternion([pi/2,0,pi],'euler','ZYX','frame')*q_enu*quaternion([0,0,pi],'euler','ZYX','frame');
    % Start on the left from your referance orientation
    % End on the right in your object orientation.

    euler_angles = euler(q_ned,'ZYX','frame');

    yaw = euler_angles(1);
    pitch = euler_angles(2);
    roll = euler_angles(3);
end
