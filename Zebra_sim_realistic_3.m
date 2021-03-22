clear
%% Parameters
%% Zebra parameters
zebraStart_x=0;         % Centre of zebra in x at start
zebraStart_y=0;         % Centre of zebra in y at start (Zebra has no width)
zebraLength=2.42;       % Length of zebra
zebra_max_speed=17;     % Zebra max speed (m/s)
zebra_acc=5;            % Zebra acceleration (m/s^2)

% zebra x and y coords
zebraBack_y=zebraStart_y;
zebraBack_x=zebraStart_x-zebraLength/2;
zebraFront_y=zebraStart_y;
zebraFront_x=zebraStart_x+zebraLength/2;

%% Lion Parameters
lion_start_speed=14;    % Lion max speed (m/s)
lion_acc=9.5;           % Lion acceleration (m/s^2)
lionStart_x=0;          % Lion start coord x
lionStart_y_begin=10;   % Lion start coord y

%% simulation parameters
time_increment=1000;    % (ms)
cutoff=22;              % cutoff for timeout (s)
num_increments_speed=7; % number of increments of speed increase (predator)
num_increments_dist=10; % number of increments of distance increase (start position of predator in y)
distance_increment=5;   % distance increment (m)
speed_increment=1;      % speed increment (m/s)
timeout=0;              % set timeout to 0 
i=1;                    % set iteration to 1
%% Simulate
% run aiming at front and aiming at back
for aim=1:2
    for a=1:num_increments_dist                                             % run series of start distances
        for b=1:num_increments_speed                                        % run series of lion speeds
            lionStart_y=-lionStart_y_begin+((a-1)*-distance_increment);
            lion_max_speed=lion_start_speed+((b-1)*speed_increment);
            timeout=0;
            i=1;
            while timeout==0
                if i==1
                    distance_frontaim(i)=pdist([zebraBack_x zebraBack_y;lionStart_x lionStart_y],'Euclidean');
                    distance_backaim(i)=pdist([zebraBack_x zebraBack_y;lionStart_x lionStart_y],'Euclidean');
                    zeb_pos_f(i,1:2)=[zebraFront_x,zebraFront_y];
                    zeb_pos_b(i,1:2)=[zebraBack_x,zebraBack_y];
                    zeb_pos_c(i,1:2)=[zebraStart_x,zebraStart_y];
                    if aim==1
                        lion_pos_frontaim(i,1:2)=[lionStart_x,lionStart_y];
                    elseif aim==2
                        lion_pos_backaim(i,1:2)=[lionStart_x,lionStart_y];
                    end
                    
                    zebraSpeed(i)=0;
                    lionSpeed(i)=0;
                    
                else
                    % determine prey speeds (based on acceleration)
                    zebraSpeed(i)=zebraSpeed(i-1)+(zebra_acc/(time_increment^2)); % accelerate zebra to max speed
                    if zebraSpeed(i)>zebra_max_speed/time_increment
                        zebraSpeed(i)=zebra_max_speed/time_increment;
                    end
                    % move zebra front (f), centre (c) and back (b)
                    % locations
                    zeb_pos_f(i,1:2)=[zeb_pos_f(i-1,1)+(zebraSpeed(i)),zeb_pos_f(i-1,2)]; % +(zebraSpeed(i)*cos(angle_z))
                    zeb_pos_b(i,1:2)=[zeb_pos_b(i-1,1)+(zebraSpeed(i)),zeb_pos_b(i-1,2)]; % +(zebraSpeed(i)*cos(angle_z))
                    zeb_pos_c(i,1:2)=[zeb_pos_c(i-1,1)+(zebraSpeed(i)),zeb_pos_c(i-1,2)]; % +(zebraSpeed(i)*cos(angle_z))
                    
                    % aim 1 = front, aim 2 = back
                    if aim==1
                        % calculate predator heading angle (front sim)
                        x_f=zeb_pos_f(i-1,1)-lion_pos_frontaim(i-1,1);
                        y_f=zeb_pos_f(i-1,2)-lion_pos_frontaim(i-1,2);
                        angle1_f=atan(x_f/y_f);
                        if y_f<0
                            angle1_f=angle1_f-pi;
                        end
                    elseif aim==2
                        % calculate predator heading angle (back sim)
                        x_b=zeb_pos_b(i-1,1)-lion_pos_backaim(i-1,1);
                        y_b=zeb_pos_b(i-1,2)-lion_pos_backaim(i-1,2);
                        angle1_b=atan(x_b/y_b);
                        if y_b<0
                            angle1_b=angle1_b-pi;
                        end
                    end
                    % determine predator speed (based on acceleration and
                    % time)
                    lionSpeed(i)=lionSpeed(i-1)+(lion_acc/(time_increment^2));
                    if lionSpeed(i)>lion_max_speed/time_increment
                        lionSpeed(i)=lion_max_speed/time_increment;
                    end
                    
                    % update predator position
                    if aim==1
                        lion_pos_frontaim(i,1)=lion_pos_frontaim(i-1,1)+(lionSpeed(i)*sin(angle1_f));
                        lion_pos_frontaim(i,2)=lion_pos_frontaim(i-1,2)+(lionSpeed(i)*cos(angle1_f));
                        angle2_f=rad2deg(angle1_f);
                    elseif aim==2
                        lion_pos_backaim(i,1)=lion_pos_backaim(i-1,1)+(lionSpeed(i)*sin(angle1_b));
                        lion_pos_backaim(i,2)=lion_pos_backaim(i-1,2)+(lionSpeed(i)*cos(angle1_b));
                        angle2_f=rad2deg(angle1_b);
                        
                    end
                end
                
                % calculate Euclidean distance between predator and prey
                if aim==1
                    distance_frontaim(i,a,b)=pdist([zeb_pos_c(i,1) zeb_pos_c(i,2);lion_pos_frontaim(i,1) lion_pos_frontaim(i,2)],'Euclidean');
                elseif aim==2
                    distance_backaim(i,a,b)=pdist([zeb_pos_c(i,1) zeb_pos_c(i,2);lion_pos_backaim(i,1) lion_pos_backaim(i,2)],'Euclidean');
                end
                
                timer=i;    % timer function
                
                % to determine whether to time out or whether to continue
                % based on x and y distance b/w lion and zebra
                if i>500 % over 500ms
                    if aim==1
                        if timer<cutoff*time_increment
                            if zeb_pos_b(i,1)<=lion_pos_frontaim(i,1) && abs(zeb_pos_b(i,2)-lion_pos_frontaim(i,2))<1 % lion x>zebra x and lion y>zebra y zeb_pos_b(i,2)<=lion_pos_frontaim(i,2) && 
                                time(a,b,aim)=i;
                                timeout=1;
                            end
                        else
                            time(a,b,aim)=i;
                            timeout=1;
                        end
                    elseif aim==2
                        if timer<cutoff*time_increment
                            if zeb_pos_b(i,1)<=lion_pos_backaim(i,1) && abs(zeb_pos_b(i,2)-lion_pos_backaim(i,2))<1% lion x>zebra x and lion y>zebra y  && 
                                time(a,b,aim)=i;
                                timeout=1;
                            end
                        else
                            time(a,b,aim)=i;
                            timeout=1;
                        end
                    end
                end
                i=i+1;
                
                
            end
        end
    end
end

%% PLOTTING
xax=lion_start_speed+[(1:num_increments_speed)-1].*speed_increment; %
yax=lionStart_y_begin+[(1:num_increments_dist)-1].*distance_increment;

%% calculate front/back distances and times
for a=1:num_increments_dist
    for b=1:num_increments_speed
        time_front(a,b)=time(a,b,1)/1000;
        distance_front(a,b)=time(a,b,1);
        
        r=find(distance_backaim(:,a,b)==0);
        if isempty(r)
            distance_back(a,b)=nan;                                     %distance_backaim(end,a,b);
        else
            distance_back(a,b)= nan;                                    %distance_backaim(r(1)-1,a,b);
        end
        time_back(a,b)=time(a,b,2)/1000;
    end
end
%% front/back heatmaps
figure(1)
subplot(1,4,2); hold on
ax = gca;
ax.Position=[.425,.125,.165,.33];
colormap('hot')
imagesc(xax,yax,time_front(:,:))
colorbar
xlabel('Predator Max Speed (m/s)','FontSize',12)
ylabel('Starting Distance (m)','FontSize',12)
title('Time-to-capture (s): Front Aim','Fontsize',13)
xlim([13.5 20.5]); 
ylim([7.5 57.5]);
hold off
subplot(1,4,3); hold on
ax = gca;
ax.Position=[.425,.6,.165,.35];
colormap('hot')
imagesc(xax,yax,time_back(:,:))
colorbar
xlabel('Predator Max Speed (m/s)','FontSize',12)
ylabel('Starting Distance (m)','FontSize',12)
title('Time-to-capture (s): Rear Aim','FontSize',13)
xlim([13.5 20.5]); 
ylim([7.5 57.5]);
hold off

%% differences heatmap
d3=abs(time_front(:,:)-time_back(:,:));
for r=1:num_increments_dist
    for c=1:num_increments_speed
        if time_back(r,c)==cutoff & time_front(r,c)<cutoff
            d3(r,c)=1.5
        end
    end
end
% [m,i]=maxk(d3(:),2);
% d3(i(1))=m(2);
d3=reshape(d3,[10 7]);

subplot(1,4,4); hold on
ax = gca;
ax.Position=[.65,.125,.35,.825];
colormap('hot')
imagesc(xax,yax,d3)
colorbar
xlabel('Predator Max Speed (m/s)','FontSize',12)
ylabel('Starting Distance (m)','FontSize',12)
title('Difference in Time-to-capture (s)','FontSize',13)
xlim([13.5 20.5]); 
ylim([7.5 57.5]);
hold off
