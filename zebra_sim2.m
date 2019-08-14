clear
%% Parameters
% Zebra params
zebraStart=[10 0];   % centre of zebra start coords
zebraLength=2.5;
zebraBack=[zebraStart(1)-zebraLength/2 0];
zebraFront=[zebraStart(1)+zebraLength/2 0];
zebraSpeed=10.9;
angle_z=1;

num_its=1;
for aim=1
    
    % Lion params
    lionStart(1)=10;
    for a=1:num_its
            lionStart(2)=-5+(a*-2);
%            lionStart(2)=-400;
        for b=1:num_its
              lionSpeed=12+b*.025;
%             lionSpeed=15;
            
            % Lion aims for front or back of zebra (front=1; back=2);
            distance=pdist([zebraBack(1) zebraBack(2); lionStart(1) lionStart(2)],'Euclidean');
            i=1;
            
            while distance>=4
                
                if i==1
                    zeb_pos_f(i,1:2)=zebraFront;
                    zeb_pos_b(i,1:2)=zebraBack;
                    zeb_pos_c(i,1:2)=zebraStart;
                    lion_pos(i,1:2)=lionStart;
                else
                    zeb_pos_f(i,1:2)=[zeb_pos_f(i-1,1)+(zebraSpeed*sin(angle_z)),zeb_pos_f(i-1,2)+(zebraSpeed*cos(angle_z))];
                    zeb_pos_b(i,1:2)=[zeb_pos_b(i-1,1)+(zebraSpeed*sin(angle_z)),zeb_pos_b(i-1,2)+(zebraSpeed*cos(angle_z))];
                    zeb_pos_c(i,1:2)=[zeb_pos_c(i-1,1)+(zebraSpeed*sin(angle_z)),zeb_pos_c(i-1,2)+(zebraSpeed*cos(angle_z))];
                    
                    if lion_pos(i-1,2)>0
                        lion_pos(i-1,2)=0;
                    end
                    
                    if aim==1 % Lion aims at front
                        x=zeb_pos_f(i-1,1)-lion_pos(i-1,1);
                        y=zeb_pos_f(i-1,2)-lion_pos(i-1,2);
                        angle1=atan(x/y);
                        
                        if y<0
                            angle1=angle1-pi;
                        end
                        
                    elseif aim==2 % Lion aims at back
                        x=zeb_pos_b(i-1,1)-lion_pos(i-1,1);
                        y=zeb_pos_b(i-1,2)-lion_pos(i-1,2);
                        angle1=atan(x/y);
                        
                        if y<0
                            angle1=angle1-pi;
                        end
                    end
                    
                    lion_pos(i,1)=lion_pos(i-1,1)+(lionSpeed*sin(angle1));
                    lion_pos(i,2)=lion_pos(i-1,2)+(lionSpeed*cos(angle1));
                    angle2=rad2deg(angle1);
                    
                end
                
                distance=pdist([zeb_pos_b(i,1) zeb_pos_b(i,2);lion_pos(i,1) lion_pos(i,2)],'Euclidean');
                
                if i>2
                    if lion_pos(i,1)>=zeb_pos_b(i,1) && lion_pos(i,2)>=zeb_pos_b(i,2)
                        distance=0;
                    end
                end
                
                figure(1);
                if aim==1
                    subplot(2,1,1); plot(zeb_pos_b(i,1),zeb_pos_b(i,2),'ro',lion_pos(i,1),lion_pos(i,2),'bo'); hold on
%                     ylim([-205 2]);
%                     xlim([-5 500])
                    title(sprintf('Evolving Lion and Zebra Positions: t=%d',i));
                
                    subplot(2,1,2); plot(i,distance,'bx'); hold on
%                     xlim([0 50])
%                     ylim([0 10])
                    title 'Distance between Lion and Zebra'
                elseif aim==2
                    subplot(2,1,1); plot(zeb_pos_c(i,1),zeb_pos_c(i,2),'ro',lion_pos(i,1),lion_pos(i,2),'bo'); hold on
                    ylim([-205 2]);
                    xlim([-5 500])
                    title(sprintf('Evolving Lion and Zebra Positions: t=%d',i));
                
                    subplot(2,1,2); plot(i,distance,'gx'); hold on
                    xlim([0 50])
                    ylim([0 10])
                    title 'Distance between Lion and Zebra'
                 end
                  
                distance2(a,b)=(i*lionSpeed)+distance;
                            i=i+1;
                
            end

%                  waitforbuttonpress
        end
    end

   
%% PLOTTING
xax=12+[1:num_its]*.025; %
yax=5+[1:num_its].*2;
figure(1);
% if aim==1
% subplot(2,1,1); surf(xax,yax,distance2);
% colormap 'hot'
% elseif aim==2
% subplot(2,1,2); surf(xax,yax,distance2);
% colormap 'hot'
% end


figure(aim)
colormap('hot')
imagesc(xax,yax,distance2)
colorbar
xlabel 'Relative speed: lion/zebra'
ylabel 'Starting distance'
title 'Heatmap indicating chase length to catch zebra (s)'
if aim==1
    d1=distance2;
else
    d2=distance2;
end
end

    
    d3=d2-d1;
    figure(5)
    colormap('hot')
    imagesc(xax,yax,d3)
    colorbar
    xlabel 'Relative speed: lion/zebra'
    ylabel 'Starting distance'
    title 'Heatmap indicating difference in chase time between lion aiming for the back relative to the front'