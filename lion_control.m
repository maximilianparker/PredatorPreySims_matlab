clear

t=1:50;
% zeb_y=[1:1:100]*1;
% zeb_x=1:1:100;
% plot(zeb_x,zeb_y)

lion_start=[0,-100];
zeb_start=[1,1];

lion_gain=0.1;
zeb_gain=.01;
T=1;
ref=0;
damp=0;
dt=1/60;
counter=0;
movement=1;

lion_x(1:2)=lion_start(1);
lion_y(1:2)=lion_start(2);
zeb_x(1:2)=zeb_start(1);
zeb_y(1:2)=zeb_start(2);
for i=3:length(t)
   zeb_x(i)=zeb_x(i-1)+1;
   zeb_y(i)=zeb_y(i-1)+1.5;
   
   lion_x(i)=lion_x(i-1)+(lion_gain*(zeb_x(i-T)-lion_x(i-T)))-lion_x(i-1)*damp;
   lion_y(i)=lion_y(i-1)+(lion_gain*(zeb_y(i-T)-lion_y(i-T)))-lion_y(i-1)*damp;
   

%    zeb_x(i)=zeb_x(i-1)+zeb_gain*(lion_x(i-T)+zeb_x(i-T))-zeb_x(i-1)*damp;
%    zeb_y(i)=zeb_y(i-1)+zeb_gain*(lion_y(i-T)+zeb_y(i-T))-zeb_y(i-1)*damp;

xx(i)=zeb_x(i)-lion_x(i)
yy(i)=zeb_y(i)-lion_y(i)
overall_dist(i)=sqrt(xx(i)^2+yy(i)^2)

  
figure(1); plot(zeb_x,zeb_y,'o',lion_x,lion_y,'x')
xlim([-150 150])
ylim([-150 150])
end

    