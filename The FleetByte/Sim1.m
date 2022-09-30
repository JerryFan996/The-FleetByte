%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CSC C85 - Fundamentals of Robotics and Automated Systems - UTSC
%
% Simulate 1 second of data and return (noisy) sensor readinds to be
% used by the FleetByte to estimate actual state variable values.
%
% The function actually has 2 modes of operation. If called with empty ([]) input arguments,
% it simulates 1 second of data and returns the sensor values to FleetByte.m so they can be
% processed. 
%
% If arguments are passed (with estimates of position, heartrate, direction, and velocity),
% it plots these against the real values of each of these variables - so we can evaluate how
% well the FleetByte code is doing at estimating the relevant quantities.
%
% You can see how this script works. But YOU ARE NOT ALLOWED to extract
% actual information from it other than the sensor readings returned to you.
%
% function [MPS,HRS,Rg]=Sim1(map_idx,xyz,hr,di,vel)
%
%  map_idx - index (1, or 2), of the map to be used.
%  xyz - a vector with 3 values corresponding to position.
%  hr - real valued input corresponding to estimated heart rate
%  di - a vector with 2 entries with the estimated motion direction (no vertical component)
%  vel - Estimated motion velocity, i.e. how fast the jogger is running. Note that a
%        displacement of 1 pixel corresponds to 1m. Your velocity should be in Km/h
% 
%  In practice, the code in FleetByte.m already calls this function when needed and with
%  the required input parameters, so all you have to do is estimate the values for them
%  from the sensor readings.
%
% Returns:
%
% MPS -> 3D position (x,y,z) on the map, noisy as per expected MPS location error, 1 per second
% HRS -> Image array containing the signal captured by the hear rate sensor on the FleetByte over the past
%        10 seconds. Somehow you have to convert this into a heartdate estimate
%        The hear rate sensor's signal is read at 120 Hz (120 times per second) so the entire array is
%        1200 entries long.
% Rg -> Rate gyro readings - tells you the rotation amount (not absolute angle!) over the last 1 second. 
%
% All of the returned values are noisy, so you have to do some work to recover an estimate close to the
% truth.
%
% Starter code: F. Estrada, Sep. 2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [MPS,HRSt,Rg]=Sim1(map_idx, xyz, hr, di, vel)

pkg load image;     % COMMENT THIS OUT IF USING MATLAB - this is only needed in Octave

% Persistent variables are equivalent to C static variables. They are local, but keep their value
% between function calls. Rather useful for simulations :)
persistent XYZ=[256 256 .5];
persistent x_old=XYZ(1);
persistent y_old=XYZ(2);
persistent d=[1 0]';
persistent HR=72;
persistent spd=10;
persistent frame=0;
persistent map;
persistent HRS=zeros(1200,1);
persistent HRnoise=.15;
persistent Rgnoise=pi/22;
persistent beatidx=1;
persistent beat1=[];
persistent hist_spd=[];
persistent hist_HR=[];
persistent hist_dr=[];
persistent hist_XYZ=[];
persistent hist_vel=[];
persistent hist_hr=[];
persistent hist_di=[];
persistent hist_xyz=[];
persistent hist_dh=[];
persistent gbl=[0.06136 	0.24477 	0.38774 	0.24477 	0.06136];

if (exist('xyz'))
    hist_vel(end+1)=vel;
    hist_hr(end+1)=hr;
    hist_di(end+1,:)=di/norm(di);
    hist_xyz(end+1,:)=xyz;
    
    if (size(hist_vel,2)~=size(hist_spd,2)|size(hist_hr,2)~=size(hist_HR,2)|size(hist_di,2)~=size(hist_dr,2)|size(hist_xyz,2)~=size(hist_XYZ,2))
        fprintf(2,'Requested plot but historic data and estimated data have different length - can not plot.\n');
        return;
    end;

    figure(1);clf;plot(hist_spd,'*-','linewidth',1.5,'color',[0 0 1]);hold on;
    plot(hist_vel,'+-','linewidth',1.5,'color',[1 0 0]);
    title('Velocity (blue) vs. estimated velocity (red)','fontsize',14);
    xlabel('Time','fontsize',14);
    ylabel('Velocity (Km/h)','fontsize',14);
    grid on;drawnow;

    figure(2);clf;plot(hist_HR,'*-','linewidth',1.5,'color',[0 0 1]);hold on;
    plot(hist_hr,'+-','linewidth',1.5,'color',[1 0 0]);
    title('Heart rate (blue) vs. estimated heart rate (red)','fontsize',14);
    xlabel('Time','fontsize',14);
    ylabel('Heart Rate (bpm)','fontsize',14);
    grid on;drawnow;

    figure(3);clf;plot(atan2(hist_dr(:,2),hist_dr(:,1)),'*-','linewidth',1.5,'color',[0 0 1]);hold on;
    plot(atan2(hist_di(:,2),hist_di(:,1)),'+-','linewidth',1.5,'color',[1 0 0]);
    title('Running direction angle (blue) vs. estimated running direction angle (red)','fontsize',14);
    xlabel('Time','fontsize',14);
    ylabel('Direction Angle (radians)','fontsize',14);
    grid on;drawnow;

    figure(4);clf;imagesc(map);axis image;colormap(gray);hold on;
    plot(hist_XYZ(:,1),hist_XYZ(:,2),'.','markersize',14,'color',[0 0 1]);
    plot(hist_xyz(:,1),hist_xyz(:,2),'.','markersize',14,'color',[1 0 0]);
    title('Actual runner position (blue) vs. estimated runner position (red)','fontsize',14);
    drawnow;
    
    fprintf(2,'_______________________________________________________\n');
    fprintf(2,'Frame: %d statistics\n',frame);
    fprintf(2,'Velocity estimates:\n');
    fprintf(2,'  Max error: %f Km/h\n',max(abs(hist_vel-hist_spd)));
    fprintf(2,'  RMS: %f\n',sqrt(mean((hist_vel-hist_spd).^2)));
    fprintf(2,'Heart Rate estimates:\n');
    fprintf(2,'  Max error: %f bpm\n',max(abs(hist_HR-hist_hr)));
    fprintf(2,'  RMS: %f\n',sqrt(mean((hist_HR-hist_hr).^2)));
    fprintf(2,'Running direction estimates:\n');
    dps=sum(hist_di.*hist_dr,2);
    angs=acos(dps);
    fprintf(2,'  Max error: %f radians\n',max(angs));
    fprintf(2,'  RMS: %f\n',sqrt(mean(angs.^2)));
    fprintf(2,'Position estimates:\n');
    dd=sum((hist_XYZ-hist_xyz).^2,2).^.5;
    fprintf(2,'  Max error: %f meters\n',max(dd));
    fprintf(2,'  RMS: %f\n',sqrt(mean(dd.^2)));
    return;    
end;

if (frame==0)
    
 % Map is 512x512 - arbitrarily we will make this equal to 512 x 512 m, so each pixel is 1 meter in size.
 % at 10Km/h, each second, the jogger would move 2.77m. So 2.77 pixels per second. The simulation
 % keeps track of position and inclination (based on direction).
 if (map_idx==1)
   map=double(imread('elevation1_crop.jpg'));
 else
   map=double(imread('elevation2_crop.jpg'));
 end;
 
 map=mean(map,3);
 map=map-min(map(:));
 map=map/max(map(:));
 map=imfilter(map,gbl,'replicate');
 map=imfilter(map,gbl','replicate');

 beat1=interp1([0 .25 .5 .75 1],[0 .75 .25 .15 0],[0:1/60:1],'cubic');
 beat1=beat1(1:60);
 beatidx=1;
 beat_inc=HR/60;
 XYZ(3)=map(256,256);

 % Generate initial HRS data
 for i=1:1200
  beatidx=beatidx+beat_inc;
  if (beatidx>61) beatidx=beatidx-60; end;
  HRS(i)=beat1((floor(beatidx)))+(HRnoise*(rand-.5));
 end;
end;

% Update state variables: XYZ, d, HR, spd, stm, and HRS
delta=spd*1000/3600;    % Distance increase in pixels, per second travelled

x_new=XYZ(end,1)+(d(1)*delta);
y_new=XYZ(end,2)+(d(2)*delta);

if (x_new<1|y_new<1|x_new>size(map,2)|y_new>size(map,1))
    fprintf(2,'The jogger left the map!!\n');
    MPS=[-1 -1 -1];
    Acc=zeros(60,1);
    HRS=-1;
    Rg=0;
    return;
end;

% Determine inclination
x1=floor(x_new);
y1=floor(y_new);
x2=ceil(x_new);
y2=ceil(y_new);

dh=map(y2,x2)-map(y1,x1);

% From delta, update velocity, HR, and stm

% Velocity update - going uphill, reduce speed, going downhill, increase speed
hist_dh(end+1)=dh*2.5;
%% Processing to figure out if we're accelerating or slowing down
if (frame>5)
 smooth_dh=imfilter(imfilter(hist_dh,gbl,'replicate'),gbl,'replicate');
 spd=spd+smooth_dh(end-2);
end;
spd=min(15,spd);
spd=max(5,spd);

% Randomized speed change

% HR
if (frame>6)
 spm=mean(hist_spd(end-5:end));
 tr=70+110*sqrt((spm-5)/10);
 tr=min(180,tr);
 tr=max(60,tr);
else
 tr=70;
end;

if (HR<tr) 
    HR=HR+min(5,sqrt(tr-HR));
else
    HR=HR-min(5,sqrt(HR-tr));
end;

beat_inc=HR/60;

% Generate HRT data for the last second
HRS(1:1200-120)=HRS(121:end);
npr=rand;
for i=1200-120:1200
    beatidx=beatidx+beat_inc;
    if (beatidx>61) beatidx=beatidx-60; end;
    HRS(i)=beat1((floor(beatidx)))+(HRnoise*(rand-.5));
    if (npr<.05)
     HRS(i)=HRS(i)*.5*rand;
    else if (npr<.1)
        HRS(i)=HRS(i)*.25*rand;
      end;
    end;
end;

% Position data - Gaussian noise added at the specified amount
XYZ(:)=[x_new y_new map(y2,x2)];
MPS=[0 0 0];
MPS(1)=x_new+(1.5*randn(1));
MPS(2)=y_new+(1.5*randn(1));
MPS(3)=XYZ(3)+(1.5*randn(1));

% Update direction and rate gyro
dr=pi/8;
dang=dr*randn(1);
R=[cos(dang) -sin(dang)
   sin(dang) cos(dang)];
d=R*d;

Rg=dr+(Rgnoise*randn(1));

hist_spd(end+1)=spd;
hist_HR(end+1)=HR;
hist_dr(end+1,:)=d;
hist_XYZ(end+1,:)=XYZ;

frame=frame+1;
HRSt=HRS;

