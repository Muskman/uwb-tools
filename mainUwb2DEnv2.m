%% count number of points covered in a UWB placement
% specify size of work space [x,y] in meters
ws = [120 90];

c_off = 3; d_off = 25;
theta = atan(ws(2)/ws(1));
theta1 = atan(0.5*ws(2)/d_off);
theta2 = atan(0.5*ws(2)/(0.5*ws(1)-d_off));

% specify camera positions for narrow and wide angled cameras, each row is a camera
prime_c = [ 
           % corners
           0 0 atan(theta);
           ws(1) 0 pi-theta;
           ws(1) ws(2) -pi+theta;
           0 ws(2) -theta

           % in the middle of axis
           0 0.5*ws(2) 0; 
           0.5*ws(1) 0 pi/2; 
           ws(1) 0.5*ws(2) -pi; 
           0.5*ws(1) ws(2) -pi/2
           
           % add in between x1 axis
           0.125*ws(1) 0 pi/2;
           0.25*ws(1) 0 pi/2;
           0.375*ws(1) 0 pi/2;
           0.625*ws(1) 0 pi/2;
           0.75*ws(1) 0 pi/2;
           0.875*ws(1) 0 pi/2;

%            0.1667*ws(1) 0 pi/2;
%            0.33*ws(1) 0 pi/2;
%            0.667*ws(1) 0 pi/2;
%            0.833*ws(1) 0 pi/2;
           
           % add in between x2 axis
           0.125*ws(1) ws(2) -pi/2;
           0.25*ws(1) ws(2) -pi/2;
           0.375*ws(1) ws(2) -pi/2;
           0.625*ws(1) ws(2) -pi/2;
           0.75*ws(1) ws(2) -pi/2;
           0.875*ws(1) ws(2) -pi/2;
           
%            0.1667*ws(1) ws(2) -pi/2;
%            0.33*ws(1) ws(2) -pi/2;
%            0.667*ws(1) ws(2) -pi/2;
%            0.833*ws(1) ws(2) -pi/2;
           
           % add in between y1 axis
           0 0.1667*ws(2) 0;
           0 0.33*ws(2) 0;
           0 0.667*ws(2) 0;
           0 0.833*ws(2) 0;
           
           % add in between y2 axis
           ws(1) 0.166*ws(2) pi;
           ws(1) 0.333*ws(2) pi;
           ws(1) 0.667*ws(2) pi;
           ws(1) 0.833*ws(2) pi;
           
           % +4 projecting outwards left cluster
           ws(1)/2+c_off-d_off ws(2)/2+c_off theta1;
           ws(1)/2+c_off-d_off ws(2)/2-c_off -theta1;
           ws(1)/2-c_off-d_off ws(2)/2+c_off pi-theta2;
           ws(1)/2-c_off-d_off ws(2)/2-c_off -pi+theta2;
           
           % +4 projecting outwards
           ws(1)/2-d_off ws(2)/2+c_off pi/2;
           ws(1)/2-d_off ws(2)/2-c_off -pi/2;
           ws(1)/2+c_off-d_off ws(2)/2 0;
           ws(1)/2-c_off-d_off ws(2)/2 -pi;
           
           % +4 projecting outwards
           ws(1)/2+c_off+d_off ws(2)/2+c_off theta2;
           ws(1)/2+c_off+d_off ws(2)/2-c_off -theta2;
           ws(1)/2-c_off+d_off ws(2)/2+c_off pi-theta1;
           ws(1)/2-c_off+d_off ws(2)/2-c_off -pi+theta1;
           
           % +4 projecting outwards
           ws(1)/2+d_off ws(2)/2+c_off pi/2;
           ws(1)/2+d_off ws(2)/2-c_off -pi/2;
           ws(1)/2+c_off+d_off ws(2)/2 0;
           ws(1)/2-c_off+d_off ws(2)/2 -pi;
           
           % +2 projecting outwards
           ws(1)/2 ws(2)/2-10 -pi/2;
           ws(1)/2 ws(2)/2+10 +pi/2;
           ];
         
% resolution of points in space 
res = 2; 

ws = ws*res;
pp = 0; total_pts = prod(ws);

% sensor coverage array 
sensor_coverage = zeros(total_pts,2+size(prime_c,1));

textprogressbar('calculating coverage: ');
for i=1:ws(1)
    for j=1:ws(2)
        count = 0;
        pt = [i,j]/res;
        temp_coverage = pt;
        % check for prime camera
        for p = 1:size(prime_c,1)
            cam_loc = prime_c(p,:);
            temp = 0;
            if isPtInUwbView2D(cam_loc,pt)
                count = count+1; temp = 1;
            end
            temp_coverage = [temp_coverage temp];
        end
        pp = pp+1;
        sensor_coverage(pp,:) = temp_coverage;           
        textprogressbar(pp*100/total_pts)
    end
end
textprogressbar('done');

disp('----------------------------------');

x = sum(sensor_coverage(:,3:end),2);
X = sensor_coverage(:,1); Y = sensor_coverage(:,2);
figure;scatter(X,Y,[],x,'filled'); c = colorbar;
xlabel('X (m)'); ylabel('Y (m)'); 
title('UWB sensor coverage')
c.Label.String = 'number of sensors covering the point';
hold on
quiver3(prime_c(:,1),prime_c(:,2),repmat(2.6,size(prime_c,1),1),cos(prime_c(:,3)),sin(prime_c(:,3)),zeros(size(prime_c,1),1),0.5,'LineWidth',1,'DisplayName','Prime13')
axis equal

nc_ind = find(x==4);
% disp(strcat('Percentage points covered with four sensors = ',num2str(size(nc_ind,1)*100/total_pts)));
figure;scatter(X(nc_ind),Y(nc_ind),[],x(nc_ind),'filled'); c = colorbar;
xlabel('X (m)'); ylabel('Y (m)'); 
title('UWB sensor - points with low coverage')
c.Label.String = 'number of sensors covering the point';
hold on
quiver3(prime_c(:,1),prime_c(:,2),repmat(2.5,size(prime_c,1),1),cos(prime_c(:,3)),sin(prime_c(:,3)),zeros(size(prime_c,1),1),0.5,'LineWidth',1,'DisplayName','Prime13')
axis equal

nc_ind = find(x<4);
% disp(strcat('Percentage uncovered Points                 = ',num2str(size(nc_ind,1)*100/total_pts)));
figure;scatter(X(nc_ind),Y(nc_ind),[],x(nc_ind),'filled'); c = colorbar;
xlabel('X (m)'); ylabel('Y (m)');
title('UWB sensor - uncovered points')
c.Label.String = 'number of sensors covering the point';
hold on
quiver3(prime_c(:,1),prime_c(:,2),repmat(2.5,size(prime_c,1),1),cos(prime_c(:,3)),sin(prime_c(:,3)),zeros(size(prime_c,1),1),0.5,'LineWidth',1,'DisplayName','Prime13')
axis equal

disp(strcat('Number of sensors used                      = ',num2str(size(prime_c,1))));
disp(strcat('Total coverage                              = ',num2str(100-size(nc_ind,1)*100/total_pts)));

disp('----------------------------------');

