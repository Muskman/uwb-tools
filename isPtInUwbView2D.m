function x = isPtInUwbView2D(uwb_loc, pt)
% function to check if point is in camera view
% inputs
% uwb_type: type of uwb, either narrow or wide
% uwb_loc: [x,y,theta]
% pt: point [x,y]

% transform point to camera frame
transformedPt = transformPointToUwbFrame2D(uwb_loc,pt);
% transformedPt = pt;


% check if point in range
a = transformedPt(1)<60 & transformedPt(1)>-5;
% check if point in FOV
r = abs(transformedPt(2));
if(a && transformedPt(1)>0)
    b = r<25*(1-transformedPt(1)/60);
elseif(a && transformedPt(1)<0)
    b = r<25*(1+transformedPt(1)/5);
else
    b = 0;
end


x = a & b;

end