function transformedPt = transformPointToUwbFrame2D(cam_loc,pt)
% function to transform a point to camera frame
% inputs
% cam_loc: [x,y,theta]
% pt: point [x,y,z]

transformedPt = plus_transform(minus_transform(cam_loc'),[pt(1:2)';0]);
transformedPt = transformedPt(1:2);



end