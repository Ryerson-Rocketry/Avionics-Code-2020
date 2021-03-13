%transmitter info
tx = txsite('Name','Ground Station', ...
        'Latitude', 32.940727, ...
        'Longitude', -106.922101, 'AntennaHeight', 1.5, ...
        'SystemLoss', 10, ...
        "TransmitterFrequency", 900e6, ...
        "TransmitterPower", 10);
    
%receiver info
rx = rxsite('Name','Rocket', ...
       'Latitude',33.195200, ...
       'Longitude',-106.847713,'AntennaHeight',0.5,...
       'SystemLoss',10,...
       'Antenna', dipole,...
       'ReceiverSensitivity', -50);

%propagation model info
pm = propagationModel('longley-rice', ...
    'GroundConductivity', 4, ...
    'ClimateZone', 'desert');
 
%propagation data
x = coverage(tx, rx, 'SignalStrengths',-120:10,...
       'PropagationModel', pm);
t = x.Data;
t = sortrows(t,3,'descend');

toDelete = t.Power <= -121;
t(toDelete,:) = [];

d_largest = 0;
target = 0;
for i=1:height(t)
    [d_prev, na] = lldistkm([t.Latitude(i), t.Longitude(i)], [32.940727, -106.922101]);
    if d_largest<d_prev
        d_largest = d_prev;
        target = i;
    end
end
fprintf("Max. range: %f km\n", d_largest);
fprintf("Power at max. range: %f dB\n", t.Power(target));
coverage(tx, rx, 'SignalStrengths',-121:-50,...
       'PropagationModel', pm);

function [d1km, d2km]=lldistkm(latlon1,latlon2)
    radius=6371;
    lat1=latlon1(1)*pi/180;
    lat2=latlon2(1)*pi/180;
    lon1=latlon1(2)*pi/180;
    lon2=latlon2(2)*pi/180;
    deltaLat=lat2-lat1;
    deltaLon=lon2-lon1;
    a=sin((deltaLat)/2)^2 + cos(lat1)*cos(lat2) * sin(deltaLon/2)^2;
    c=2*atan2(sqrt(a),sqrt(1-a));
    d1km=radius*c;    %Haversine distance
    x=deltaLon*cos((lat1+lat2)/2);
    y=deltaLat;
    d2km=radius*sqrt(x*x + y*y); %Pythagoran distance
end





