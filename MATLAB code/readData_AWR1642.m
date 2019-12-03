clearvars; close all
delete(instrfind);

% Setup radar
configFile = "1642config.cfg";
[DATA_sphandle,UART_sphandle, ConfigParameters] = radarSetup16XX(configFile);

figure;
set(gcf, 'Position', get(0, 'Screensize'));
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Stop', ...
                    'Callback', 'stopVal = 1','Position',[100 600 100 30]);             
h(1) = polarscatter([],[],'filled');
thetalim([30,150]);
rlim([0,1]);
title('Raw data');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%                   MAIN   LOOP              %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%&&&&%%%%%%%%%%%%%%%%%%%%%%%%%
myInd = 1;
frame = [];
lastreadTime = 0;
dataOk = 0;
stopVal = 0;
previousTime = 0;
tic

while 1
    
    dataOk = 0;
    timeNow = toc;
    elapsedTime = timeNow - lastreadTime;   
    
    % Read the data from the radar:
    [dataOk, frameNumber, detObj] = readAndParseData16XX(DATA_sphandle, ConfigParameters);
    lastreadTime = timeNow;
    
    if dataOk == 1
        % Store all the data from the radar
        frame{myInd} = detObj;
        
        % Convert to polar coordinates
        [theta, ro] = cart2pol(-detObj.x,detObj.y);
        
        % Plot the polar plot:
        h(1).RData = ro;
        h(1).ThetaData = theta;
        
    end
    
    pause(0.03);
    if stopVal == 1
        stopVal = 0;
        break;
    end
    
end
fprintf(UART_sphandle, 'sensorStop');
delete(instrfind);
clear cam
close all



