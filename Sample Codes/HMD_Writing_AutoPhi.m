clear *
clc

%--------------Parameters---------------
mMirror = 1;% Motor for mirror
mHMD = 2;   % Motor for HMD
ELLZero = 223; % ELL stage position that corresponds to Phi=0°. The position
               % ELLZero=223 allows the HMD to continuously rotate in the
               % range Phi = (-90, 388)

tM = 1; % Minimum time [s] between consecutive move commands for the motors
tInterval = 3; % Delay time [s] between holograms in a single location
tPosChange = 30; % Estimate for how long it takes to change Phi

HMD = '24';
Disk = '-1';

%----------Load write schedule----------
file_dir = ['C:\Users\LAPT\Desktop\Synchronized Experiments\HMD Disk Data\HMD ',HMD,'\'];
file_name = ['HMD_',HMD,Disk,'_Writing-data.csv'];

param = readcell([file_dir,file_name]);
% The parameter CSV file contains the writing locations in the first
% column, the mirror angles in the next N columns, the HMD angles in
% the next N columns, the write times in the next N columns, and the
% write energies in the next N columns, where N is the number of gratings
% to be writen at one location. Thus, there will always be 4N+1 columns.
%


param(1,:)=[]; % Get rid of the text in the first row of the CSV
param(cellfun(@ismissing,param)) = {NaN};
param = cell2mat(param);

CSV_InfoBlockQuantity = 4;  % How many information blocks we have in the CSV.
                            % This does not include the Phi angles.

loc = param(1:end,1);
gNumMax =  (size(param,2)-1)/CSV_InfoBlockQuantity; % Maximum number of gratings in a single location.
for i=1 : length(loc)
    temp = param(i,:);
    temp = temp(~isnan(temp)); % Drop the NaNs. In case gNum /= gNumMax
    gNum(i) = (size(temp,2)-1)/CSV_InfoBlockQuantity; % Number of gratings at the i'th location.
end

for i=1 : length(loc)
    temp = param(i,(2:(1+gNumMax)));
    angles{i}(mMirror,:) = temp(~isnan(temp)); % Drop the NaNs. In case gNum /= gNumMax
    
    temp = param(i,((2+gNumMax):(1+2*gNumMax)));
    angles{i}(mHMD,:) = temp(~isnan(temp));
    
    temp = param(i,((2+2*gNumMax):(1+3*gNumMax)));
    times{i} = temp(~isnan(temp));
    
    % If you want the Ee at the i'th location. Use:
    %       temp = param(i,((2+3*gNumMax):(1+4*gNumMax)));
    %       Ee{i} = temp(~isnan(temp));
    
    % Round to 4 decimal places
    angles{i} = round(1e4.*angles{i})./1e4;
    times{i} = round(1e4.*times{i})./1e4;
    
    % Mirror angles: angles{i}(mMirror,:)
    % HMD    angles: angles{i}(mHMD,:)
end

% tot_Time = ((sum(gNum))*(2*tM+tInterval+1) + (length(loc)-1)*tPosChange + sum(cell2mat(times))); % [s]
% disp(sprintf('This will take approximately %.1f minutes\n', tot_Time./60));

%%
%-----Motor initialization-----
sM = serialport("COM3", 19200);
configureTerminator(sM,"CR");
disp('Motor controller connected');

%Turn motors on
writeline(sM, sprintf('%dMO',mHMD)); pause(0.1);
writeline(sM, sprintf('%dMO',mMirror));
pause(1);

% Go to 0°
writeline(sM, sprintf('%dPA%.4f',mHMD,0));
pause(tM);
writeline(sM, sprintf('%dPA%.4f',mMirror,0));
pause(tM);
%------------------------------

%----Shutter initialization----
sS = serialport("COM5",9600);
configureTerminator(sS,"CR");
%------------------------------
disp('Shutter controller connected');
writeline(sS, sprintf('mode=3')); % Sets shutter to 'Single' mode
ans = readline(sS);

%------ELL initialization------
mPhi = ELL18(serialport("COM6",9600));
disp('ELL Phi motor connected');
mPhi.Init();
mPhi.setVel(30);
mPhi.homeAbs(); % Find true 0
pause(0.5);
mPhi.Init();
mPhi.setVel(30);
mPhi.homeAbs(); % Find true 0
mPhi.homeAbs(); % All movement commands for the ELL module must be sent twice
                % In case there was a timeout
mPhi.HomePos = ELLZero;
mPhi.home(); % Go to Phi=0°
mPhi.home(); % Send movement command twice in case of timeout.
mPhi.setVel(50);
%------------------------------

disp('----Initial configuration completed----');

%%
aPhi = loc(1);
if (aPhi>=270) && (aPhi<=360)
   aPhi = aPhi-360;
end
mPhi.setPos(-1.5.*aPhi); % The program will pause until the position is
mPhi.setPos(-1.5.*aPhi); % reached or a timeout occurs. We set the position
                         % twice in case there was a timeout.

beep;pause(0.75);beep;
popUp = inputdlg('Start the exposure? All locations will be exposed.');
if popUp{1}=='0'
    beep; pause(0.4); beep; pause(0.4); beep;
    return;
end
beep;pause(0.4);beep;

for i= 1: length(loc)
    aPhi = loc(i);
    aMirror = angles{i}(mMirror,:);
    aHMD = angles{i}(mHMD,:);
    tExp = times{i};
    
    if (aPhi>=270) && (aPhi<=360)
       aPhi = aPhi-360;
    end
    
    mPhi.setPos(-1.5.*aPhi); % The program will pause until the position is
    mPhi.setPos(-1.5.*aPhi); % reached or a timeout occurs. We set the 
                             % position twice in case there was a timeout.
    pause(1);
    
    beep;pause(0.4);beep;
    
    for j=1:gNum(i)
        Mirr = aMirror(j);
        HMD = aHMD(j);
        t = tExp(j);
        
        % Go to motor positions.
        writeline(sM, sprintf('%dPA%.4f',mMirror,Mirr));
        pause(tM);
        writeline(sM, sprintf('%dPA%.4f',mHMD,HMD));
        pause(tM);
        % Program shutter
        writeline(sS, sprintf('open=%.0f',t.*1e3)); % Set exposure time
        ans = readline(sS);
        pause(tInterval);
        % Open shutter
        writeline(sS, sprintf('ens')); % Toggle the enable
        ans = readline(sS);
        pause(0.25);
        % Wait for shutter to close
        pause(ceil(t));
        pause(0.5);
    end
    disp(sprintf('Phi = %d° completed',loc(i)));
end


disp('Finished! Good Job =D');
beep; pause(0.4); beep; pause(0.4); beep;

mPhi.home();
mPhi.home();


