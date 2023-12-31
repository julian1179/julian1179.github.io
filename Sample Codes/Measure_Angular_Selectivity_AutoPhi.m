clear *
close all
clc

cd('C:\Users\LAPT\Desktop\Synchronized Experiments\Instrument control\');
%--------------Parameters---------------
motor = 2; % Motor to control
Wavelength = 532; % nm
ELLZero = 223; % ELL stage position that corresponds to Phi=0°. The position
               % ELLZero=223 allows the HMD to continuously rotate in the
               % range Phi = (-90, 388)

AngleZero = -115; % Motor Angle where the input beam is normal to the HMD surface
AngleCenter = -131.8; % Center of the measurements, typically the peak D.E.
readAngleCenter = -(AngleZero-AngleCenter); % Center of the reading angles
% readAngleCenter = 14.7; % Center of the reading angles
Start = -1; % Scan goes from: AngleZero + readAngleCenter + Start
Stop = 1; %               to: AngleZero + readAngleCenter + Stop
Step = 0.01;
PhiOffset = 0; % Phi correction in degrees

Averages = 10;
maxPwr = 8; % mW

HMD = '24';
Disk = '-1';
modifier= '_LeftIn';

save = 1;
    base_dir = 'C:\Users\LAPT\Desktop\Synchronized Experiments\HMD Disk Data';
    dir = [base_dir, '\HMD ',HMD,'\HMD ',HMD,Disk,' Angle vs DE ',num2str(Wavelength),'nm MatLab data\'];
    anglesMotor = [AngleZero+readAngleCenter+Start : Step : AngleZero+readAngleCenter+Stop];
    anglesHOE = [readAngleCenter+Start : Step : readAngleCenter+Stop];

    % ----------Load write schedule----------
    file_dir = ['C:\Users\LAPT\Desktop\Synchronized Experiments\HMD Disk Data\HMD ',HMD,'\'];
    file_name = ['HMD_',HMD,Disk,'_Writing-data.csv'];
    
    param = readcell([file_dir,file_name]);
    param(1,:)=[]; % Get rid of the text in the first row of the CSV
    param(cellfun(@ismissing,param)) = {NaN};
    param = cell2mat(param);
    CSV_InfoBlockQuantity = 4;  % How many information blocks we have in the CSV.
                                % This does not include the Phi angles.
    loc = param(1:end,1);
    
    name = cell(size(loc));
    for i=1 : length(loc)
        name{i} =sprintf(['HMD_', HMD, Disk,'-phi%d_Ang_vs_DE_MatLab_%dnm_%.1f-%.1fdeg-%.4f',modifier],loc(i),Wavelength,min(anglesMotor),max(anglesMotor),Step);
    end
    loc = loc+PhiOffset;

t = 1.5;  % [s] Delay between setting the angle and reading the data.
      % Minimum value = ~0.5
%---------------------------------------
time_pr_Av = 0.003146;
tot_Time = length(loc)*(t + 2*Averages*time_pr_Av).*length(anglesMotor)./60;
disp(sprintf('This will take approximately %.1f minutes\n', tot_Time));
%%
%-----Motor initialization-----
s = serialport("COM3", 19200);
configureTerminator(s,"CR");
%------------------------------

%-----Laser initialization-----
verdi = serialport("COM4", 19200); % Remember to double check which COM port is used
configureTerminator(verdi,"CR/LF"); % Configure the terminator
writeline(verdi,"L:1"); % Laser on
pause(5); % Pause to let the laser seek
%------------------------------

%----PM100D initialization-----
SN1 = 'P0010557'; % Holo lab 1
SN2 = 'P0032176'; % Holo lab 2

pmHolo = PM100D(SN2);
pmHolo.setWavelength(Wavelength);
rng1 = pmHolo.setRange_mW(maxPwr);

pmTran = PM100D(SN1);
pmTran.setWavelength(Wavelength);
rng2 = pmTran.setRange_mW(maxPwr);
%------------------------------

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

% % If you ended up at the wrong home, run the following code as needed:
% mPhi.setPos_raw(359);
% mPhi.homeAbs();


t_ell = 3; tries = 0;
while (t_ell>2) % Go to Phi=0°
    tries = tries+1;
    tic;
    mPhi.home(); % Send movement command until home has been reached.
    t_ell= toc;
    if tries>=3
        pmHolo.close();
        pmTran.close();
        break;
        % error("Unable to set home position of the ELL auto-phi stage.");
    end
end

if tries>=3
    tries = 0;
    mPhi.getPos();
    mPhi.setPos(mPhi.Pos-1.5*(10));
    mPhi.setVel(50);
    while (t_ell>2) % Go to Phi=0°
        tries = tries+1;
        tic;
        mPhi.home(); % Send movement command until home has been reached.
        t_ell= toc;
        if tries>=3
            pmHolo.close();
            pmTran.close();
            % return;
            error("Unable to set home position of the ELL auto-phi stage.");
        end
    end
end

clear t_ell tries;
mPhi.setVel(50);
%------------------------------

%%
writeline(verdi,"S:1"); % Laser Shutter open

writeline(s, sprintf('%dMO',motor)); % Turn motor on
pause(1);
% writeline(s, sprintf('%dOR',motor)); % Search for home
% pause(5);
writeline(s, sprintf('%dPA%.4f',motor,anglesMotor(1)));

aPhi = loc(1);
if (aPhi>=270) && (aPhi<=360)
   aPhi = aPhi-360;
end
mPhi.setPos(-1.5.*aPhi); % The program will pause until the position is
mPhi.setPos(-1.5.*aPhi); % reached or a timeout occurs. We set the position
                         % twice in case there was a timeout.
pause(t);
%
figure
    axis([min(anglesHOE) max(anglesHOE) 0 100]);
    title(sprintf(['HMD ', HMD, Disk,'\nAngle vs DE\n(live)']));
    grid on;
c = lines(length(loc));
h = cell(size(loc));
for j=1 : length(loc)
    h{j} = animatedline('Color', c(j,:), 'Linestyle', 'none', 'Marker', '.');

    aPhi = loc(j);    
    if (aPhi>=270) && (aPhi<=360)
       aPhi = aPhi-360;
    end
    mPhi.setPos(-1.5.*aPhi); % The program will pause until the position is
    mPhi.setPos(-1.5.*aPhi); % reached or a timeout occurs. We set the 
                             % position twice in case there was a timeout.
    for i= 1: length(anglesMotor)
        if anglesHOE(i) == floor(anglesHOE(i))
            disp(sprintf('Starting angle= %d°',anglesHOE(i)));
        end
        
        pos = anglesMotor(i);
        writeline(s, sprintf('%dPA%.4f',motor,pos));
        pause(t);
        
        Tran(i) = pmTran.measureAv_mW(Averages);%.*1.0919 - 0.014888;
        pause(Averages*0.005);
        Holo(i) = pmHolo.measureAv_mW(Averages);
        
        Tot(i) = Tran(i)+Holo(i);
        DE(i) = 100.*Holo(i)./Tot(i);
        
        addpoints(h{j},anglesHOE(i),DE(i));
        drawnow update
        
    end
    if save
        if ~exist(dir) % If there isn't a folder for this disk at this wavelength, make the folder
            mkdir(dir) % Make 
        end
        m = [anglesHOE',Tran',Holo',Tot',DE'];
        extension = '.csv';
        writematrix(m,[dir,name{j},extension]); 
        fprintf('Finished saving data for Phi = %d°.\n',loc(j)-PhiOffset);
    else % Only run one loop if not saving
        break;
    end
    beep; pause(0.7); beep;
end
disp('Finished! Good Job =D');

%%
writeline(verdi,"S:0"); % Laser Shutter closed
writeline(verdi,"L:0"); % Laser off

pmHolo.close();
pmTran.close();
mPhi.home();
beep; pause(0.4); beep; pause(0.4); beep;
mPhi.home();
% writeline(s, sprintf('%dMF\r',motor)); % Turn motor off
pause(0.2);

%%
figure;
    plot(anglesMotor(1:length(DE)), DE,'.', 'Color', '#77AC30'); hold on;
    grid on;
    xlim([min(anglesMotor) max(anglesMotor)]);
    title(sprintf(['HMD ', HMD, Disk,'\nAngle vs DE']));
%     ylim([0 40]);




