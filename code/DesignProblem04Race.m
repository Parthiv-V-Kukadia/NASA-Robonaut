function DesignProblem04Race(dirname,roadfile,varargin)
% DesignProblem04Race   run simulation of two-wheeled robot race
%
%   DesignProblem04Race('dirname','roadfile') looks in the directory
%       'dirname' for controllers and loads the road from 'roadfile'.
%
%   DesignProblem04Race('FunctionName','P1',V1,'P2','V2',...) optionally
%       defines a number of parameter values:
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%
%           'snapshotfile' : a filename (e.g., 'snap.pdf') where, if
%                            defined, a PDF with a snapshot of the last
%                            frame of the simulation will be saved
%
%           'display' : a flag...
%
%                       - If true, it will clear the current figure and
%                         will show the simulation. To quite, type 'q' when
%                         this figure is in the foreground.
%
%                       - If false, it will not show any graphics and will
%                         run the simulation as fast as possible (not in
%                         real-time).
%
%           'seed' : a value that, if defined, will be used to seed the
%                    random number generator
%
%           'savedata' : a flag - true or false (default) that, if true,
%                        will save logged data from each controller to a
%                        file 'dirname/*_data.mat' where '*' is the name
%                        associated with the controller

% Parse the arguments
% - Create input parser
p = inputParser;
% - Parameter names must be specified in full
p.PartialMatching = false;
% - This argument is required, and must be first
addRequired(p,'dirname',@ischar);
% - This argument is required, and must be second
addRequired(p,'roadfile',@ischar);
% - These parameters are optional, and can be in any order
addParameter(p,'moviefile',[],@ischar);
addParameter(p,'snapshotfile',[],@ischar);
addParameter(p,'display',true,@islogical);
addParameter(p,'seed',[]);
addParameter(p,'savedata',false,@islogical);
addParameter(p,'rainbow',false,@islogical);
% - Apply input parser
parse(p,dirname,roadfile,varargin{:});
% - Extract parameters
params = p.Results;


% Define other parameters
params.resultsfile = 'results.mat';
params.students = GetStudents(params.dirname);
params.nstudents = length(params.students);
params.tStop = 150;
params.delayBonus = 2;

% Get path
curPath = path;

% Add directory with controllers to path
addpath(params.dirname);

% Either store or set RNG seed
if isempty(params.seed)
    params.rngSettings = rng;
else
    params.rngSettings = rng(params.seed);
end

% Define start time and time step
params.tStart = 0;
params.tStep = 1/50;

% Load the road
if (exist(params.roadfile,'file')==2)
    fprintf(1,'Loading road from file %s.\n',params.roadfile);
    load(params.roadfile);
    params.road = road;
else
    error(sprintf('Road file %s does not exist.\n',params.roadfile));
end

% Setup the simulation for each robot
processList = {};
controllerList = {};
for i=1:params.nstudents
    
    process = struct;
    process.controller = params.students(i).filename;
    process.team = params.students(i).name;
    if (params.savedata)
        process.datafile = sprintf('%s/%s_data.mat',params.dirname,params.students(i).name);
    else
        process.datafile = [];
    end
    
    try
        [process,controller] = SetupSimulation(process,params);
        processList{end+1} = process;
        controllerList{end+1} = controller;
    catch exception
        fprintf(1,' skipped (bad setup): %s\n',params.students(i).filename);
    end
end

if isempty(processList)
    error('Found no viable controllers.');
end

% Run the simulation
RunSimulation(processList,controllerList,params);

% Restore path
path(curPath);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process,params)

% DEFINE CONSTANTS

% Constants related to simulation.

% - Start time.
process.tStart = params.tStart;
% - Time step.
process.tStep = params.tStep;
% - Stop time.
process.tStop = params.tStop;

% - Names of things to log in datafile, if desired
process.processdatatolog = {'t','x','y','theta','phi','phidot','v','w','e_lateral','e_heading','result'};

% - Gravity
process.g = 9.81;
% - Density (kg/m^3)
process.rho = 200;
% - Lengths (m)
process.b = 0.4;
process.r = 0.2;
process.l = 0.2;
process.wheelWidth = (0.2*process.b);
process.bodyWidth = (0.8*process.b);
process.bodyExtension = 0.2*process.r;
process.bodyDepth = 0.5*process.b;
process.bodyHeight = 2*(process.l+process.bodyDepth);
% - Masses (kg)
process.mo = process.rho*process.bodyWidth*process.bodyHeight*process.bodyDepth;
process.mw = process.rho*pi*process.r^2*process.wheelWidth;
% - Moments of inertia
process.Jx = (process.mo/12)*(process.bodyWidth^2+process.bodyHeight^2);
process.Jy = (process.mo/12)*(process.bodyDepth^2+process.bodyHeight^2);
process.Jz = (process.mo/12)*(process.bodyWidth^2+process.bodyDepth^2);
process.Jw = (process.mw/2)*process.r^2;

% - EOM
filename = 'DesignProblem04_EOMs.mat';
if (exist(filename,'file')==2)
    fprintf(1,'Loading EOMs from file (delete %s to start fresh).\n',filename);
    load(filename);
else
    [symEOM,numEOM] = GetEOM(process.g,...
                             process.Jx,process.Jy,process.Jz,process.Jw,...
                             process.mo,process.mw,...
                             process.b,process.r,process.l);
	fprintf(1,'Saving EOMs to file (load %s to work with them).\n',filename);
	save('DesignProblem04_EOMs.mat','symEOM','numEOM');
end
process.symEOM = symEOM;
process.numEOM = numEOM;

% - Maximum torque
process.tauMax = 5;

% - Road and road width
process.road = params.road;
process.roadwidth = params.road.roadwidth;

% - Initial states
process.initial = [0;0;0;0;0;0;0]+[0;0;0.01;0.01;0.01;0.01;0.01].*randn(7,1);

% - Controller data to log
process.controllerdatatolog = [];

% DEFINE VARIABLES

% Time
process.t = 0;
% States
process.x = process.initial(1,1);
process.y = process.initial(2,1);
process.theta = process.initial(3,1);
process.phi = process.initial(4,1);
process.phidot = process.initial(5,1);
process.v = process.initial(6,1);
process.w = process.initial(7,1);
process.psiR = 0;
process.psiL = 0;
% Extra stuff
[process.sClosest,dClosest,qclosest] = GetClosestPoint([process.x;process.y;process.theta],0,process.road);
xc = qclosest(1,1);
yc = qclosest(2,1);
thetac = qclosest(3,1);
process.e_lateral = -[-sin(thetac); cos(thetac)]'*[process.x-xc;process.y-yc];
process.e_heading = angdiff(process.theta,thetac);
process.pClosest = [xc;yc];
process.result = nan;
% Maximum computation time for "run"
process.maxComputationTime = process.tStep;
% Number of times "run" has computation time too long
process.numTooLong = 0;
% Max value of "tooLong" before controller shuts off.
process.maxNumTooLong = 5;

% DEFINE CONTROLLER

% Functions
% - get handles to user-defined functions 'init' and 'run'
%   (as well as the value of iDelay, if defined)
controller = eval(process.controller);
controller.name = process.controller;
if isfield(controller,'iDelay')
    process.iDelay = floor(controller.iDelay);
    if (process.iDelay<0)
        process.iDelay = 0;
    end
    fprintf(1,'     controller %s has iDelay=%d\n',controller.name,process.iDelay);
else
    process.iDelay = 0;
end
% Parameters
% - define a list of constants that will be passed to the controller
names = {'tStep','tauMax','roadwidth','symEOM','numEOM','b','r'};
% - loop to create a structure with only these constants
controller.parameters = struct;
for i=1:length(names)
    controller.parameters.(names{i}) = process.(names{i});
end
% Storage
controller.data = struct;
% Status
controller.hasrunonce = false;
controller.running = true;
% Init
tStart = tic;
try
    [command_window_output,controller.data] = ...
        evalc(['controller.init(controller.parameters, ' ...
                               'controller.data)']);
catch exception
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'threw the following error:\n\n' ...
             '==========================\n' ...
             '%s\n', ...
             '==========================\n\n' ...
             'Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,getReport(exception));
	controller.actuators = ZeroActuators();
    controller.running = false;
end
if ~isempty(command_window_output)
    warning(['The ''init'' function of controller\n     ''%s''\n' ...
             'printed this text to the command window:\n\n%s\n\n' ...
             'It should not be printing any text to the command\n' ...
             'window. Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,command_window_output);
    controller.actuators = ZeroActuators();
    controller.running = false;
end
controller.tInit = toc(tStart);
% Get reference values
controller.references = GetReferences(process);
% Get sensor values
sensors = CreateSensors(process);
for i=0:process.iDelay
    process.sensors_delayed{i+1} = sensors;
end
controller.sensors = GetSensors(process);
% Get actuator values (run controller)
[process,controller] = RunController(process,controller);
end

function [process,controller] = RunController(process,controller)
% Get actuator values (run controller)
if (controller.running)
    tStart = tic;
    try
        if (controller.hasrunonce)
            command_window_output = [];
            [controller.actuators,controller.data] = ...
                controller.run(controller.sensors, ...
                                  controller.references, ...
                                  controller.parameters, ...
                                  controller.data);
        else
            controller.hasrunonce = true;
            [command_window_output,controller.actuators,controller.data] = ...
                evalc(['controller.run(controller.sensors, ' ...
                       'controller.references, ' ...
                       'controller.parameters, ' ...
                       'controller.data)']);
        end
    catch exception
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name,getReport(exception));
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''run'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tRun = toc(tStart);
    
    % Check if there was output to the command window
    if (~isempty(command_window_output))
        warning(['The controller\n     ''%s''\n' ...
             'printed this text to the command window:\n\n%s\n\n' ...
             'It should not be printing any text to the command\n' ...
             'window. Turning off controller and setting all\n' ...
             'actuator values to zero.\n'],controller.name,command_window_output);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    
    % Check if computation time was too long
    if controller.tRun > process.maxComputationTime
        % Increment number of times computation was too long
        process.numTooLong = process.numTooLong + 1;
        % Check if reached the maximum number of times
        if process.numTooLong > process.maxNumTooLong
            
            warning(['The controller\n     ''%s''\n' ...
                     'exceeded the maximum computation time on\n' ...
                     '%d different occasions. Turning off controller\n' ...
                     'and setting all actuator values to zero.\n'],...
                     controller.name,process.maxNumTooLong);
            
            controller.actuators = ZeroActuators();
            controller.running = false;
        end
    end

else
    controller.tRun = 0;
end
end

function references = GetReferences(process)
references = struct;
end

function [symEOM,numEOM] = GetEOM(g,Jx,Jy,Jz,Jw,mo,mw,b,r,l)
% States
syms phi phidot v w real
% Inputs
syms tauR tauL real
% EOMs
J5 = (Jx+mo*l^2)*sin(phi)^2+Jz*cos(phi)^2+(3/4)*mw*b^2+(1/2)*mw*r^2;
M = [Jy+mo*l^2              mo*l*cos(phi)             0;
     mo*l*cos(phi)        mo+2*mw+(2*Jw/r^2)          0;
     0                      0                           J5];
N = [(1/2)*(Jz-Jx-mo*l^2)*w^2*sin(2*phi)-mo*l*g*sin(phi);
     -mo*l*sin(phi)*(w^2+phidot^2);
     (Jx-Jz+mo*l^2)*phidot*w*sin(2*phi)+mo*l*v*w*sin(phi)];
R = [-1         -1;
     1/r        1/r;
     b/(2*r)      -b/(2*r)];
% Symbolic
symEOM.M = M;
symEOM.N = N;
symEOM.R = R;
symEOM.f = simplify(inv(M)*(R*[tauR;tauL]-N));
% Numeric
numEOM.f = matlabFunction(symEOM.f,'Vars',[phi phidot v w tauR tauL]);
end

function sensors = CreateSensors(process)
sensors.t = process.t;
sensors.e_lateral = process.e_lateral;
sensors.e_heading = process.e_heading;
sensors.wR = (1/process.r)*(process.v+(process.b/2)*process.w);
sensors.wL = (1/process.r)*(process.v-(process.b/2)*process.w);
[qtmp,wdes] = WhereAmI(process.sClosest,1,process.road);
sensors.r_road = 1/wdes;

% Add noise
%   (nothing)
end

function process = UpdateSensors(process)
sensors = CreateSensors(process);
process.sensors_delayed = process.sensors_delayed(2:end);
process.sensors_delayed{end+1} = sensors;
end

function sensors = GetSensors(process)
sensors = process.sensors_delayed{1};
end

function [t,x] = Get_TandX_From_Process(process)
t = process.t;
x = [process.x;
     process.y;
     process.theta;
     process.phi;
     process.phidot;
     process.v;
     process.w;
     process.psiR;
     process.psiL];
end

function u = GetInput(process,actuators)
% Copy input from actuators
u = [actuators.tauR; actuators.tauL];

% Bound input
for i=1:length(u)
    if (u(i) < -process.tauMax)
        u(i) = -process.tauMax;
    elseif (u(i) > process.tauMax)
        u(i) = process.tauMax;
    end
end

% Add disturbance
%   (nothing)
end

function process = Get_Process_From_TandX(t,x,process)
% The normal stuff
process.t = t;
process.x = x(1,1);
process.y = x(2,1);
process.theta = x(3,1);
process.phi = x(4,1);
process.phidot = x(5,1);
process.v = x(6,1);
process.w = x(7,1);
process.psiR = x(8,1);
process.psiL = x(9,1);
% Update closest point
[process.sClosest,dClosest,qclosest] = GetClosestPoint([process.x;process.y;process.theta],process.sClosest,process.road);
xc = qclosest(1,1);
yc = qclosest(2,1);
thetac = qclosest(3,1);
process.e_lateral = -[-sin(thetac); cos(thetac)]'*[process.x-xc;process.y-yc];
process.e_heading = angdiff(process.theta,thetac);
process.pClosest = [xc;yc];
%
% You would expect e_lateral and dClosest to be the same. They are not. The
% reason is numerical error. "GetClosestPoint" compares the current pose to
% points that are sampled along the road at some fixed resolution.
%
% For this reason and in the interest of creating a "starting box," I've
% changed from checking e_lateral to checking dClosest when detecting a
% crash.
%
% OLD:
% if (abs(process.e_lateral)>0.5*process.road.roadwidth)
%
if (dClosest>0.5*process.road.roadwidth)
    % Crash (off road)
    process.result = 0;
elseif (((2*process.l+process.bodyExtension)*cos(abs(process.phi)))+...
        ((process.bodyDepth/2)*sin(abs(process.phi))))<=0
    % Crash (dropped payload)
    process.result = 0;
elseif (process.sClosest>=process.road.s(end))
    % Win
    process.result = 1;
end
% Update sensors
process = UpdateSensors(process);
end

function xdot = GetXDot(t,x,u,process)
theta = x(3,1);
phi = x(4,1);
phidot = x(5,1);
v = x(6,1);
w = x(7,1);
tauR = u(1,1);
tauL = u(2,1);
xdot = [v*cos(theta);
        v*sin(theta);
        w;
        phidot;
        process.numEOM.f(phi,phidot,v,w,tauR,tauL);
        (1/process.r)*(v+(process.b/2)*w);
        (1/process.r)*(v-(process.b/2)*w)];
end

function iscorrect = CheckActuators(actuators)
iscorrect = false;
if all(isfield(actuators,{'tauR','tauL'}))&&(length(fieldnames(actuators))==2)
    if isnumeric(actuators.tauR)&&isnumeric(actuators.tauL)
        if isscalar(actuators.tauR)&&isscalar(actuators.tauL)
            if (~isnan(actuators.tauR))&&(~isinf(actuators.tauR))&&(~isnan(actuators.tauL))&&(~isinf(actuators.tauL))
                iscorrect = true;
            end
        end
    end
end
end

function actuators = ZeroActuators()
actuators = struct('tauR',0,'tauL',0);
end

function res = ShouldStop(process)
% Check stopping condition (crash, win)
if ~isnan(process.result)
    res = true;
else
    res = false;
end
end

function fig = UpdateFigure(fig,processList,controllerList,params)
if (isempty(fig))
    % CREATE FIGURE

    % Clear the current figure.
    clf;

    % Create an axis for text (it's important this is in the back,
    % so you can rotate the view and other stuff!)
    fig.text.axis = axes('position',[0 0 1 1]);
    axis([0 1 0 1]);
    hold on;
    axis off;
    fs = 14;
    
    % Text for each robot
    for i=1:length(processList)
        
        % Define status
        if (processList{i}.result==1)
            status = 'finished';
        elseif (processList{i}.result==0)
            status = 'crashed';
        elseif (controllerList{i}.running)
            status = 'on';
        else
            status = 'off';
        end
        
        % Define distance
        s = processList{i}.sClosest;
        
        % name : time : distance : status
        fig.text.robot(i) = ...
            text(0.01,1.0-i*0.05,...
                 sprintf('%10.10s : %6.3f seconds : %6.1f meters : %s',...
                         processList{i}.team,...
                         processList{i}.t-processList{i}.iDelay*params.delayBonus,...
                         s,...
                         status),...
                 'fontsize',16,...
                 'fontname','fixedwidth',...
                 'fontweight','bold');
        
        
    end
    
    set(gcf,'renderer','opengl');
    set(gcf,'color','w');
    
    fig.view0.axis = axes('position',[-0.75 -0.2 2.5 1.5]);
    axis equal;
    fig.view0.dx = 3;
    fig.view0.dy = 3;
    
    % Find the leader
    sMax = -Inf;
    for i=1:length(processList)
        if processList{i}.sClosest>sMax
            sMax = processList{i}.sClosest;
            xMax = processList{i}.pClosest(1);
            yMax = processList{i}.pClosest(2);
%             xMax = processList{i}.x;
%             yMax = processList{i}.y;
        end
    end
    
    set(fig.view0.axis,'xlim',[xMax-fig.view0.dx xMax+fig.view0.dx]);
    set(fig.view0.axis,'ylim',[yMax-fig.view0.dy yMax+fig.view0.dy]);
    set(fig.view0.axis,'zlim',[0 1]);
    axis manual;
    hold on;
    axis off;
    box on;
    
    view([180-37.5,20]);
    set(gca,'projection','perspective');
    set(gca,'clipping','on','clippingstyle','3dbox');
    lighting gouraud
    fig.view0.light = light('position',[xMax;yMax;1],'style','local');
    
    for iProcess=1:length(processList)
        process = processList{iProcess};
    
        % - colors
        robot.colors.uiuc_orange=[1,0.6,0];
        robot.colors.uiuc_blue=[0.4745,0.6471,0.9098];

        % - dimensions
        robot.a = process.wheelWidth/2;
        robot.r = process.r;
        robot.b = process.bodyWidth/2;
        robot.c = process.bodyExtension;
        robot.d = 0.5*process.b;

        % - vertices, faces, and colors for both wheels
        n = 32;
        p = [[0;robot.a;0] -[0;robot.a;0]];
        ang = linspace(0,2*pi,n+1);
        f1 = [];
        f2 = [];
        for i=1:n
            p = [p [robot.r*cos(ang(i));0;robot.r*sin(ang(i))]];
            f1 = [f1; [1 2+i 3+i]];
            f2 = [f2; [2 3+i 2+i]];
        end
        f = [f1;f2];
        f(f==3+n)=3;
        c = [repmat(robot.colors.uiuc_blue,floor(n/4),1);
             repmat(robot.colors.uiuc_orange,n-floor(n/4),1);
             repmat(robot.colors.uiuc_blue,floor(n/8),1);
             repmat(robot.colors.uiuc_orange,floor(n/8),1);
             repmat(robot.colors.uiuc_blue,floor(n/8),1);
             repmat(robot.colors.uiuc_orange,n-3*floor(n/8),1)];
        % (#2 is L wheel frame, #3 is R wheel frame)
        robot.wheel2.p_in2 = p;
        robot.wheel2.faces = f;
        robot.wheel2.colors = c;
        robot.wheel3.p_in3 = p;
        robot.wheel3.faces = f;
        robot.wheel3.colors = c;

        % - vertices, faces, and colors for chassis
        % (#1 is chassis frame)
        dx = robot.d;
        dy = 2*robot.b;
        dz = 2*(process.l+robot.c);
        p = [0.5*dx*[-1 1 1 -1 -1 1 1 -1];
             0.5*dy*[-1 -1 -1 -1 1 1 1 1];
             0.5*dz*[-1 -1 1 1 -1 -1 1 1]];
        robot.chassis.p_in1 = p;
        robot.chassis.faces =  [1 2 3;
                                3 4 1;
                                2 6 7;
                                7 3 2;
                                6 5 8;
                                8 7 6;
                                5 1 4;
                                4 8 5;
                                4 3 7;
                                7 8 4;
                                5 6 2;
                                2 1 5];
        robot.chassis.colors = [repmat(robot.colors.uiuc_blue,2,1);
                                repmat(robot.colors.uiuc_orange,10,1)];
        
        % - transformations
        o_10in0 = [process.x;process.y;process.r];
        R_10in0 = RZ(process.theta);
        R_1in0 = R_10in0*RY(process.phi);
        o_1in0 = o_10in0+R_1in0*[0;0;process.l];
        R_2in0 = R_10in0*RY(process.psiL);
        o_2in0 = o_10in0+R_2in0*[0;process.b/2;0];
        R_3in0 = R_10in0*RY(process.psiR);
        o_3in0 = o_10in0+R_3in0*[0;-process.b/2;0];
        robot.chassis.p_in0 = Transform(o_1in0,R_1in0,robot.chassis.p_in1);
        robot.wheel2.p_in0 = Transform(o_2in0,R_2in0,robot.wheel2.p_in2);
        robot.wheel3.p_in0 = Transform(o_3in0,R_3in0,robot.wheel3.p_in3);
        % - robot
        robot.view0 = DrawRobot([],robot,1);
        
        robot.text = text(process.x,process.y,0.8,...
                          sprintf('%.8s',process.team),...
                          'fontsize',18,...
                          'HorizontalAlignment','center');
        
        
        fig.robot(iProcess) = robot;
        
    end
    
    % - road
    fig.view0.road = DrawRoad(params.road,5e-2,params.rainbow);
    
    % Make the figure respond to key commands.
    set(gcf,'KeyPressFcn',@onkeypress);
end

% UPDATE FIGURE

for i=1:length(processList)
    process = processList{i};

    % - transformations
    o_10in0 = [process.x;process.y;process.r];
    R_10in0 = RZ(process.theta);
    R_1in0 = R_10in0*RY(process.phi);
    o_1in0 = o_10in0+R_1in0*[0;0;process.l];
    R_2in0 = R_10in0*RY(process.psiL);
    o_2in0 = o_10in0+R_2in0*[0;process.b/2;0];
    R_3in0 = R_10in0*RY(process.psiR);
    o_3in0 = o_10in0+R_3in0*[0;-process.b/2;0];
    fig.robot(i).chassis.p_in0 = Transform(o_1in0,R_1in0,fig.robot(i).chassis.p_in1);
    fig.robot(i).wheel2.p_in0 = Transform(o_2in0,R_2in0,fig.robot(i).wheel2.p_in2);
    fig.robot(i).wheel3.p_in0 = Transform(o_3in0,R_3in0,fig.robot(i).wheel3.p_in3);
    % - robot
    fig.robot(i).view0 = DrawRobot(fig.robot(i).view0,fig.robot(i));
    set(fig.robot(i).text,'position',[process.x,process.y,0.8]);

end

% Find the leader
sMax = -Inf;
for i=1:length(processList)
    if processList{i}.sClosest>sMax
        sMax = processList{i}.sClosest;
        xMax = processList{i}.pClosest(1);
        yMax = processList{i}.pClosest(2);
    end
end

% Center axis on leader
set(fig.view0.axis,'xlim',[xMax-fig.view0.dx xMax+fig.view0.dx]);
set(fig.view0.axis,'ylim',[yMax-fig.view0.dy yMax+fig.view0.dy]);

% Put spotlight on leader
set(fig.view0.light,'position',[xMax;yMax;1]);

for i=1:length(processList)

    % Define status
    if (processList{i}.result==1)
        status = 'finished';
    elseif (processList{i}.result==0)
        status = 'crashed';
    elseif (controllerList{i}.running)
        status = 'on';
    else
        status = 'off';
    end

    % Define distance
    s = processList{i}.sClosest;

    % name : time : distance : status
    set(fig.text.robot(i),'string',...
             sprintf('%10.10s : %6.2f seconds : %6.1f meters : %s',...
                     processList{i}.team,...
                     processList{i}.t-processList{i}.iDelay*params.delayBonus,...
                     s,...
                     status));

end
    
drawnow;
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(processList,controllerList,params)

% START-UP

% Create empty figure.
fig = [];

% Flag to stop simulation on keypress.
global done
done = false;

% Start making movie, if necessary.
if (~isempty(params.moviefile))
    myV = VideoWriter(sprintf('%s/%s',params.dirname,params.moviefile),'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 1/params.tStep;
    open(myV);
end

% LOOP

% Start timer if display.
if (params.display)        
    iStep = 0;
    tStart = tic;
end

% Loop until break.
while (1)

    % Update figure (create one if fig is empty).
    if (params.display)
        fig = UpdateFigure(fig,processList,controllerList,params);
    end

    % Update data.
    for i=1:length(processList)
        if (~isempty(processList{i}.datafile))
            [processList{i},controllerList{i}] = UpdateDatalog(processList{i},controllerList{i});
        end
    end

    % If making a movie, store the current figure as a frame.
    if (~isempty(params.moviefile))
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end

    % Stop if done.
    alldone=true;
    for i = 1:length(processList)
        if ~ShouldStop(processList{i})
            alldone=false;
            % Update process (integrate equations of motion).
            [processList{i},controllerList{i}] = UpdateProcess(processList{i},controllerList{i});
        end
    end
    if (done||alldone)
        break;
    end
    
    % Wait if display and if necessary, to stay real-time.
    if (params.display)
        iStep = iStep+1;
        while (toc(tStart)<iStep*params.tStep)
            % Do nothing
        end
    end

end

% SHUT-DOWN

% Close and save the movie, if necessary.
if (~isempty(params.moviefile))
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% Save the data.
for i=1:length(processList)
    if (~isempty(processList{i}.datafile))
        processdata = processList{i}.log.process; %#ok<NASGU>
        controllerdata = processList{i}.log.controller; %#ok<NASGU>
        save(processList{i}.datafile,'processdata','controllerdata');
    end
end

% Save the snapshot, if necessary.
if (~isempty(params.snapshotfile))
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',sprintf('%s/%s',params.dirname,params.snapshotfile));
end

% Save the results.
if (~isempty(params.resultsfile))
    rngSettings = params.rngSettings;
    for i=1:length(processList)
        results(i).team = processList{i}.team;
        results(i).time = processList{i}.t-processList{i}.iDelay*params.delayBonus;
        results(i).status = processList{i}.result;
        results(i).controller = processList{i}.controller;
    end
    save(sprintf('%s/%s',params.dirname,params.resultsfile),'rngSettings','results','params');
end

end


function [process,controller] = UpdateDatalog(process,controller)
% Create data log if it does not already exist.
if (~isfield(process,'log'))
    process.log = struct('process',struct,...
                         'controller',struct('tInit',[],...
                                             'tRun',[],...
                                             'sensors',struct,...
                                             'actuators',struct,...
                                             'data',struct),...
                         'count',0);
end
% Increment log count.
process.log.count = process.log.count+1;
% Write process data to log.
for i=1:length(process.processdatatolog)
    name = process.processdatatolog{i};
    process.log.process.(name)(:,process.log.count) = process.(name);
end
% Write controller data to log, if controller is running.
if controller.running
    process.log.controller.tInit = controller.tInit;
    process.log.controller.tRun(:,process.log.count) = ...
        controller.tRun;
    names = fieldnames(controller.sensors);
    for i=1:length(names)
        name = names{i};
        process.log.controller.sensors.(name)(:,process.log.count) = ...
            controller.sensors.(name);
    end
    names = fieldnames(controller.actuators);
    for i=1:length(names)
        name = names{i};
        process.log.controller.actuators.(name)(:,process.log.count) = ...
            controller.actuators.(name);
    end
    for i=1:length(process.controllerdatatolog)
        name = process.controllerdatatolog{i};
        try
            process.log.controller.data.(name)(:,process.log.count) = ...
                controller.data.(name);
        catch exception
            warning(['Saving element ''%s'' of data for controller\n',...
                     '     ''%s''',...
                     'threw the following error:\n\n' ...
                     '==========================\n' ...
                     '%s\n', ...
                     '==========================\n\n' ...
                     'Turning off controller and setting all\n' ...
                     'actuator values to zero.\n'],...
                     name,controller.name,getReport(exception));
            controller.actuators = ZeroActuators();
            controller.running = false;
            return
        end
    end
end
end


function [process,controller] = UpdateProcess(process,controller)
% Integrate equations of motion
[t0,x] = Get_TandX_From_Process(process);
u = GetInput(process,controller.actuators);
[t,x] = ode45(@(t,x) GetXDot(t,x,u,process),[t0 t0+process.tStep],x);
process = Get_Process_From_TandX(t(end),x(end,:)',process);

% Get reference values
controller.references = GetReferences(process);

% Get sensor values
controller.sensors = GetSensors(process);

% Get actuator values (run controller)
[process,controller] = RunController(process,controller);
end

function students = GetStudents(dirname)
listing = dir(dirname);
fprintf(1,'\nFINDING CONTROL SYSTEMS\n');
fprintf(1,' filename : name\n');
nstudents = 0;
for i=1:length(listing)
    if (~listing(i).isdir)
        [tmp1,student.filename,tmp2] = fileparts(listing(i).name);
        if (~isempty(student.filename))
            nstudents = nstudents + 1;
            data = textscan(student.filename,'%s');
            data = strsplit(data{1}{1},'_');
            student.name = data{1};
            students(nstudents) = student;
            fprintf(1,'  %s : %s\n',student.filename,student.name);
        end
    end
end
fprintf(1,'\n\n');
if (nstudents == 0)
    error('Found no control systems in folder ''%s''.',dirname);
end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% HELPER FUNCTIONS
%

function onkeypress(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

function p_inj = Transform(o_kinj,R_kinj,p_ink)
p_inj = zeros(size(p_ink));
for i=1:size(p_ink,2)
    p_inj(:,i) = o_kinj + R_kinj*p_ink(:,i);
end
end

function dh=angdiff(h2,h1)
% Returns the shortest path from h1 to h2.
% (The vector from h1 to h2 in S^1. Note this
%  is NOT always equal to h2-h1.)
dh=mod((h2-h1)+pi,2*pi)-pi;
end

% Returns the current pose "q" and turning rate "w" on the road, given the
% current arc-length "s" along the road and the current forward speed "v".
function [q,w] = WhereAmI(s,v,road)
if (s>=road.s(end))
    
    q = road.q(:,end);
    w = v*road.w(end);
    
else
    
    i = find(s<road.s,1,'first');
    
    q0 = road.q(:,i-1);
    ds = s-road.s(i-1);
    dh = road.w(i)*ds;
    
    dq = [ds*mysinc(dh/2)*cos(q0(3)+(dh/2));
          ds*mysinc(dh/2)*sin(q0(3)+(dh/2));
          dh];
    q = q0+dq;
    
    w = v*road.w(i);
    
end
end

function [sclosest,dclosest,qclosest] = GetClosestPoint(q,s,road)
% INPUTS:
%   q = [x;y;theta]
%   s = initial guess
%   road = the road
% OUTPUTS: for the closest point...
%   sclosest = arclength
%   dclosest = e_lateral (not really...)
%   qclosest = [x;y;theta]
ds_res = 1e-2;
ds_max = 0.25;
sclosest = nan;
qclosest = nan;
dclosest = inf;
smin = max(0,s-ds_max);
smax = min(road.s(end),s+ds_max);
s = linspace(smin,smax,1+ceil((smax-smin)/ds_res));
for i=1:length(s)
    qroad = WhereAmI(s(i),1,road);
    d = norm(qroad(1:2)-q(1:2));
    if (d<dclosest)
        sclosest = s(i);
        dclosest = d;
        qclosest = qroad;
    end
end
end

% Returns sin(x)/x (MATLAB defines "sinc" slightly differently).
function y=mysinc(x)
y = sinc(x/pi);
end

function h=DrawRoad(road,ds,rainbow)
s = 0;
q = road.q(:,1);
w = road.w(:,2);
while (s<road.s(end))
    
    s = s+ds;
    [qcur,wcur] = WhereAmI(s,1,road);
    q(:,end+1) = qcur;
    w(:,end+1) = wcur;
    
end
qL = q(1:2,:)+0.5*road.roadwidth*[-sin(q(3,:)); cos(q(3,:))];
qR = q(1:2,:)-0.5*road.roadwidth*[-sin(q(3,:)); cos(q(3,:))];
hStart = linspace(pi/2,3*pi/2,1+ceil(pi*road.roadwidth/ds));
hEnd = linspace(-pi/2,pi/2,1+ceil(pi*road.roadwidth/ds));
qStart = [q(1,1)+0.5*road.roadwidth*cos(hStart+q(3,1)); q(2,1)+0.5*road.roadwidth*sin(hStart+q(3,1))];
qEnd = [q(1,end)+0.5*road.roadwidth*cos(hEnd+q(3,end)); q(2,end)+0.5*road.roadwidth*sin(hEnd+q(3,end))];
qroad = [qStart qR qEnd fliplr(qL) qR(:,1)];
if rainbow
    colormap prism
    cR = (1:size(qR,2))/size(qR,2);
    cStart = cR(1)*ones(1,size(qStart,2));
    cEnd = cR(end)*ones(1,size(qStart,2));
    cL = fliplr(cR);
    c = [cStart cR cEnd cL cStart(1)];
    h.fill = patch(qroad(1,:),qroad(2,:),c,'linestyle','-','linewidth',2);
else
    h.fill = patch(qroad(1,:),qroad(2,:),'y','linestyle','-','linewidth',2);
end
% centerline
h.center = plot(q(1,:),q(2,:),'-','linewidth',1,'color',0.5*[1 1 1]);
% hashes
h.hash = [];
for i=1:10:size(q,2)
    x = q(1,i);
    y = q(2,i);
    theta = q(3,i)+pi/2;
    r = road.roadwidth/2;
    h.hash(:,end+1) = plot([x-r*cos(theta) x+r*cos(theta)],[y-r*sin(theta) y+r*sin(theta)],'-','linewidth',1,'color',0.85*[1 1 1]);
end
end

function robotfig = DrawRobot(robotfig,robot,alpha)
if isempty(robotfig)
    % - links
    robotfig.chassis = patch('Vertices',robot.chassis.p_in0','Faces',robot.chassis.faces,'FaceVertexCData',robot.chassis.colors,'FaceColor','flat',...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6);
	robotfig.wheel2 = patch('Vertices',robot.wheel2.p_in0','Faces',robot.wheel2.faces,'FaceVertexCData',robot.wheel2.colors,'FaceColor','flat',...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6,'linewidth',0.01);
	robotfig.wheel3 = patch('Vertices',robot.wheel3.p_in0','Faces',robot.wheel3.faces,'FaceVertexCData',robot.wheel3.colors,'FaceColor','flat',...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6,'linewidth',0.01);
else
    set(robotfig.chassis,'vertices',robot.chassis.p_in0');
    set(robotfig.wheel2,'vertices',robot.wheel2.p_in0');
    set(robotfig.wheel3,'vertices',robot.wheel3.p_in0');
end
end

function R = RX(h)
R = [1 0 0;
     0 cos(h) -sin(h);
     0 sin(h) cos(h)];
end
 
function R = RY(h)
R = [cos(h) 0 sin(h);
     0 1 0;
     -sin(h) 0 cos(h)];
end
 
function R = RZ(h)
R = [cos(h) -sin(h) 0;
     sin(h) cos(h) 0;
     0 0 1];
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
