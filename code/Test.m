
%Testing Controller 1 and 2, to show possible failure and inefficieny
figure(1)
run MakeRoad.m % generate random road file
figure(2)
DesignProblem04('Controller1', 'datafile', 'data.mat') % run the controller
figure(3)
run MakeRoad.m % generate random road file
figure(4)
DesignProblem04('Controller2', 'datafile', 'data.mat')

%Testing controller 3 for 80% success
figure(5)
run MakeRoad.m % generate random road file
figure(6)
DesignProblem04('Controller3', 'datafile', 'data.mat') % run the controller
figure(7)
run MakeRoad.m % generate random road file
figure(8)
DesignProblem04('Controller3', 'datafile', 'data.mat')
figure(9)
run MakeRoad.m % generate random road file
figure(10)
DesignProblem04('Controller3', 'datafile', 'data.mat')
figure(11)
run MakeRoad.m % generate random road file
figure(12)
DesignProblem04('Controller3', 'datafile', 'data.mat')
figure(13)
run MakeRoad.m % generate random road file
figure(14)
DesignProblem04('Controller3', 'datafile', 'data.mat')
figure(15)
run MakeRoad.m % generate random road file
figure(16)
DesignProblem04('Controller3', 'datafile', 'data.mat')
figure(17)
run MakeRoad.m % generate random road file
figure(18)
DesignProblem04('Controller3', 'datafile', 'data.mat')
figure(19)
run MakeRoad.m % generate random road file
figure(20)
DesignProblem04('Controller3', 'datafile', 'data.mat')
