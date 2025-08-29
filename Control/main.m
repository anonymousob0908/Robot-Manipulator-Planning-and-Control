clc
clear

thetamat=load('thetamat.mat');
thetamat=double(thetamat.q_opt_total);
T=load('tmat.mat');
T = T.t_total;
simin=timeseries
simin.time=T';
simin.data=[thetamat'];

simin1=timeseries
simin1.time=T';
simin1.data=[thetamat(1,:)'];

simin2=timeseries
simin2.time=T';
simin2.data=[thetamat(2,:)'];

simin3=timeseries
simin3.time=T';
simin3.data=[thetamat(3,:)'];

simin4=timeseries
simin4.time=T';
simin4.data=[thetamat(4,:)'];

simin5=timeseries
simin5.time=T';
simin5.data=[thetamat(5,:)'];

simin6=timeseries
simin6.time=T';
simin6.data=[thetamat(6,:)'];

simin7=timeseries
simin7.time=T';
simin7.data=[thetamat(7,:)'];

%% 
dthetamat=load('dthetamat.mat');
dthetamat=double(dthetamat.dthetamat{1});
simind1=timeseries
simind1.time=T';
simind1.data=[dthetamat(1,:)'];

simind2=timeseries
simind2.time=T';
simind2.data=[dthetamat(2,:)'];

simind3=timeseries
simind3.time=T';
simind3.data=[dthetamat(3,:)'];

simind4=timeseries
simind4.time=T';
simind4.data=[dthetamat(4,:)'];

simind5=timeseries
simind5.time=T';
simind5.data=[dthetamat(5,:)'];

simind6=timeseries
simind6.time=T';
simind6.data=[dthetamat(6,:)'];

simind7=timeseries
simind7.time=T';
simind7.data=[dthetamat(7,:)'];


