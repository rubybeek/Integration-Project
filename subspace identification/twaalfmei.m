clearvars

load('data.1.mat')
datasmooth = smoothdata(data(:,1:2),'sgolay',60);

figure
plot(datasmooth(:,1:2))
hold on
plot(data(:,1:2))


dataset = iddata(datasmooth(:,1:2),data(:,3:4),1);
dataset.InputName  = {'Input Sensor 1';'Input Sensor 2'};
dataset.OutputName = {'Output Sensor 1';'Output Sensor 2'};

%%
MIMO = ssest(dataset,2);

figure(1)
compare(dataset,MIMO);
title('Subspace identification using ssest')


%% 
MIMO2 = n4sid(dataset,2);

figure(2)
compare(dataset,MIMO2);
title('Subspace identification using N4SID')


