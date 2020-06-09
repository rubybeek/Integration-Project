close all; clear all; clc

% include tclab.m for initialization
tclab;

disp('Test Heater 1')
disp('LED Indicates Temperature')

figure(1)
t1s = [];
t2s = [];
h1s = [];
h2s = [];
% initial heater values
ht1 = 0;
ht2 = 0;
h1(ht1);
h2(ht2);

for i = 1:3599
    tic;
    if i==10
        disp('Turn on heater 1')
        ht2 = 10;
        h1(ht1);
        h2(ht2);
    end
    if i==400
        disp('Turn on heater 1')
        ht2 = 50;
        h1(ht1);
        h2(ht2);
    end
    if i==450
        disp('Turn on heater 1')
        ht1 = 20;
        h1(ht1);
        h2(ht2);
    end
    if i==550
        disp('Turn on heater 2')
        ht2 = 15;
        h1(ht1);
        h2(ht2);
    end
    if i==850
        disp('Turn on heater 1')
        ht1 = 80;
        h1(ht1);
        h2(ht2);
    end
    if i==900
        disp('Turn on heater 1')
        ht1 = 25;
        h1(ht1);
        h2(ht2);
    end
    if i==1250
        disp('Turn on heater 2')
        ht2 = 70;
        h1(ht1);
        h2(ht2);
    end
    if i==1300
        disp('Turn on heater 2')
        ht2 = 10;
        h1(ht1);
        h2(ht2);
    end
    if i==1800
        disp('Turn on heater 1 & 2')
        ht1 = 40;
        ht2 = 40;
        h1(ht1);
        h2(ht2);
    end
    if i==2100
        disp('Turn on heater 1 & 2')
        ht1 = 15;
        ht2 = 0;
        h1(ht1);
        h2(ht2);
    end
     if i==2300
        disp('Turn on heater 1 & 2')
        ht1 = 0;
        ht2 = 20;
        h1(ht1);
        h2(ht2);
     end
     if i==2500
        disp('Turn on heater 1')
        ht1 = 40;
        h1(ht1);
        h2(ht2);
     end
     if i==2750
        disp('Turn on heater 2')
        ht1 = 50;
        ht2 = 5;
        h1(ht1);
        h2(ht2);
     end
     if i==2900
        disp('Turn on heater 1')
        ht1 = 10;
        h1(ht1);
        h2(ht2);
    end
    if i==3150
        disp('Turn on heater 2')
        ht2 = 40;
        ht1 = 20;
        h1(ht1);
        h2(ht2);
    end
    if i==3200
        disp('Turn on heater 1')
        ht2 = 10;
        h1(ht1);
        h2(ht2);
    end
    if i==3400
        disp('Turn on heater 1 & 2')
        ht1 = 30;
        ht2 = 40;
        h1(ht1);
        h2(ht2);
    end
    if i==3599
        disp('Turn off heaters')
        ht1 = 0;
        ht2 = 0;
        h1(ht1);
        h2(ht2);
    end
    
    % read temperatures
    t1 = T1C();
    t2 = T2C();

    % LED brightness
    brightness1 = (t1 - 30)/50.0;  % <30degC off, >100degC full brightness
    brightness2 = (t2 - 30)/50.0;  % <30degC off, >100degC full brightness
    brightness = max(brightness1,brightness2);
    brightness = max(0,min(1,brightness)); % limit 0-1
    led(brightness);
    
    % plot heater and temperature data
    h1s = [h1s,ht1];
    h2s = [h2s,ht2];
    t1s = [t1s,t1];
    t2s = [t2s,t2];
    n = length(t1s);
    time = linspace(0,n+1,n);
    
    clf
    subplot(2,1,1)
    plot(time,t1s,'b.','MarkerSize',10);
    hold on
    plot(time,t2s,'r.','MarkerSize',10);
    ylabel('Temperature (degC)')
    legend('Temperature 1','Temperature 2','Location','NorthWest')
    
    subplot(2,1,2)
    plot(time,h1s,'b-','LineWidth',2);
    hold on
    plot(time,h2s,'r--','LineWidth',2);
    ylabel('Heater (0-5.5 V)')
    xlabel('Time (sec)')
    legend('Heater 1','Heater 2','Location','NorthWest')
    drawnow;
    t = toc;
    pause(max(0.01,1.0-t))
end

disp('Turn off heaters')
h1(0);
h2(0);

disp('Heater Test Complete')

data = [linspace(1,length(t1s),length(t1s))' t1s' t2s' h1s' h2s'];
