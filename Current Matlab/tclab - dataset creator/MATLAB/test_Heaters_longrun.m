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
        ht1 = 30;
        h1(ht1);
        h2(ht2);
    end
    if i==450
        disp('Turn on heater 1')
        ht1 = 75;
        h1(ht1);
        h2(ht2);
    end
    if i==550
        disp('Turn on heater 2')
        ht2 = 40;
        h1(ht1);
        h2(ht2);
    end
    if i==950
        disp('Turn on heater 1')
        ht1 = 100;
        h1(ht1);
        h2(ht2);
    end
    if i==1250
        disp('Turn on heater 2')
        ht2 = 80;
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
        ht1 = 60;
        ht2 = 80;
        h1(ht1);
        h2(ht2);
    end
     if i==2400
        disp('Turn on heater 1 & 2')
        ht1 = 0;
        ht2 = 100;
        h1(ht1);
        h2(ht2);
     end
     if i==2800
        disp('Turn on heater 1')
        ht1 = 100;
        h1(ht1);
        h2(ht2);
     end
      if i==2900
        disp('Turn on heater 2')
        ht2 = 50;
        h1(ht1);
        h2(ht2);
      end
    if i==3100
        disp('Turn on heater 1')
        ht1 = 50;
        h1(ht1);
        h2(ht2);
    end
    if i==3200
        disp('Turn on heater 2')
        ht2 = 100;
        h1(ht1);
        h2(ht2);
    end
    if i==3300
        disp('Turn on heater 1')
        ht1 = 10;
        h1(ht1);
        h2(ht2);
    end
    if i==3500
        disp('Turn on heater 1 & 2')
        ht1 = 50;
        ht2 = 60;
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
