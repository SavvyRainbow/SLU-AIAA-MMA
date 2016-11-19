clear all
close all
clc

% Mission one Maritime survalence WPL=3000
% Mission two WPL=6390
% Mission three Cargo WPL=5000
% Mission four STOL WPL=3000

Wcrew=693; %Crew Weight in Lbs
Wpl=[3000,6390,5000,4620]; %Payload Weight
Wtoguess=1:10:100000; %take off weight guess

%Regession Factors (A&B) for Take off weight calculation
A=.1703;
B=1.0083; 
Mfres=.05; %Reserve Fuel Fraction

% Endurace Mission Constants
E=12;
np_e=.77;
cp_e=(.5+.7)/2;
LD_e=(13+15)/2;
cj_e=(.4+.6)/2;
V_e=150*1.15078;

%Range Mission Constants
R=[1000*1.15078,500*1.15078,250*1.125078];
np_r=.82;
cp_r=(.5+.7)/2;
LD_r=(10+12)/2;
cj_r=(.5+.9)/2;
V_r=250*1.15078;

%Life Coefficients
CLmax=1.2:.2:1.8; %Clean Config
CLmaxto=1.6:.2:2.2; %Takeoff Config
CLmaxl=1.8:.2:3.4; %Landing Config

%Takeoff Length
Sstol=1500;
Snorm=6000;
Sto=Snorm;

%Density Values
rho=.002378; %Sea Level
sigma=1; %Sea Level

%Wing Loading Sizing
WS=1:1:400;

%Landing to Takeoff Weight Fraction
WLWTO=.78;%(.95+.78)/2;

%% Calculate the Mission Fuel Fraction

%Phase 1: Engine Start and Warm-up
p1=.992;
%Phase 2: Taxi
p2=.990;
%Phase 3: Take-off
p3=.996;
%Phase 4: Climb
p4=.985;
%Phase 5: Cruise
p5=1:numel(R);
for n=1:numel(R)
    p5(n)=1/exp((R(n))/(375*(np_r/cp_r)*LD_r));
end
%Phase 6: Loiter
p6=1:numel(E);
for n=1:numel(E)
    p6(n)=1/exp((E(n))/(375*(1/V_e)*(np_e/cp_e)*LD_e));
end
%Phase 7: Descent
p7=.990;
%Phase 8: Landing, Taxi, and Shutdown
p8=.990;

%Mission Fues Fraction for Endurace Mission
Mff_e=p1*p2*p3*p4*p6*p7*p8;
%Mission Fuel Fraction for Range Mission
Mff_r=p1*p2*p3*p4*p5*p7*p8;

Mff=[Mff_e,Mff_r];

%% Calculate the Takeoff Weight for Each Mission (inputs Mff, Wtoguess, Mfres, Wpl, Wcrew, A, B)
for q=1:numel(Mff)
    for p=1:numel(Wtoguess)
        Wfused(p)=(1-Mff(q))*Wtoguess(p);
        Wfres(p)=Mfres*Wfused(p);
        Wf(p)=Wfused(p)+Wfres(p);
        Wfs(q,p)=Wf(p);
        Woetent(p)=Wtoguess(p)-Wf(p)-Wpl(q);
        WEtent(p)=Woetent(p)-Wcrew;
        WE(p)=10^((log10(Wtoguess(p))-A)/B);
        Wes(q,p)=WE(p);
        diff(q,p)=abs(WEtent(p)-WE(p));
    end
end

%% Find the average Take off Weight (Wto), Empty Weight(We) and We/Wto for each mission
for p=1:numel(Mff)
    y0=find((abs(diff(p,:))-0)<10);
    x0=Wtoguess(y0);
    z0=Wf(y0);
    WtoAvg=mean(x0);
    WfAvg=mean(z0);
    e0=Wes(y0);
    WeAvg=mean(e0);
    WeWto=WeAvg/WtoAvg;
    AvgWto(p)=WtoAvg;
    AvgWe(p)=WeAvg;
    AvgWf(p)=WfAvg;
    WeWtoAvg(p)=WeWto;
end

%Display the Take Off Weights
%fprintf('The Mission Weight is %.0f lbs\n',AvgWto)

%% 3.1 Stall Speed
% No Current Mission Reqs

%% 3.2 Takeoff Sizing
for p=1:numel(CLmaxto)
    for q=1:numel(WS)
        Top25=Sto/37.5*sigma;
        WP(p,q)=(Top25*CLmaxto(p))/WS(q);
    end
end

%% 3.3 Landing Distance Sizing
for q=1:numel(CLmaxl)
    WSl(q)=((.5*rho*( ( (sqrt(Sto/.3) /1.3) *1.688)^2))/WLWTO)*CLmaxl(q);
end
WfusedM=(1-Mff).*AvgWto;
Wl=AvgWe+Wpl+Wcrew+WfusedM*Mfres;
for p=1:numel(WSl)
    for q=1:numel(AvgWto)
        S(p,q)=Wl(q)/WSl(p);
    end
end

%% 3.4 Cruise Speed and Drag Polar Sizing
% Drag Polar Estimation
AR=12;
temp_correction=0.80; % for turbofans, but the value for turboprops is nowhere to be found
cf=.006;
a=-2.2218;
b=1;
c=.6295;
d=.6708;
Swet=10.^(c+d*(log10(AvgWto)));
f=10.^(a+b*(log10(Swet)));
Scl=S(1,:);
%CD0=f./Scl;

CD0=0.023;
e_clean=(0.80+0.85)/2;
dCD0_TO_flaps=(0.010+0.020)/2;
e_TO_flaps=(0.75+0.80)/2;
dCD0_land_flaps=(.055+0.075)/2;
e_land_flaps=(0.70+0.75)/2;
dCD0_geardown=(0.015+0.025)/2;

%k=1/(pi()*AR*e);
Kclean=1/(pi()*AR*e_clean);
Kto=1/(pi()*AR*e_TO_flaps);
Kl=1/(pi()*AR*e_clean);
Kbached=(Kto+Kl)/2;


CGR=[.012,0,.024,.012,.032,.021];
V2=[1.2,1.1,1.2,1.25,1.3,1.5];
CLto=max(CLmaxto);
CLadj=CLto./(V2.^2);

CD0toflaps=CD0+dCD0_TO_flaps;
CD0toflapsgear=CD0+dCD0_TO_flaps+dCD0_geardown;
CD0lflapsgear=CD0+dCD0_land_flaps+dCD0_geardown;
CD0ltoflaps=CD0+((dCD0_land_flaps+dCD0_TO_flaps)/2)+dCD0_geardown;

CD0adj=[CD0toflaps,CD0toflapsgear,CD0toflaps,CD0,CD0lflapsgear,CD0ltoflaps];
K=[Kto,Kto,Kto,Kclean,Kbached,Kl];
LDadj=CLadj./(CD0adj+(K.*CLadj.^2));

CGRP=(CGR+(1./(LDadj))).*CLadj.^(1/2);
for q=1:numel(CGRP)
    for p=1:numel(WS)
        WPclimb(q,p)=(18.97*sigma^(1/2))/(CGRP(q)*WS(p)^(1/2));
    end
end
% Combine all variables into one for plotting purposes


%% 3.6 Cruise SPeed Sizing
Ip=[1.7,1.02];
sigmacr=.78609;
for p=1:numel(WS)
    for q=1:numel(Ip)
        WPcr(p,q)=WS(p)/(Ip(q)*sigmacr);
    end
end


%% Plots
%Plot Takeoff Weight vs the Differnce in Empty Weight-Tentative Empty Weight
figure(1)
hold on
plot(Wtoguess,diff);grid on
legend('Maritime Survalence','Passenger','Cargo','STOL')
title('Takeoff Weight Sizing for each Aircraft')
xlabel('Takeoff Weight in Pounds')
ylabel('Difference Between esitmate and calulated values')
hold off

while(1)
        choice = menu('Do you Want to show the break down plots?', 'yes', 'no');
if choice==2 || choice==0
    break;
end

%Plot Power Loading vs Wing Loading For Climb
figure(2)
hold on
plot(WS,WPclimb);
grid on
legend('Stuff')
title('Climb Speed WS vs WP')
xlabel('Wing Loading (W/S) PSF')
ylabel('Power Loading (W/P) LB/HP')
hold off


%Plot Power Loading vs Wing Loading
figure(3)
hold on
plot(WS,WP);grid on
legend('CL=1.6','CL=1.8','CL=2.0','CL=2.2')
title('Wing Loaidng vs Power Loading')
xlabel('Wing Loading (W/S) PSF')
ylabel('Power Loading (P/W) LB/HP')
hold off

%Plot the wing loading for each landing condition for each CL
figure(4)
hold on
grid on
x=(WSl);
for p=1:length(x)
  plot([x(p) x(p)],[0 max(WP(:))])
end;
title('Landing Wing Loading')
xlabel('Wing Loading (W/S) PSF')
ylabel('Power Loading (W/P) LB/HP')

%Plot Power Loading vs Wing Loading for Cruise
figure(5)
hold on
plot(WS,WPcr);grid on
legend('Stuff')
title('Cruse Speed WS vs WP')
xlabel('Wing Loading (W/S) PSF')
ylabel('Power Loading (W/P) LB/HP')
hold off
kill=1;
if kill==1;
    break;
end
end

%The big Plot
%Plot Power Loading vs Wing Loading
figure(6)
hold on
plot(WS,WP);
x=(WSl);
for p=1:length(x)
  plot([x(p) x(p)],[0 max(WP(:))])
end;
plot(WS,WPclimb);
plot(WS,WPcr);
grid on
legend('CL=1.6','CL=1.8','CL=2.0','CL=2.2')
title('Wing Loaidng vs Power Loading')
xlabel('Wing Loading (W/S) PSF')
ylabel('Power Loading (W/P) LB/HP')
axis([1,max(WSl),1,100])
hold off

%% Analysis Based Choice

CLmaxl=input('What is the Landing CL?');
CLmaxto=input('What is the Takeoff CL?');
W=AvgWto;
WS=((.5*rho*( ( (sqrt(Sto/.3) /1.3) *1.688)^2))/WLWTO)*CLmaxl;
WP=(Top25*CLmaxto)/WS;
P=W/WP;
S=W/WS;

fprintf('The Mission Takeoff Weight is %.0f Lbs\n', W);
fprintf('The Mission Wing Loading is %.4f PSF\n', WS);
fprintf('The Mission Power Loading is is %.4f Lbs/HP\n', WP);
fprintf('The Mission Power Required is %.4f HP\n', P);
fprintf('The Mission Wing Area Required is %.4f FT^2\n', S);


figure(7)
hold on
subplot(2,1,1);plot(W,S)
grid on
title('Weight vs Wing Area required for each mission')
xlabel('Take Off Weight of the aircraft (lbs)');
ylabel('Wing Area Required (FT^2)');
hold on
subplot(2,1,2);plot(W,P)
grid on
title('Weight vs Power required for each mission');
xlabel('Take Off Weight of the aircraft (lbs)');
ylabel('Power Required (HP)');
hold off