clear all
close all
clc

rob_diam    =       1;     %robot's size enlarged [m]
Np = 12+1;
onePeriod = [linspace(rob_diam/2,(8-rob_diam/2),2*Np),ones(1,Np)*(8-rob_diam/2)...
    ,linspace((8-rob_diam/2),rob_diam/2,2*Np),ones(1,Np)*rob_diam/2 ...
    ,linspace(rob_diam/2,(8-rob_diam/2),2*Np),ones(1,Np)*(8-rob_diam/2) ...
    ,linspace((8-rob_diam/2),rob_diam/2,2*Np),ones(1,Np)*rob_diam/2 ...
    ,linspace(rob_diam/2,(8-rob_diam/2),2*Np),ones(1,Np)*(8-rob_diam/2) ...
    ,linspace((8-rob_diam/2),rob_diam/2,2*Np),ones(1,Np)*rob_diam/2 ...
    ,linspace(rob_diam/2,(8-rob_diam/2),2*Np)];

t=[ones(1,2*Np)*rob_diam/2,linspace(rob_diam/2,1.67+rob_diam/2,Np),ones(1,2*Np)*(rob_diam/2+1.67)...
    ,linspace(rob_diam/2+1.67,rob_diam/2+1.67*2,Np),ones(1,2*Np)*(rob_diam/2+2*1.67) ...
    ,linspace(rob_diam/2+1.67*2,1.67*3+rob_diam/2,Np),ones(1,2*Np)*(rob_diam/2+3*1.67) ...
    ,linspace(rob_diam/2+3*1.67,1.67*4+rob_diam/2,Np),ones(1,2*Np)*(rob_diam/2+4*1.67) ...
    ,linspace(rob_diam/2+1.67*4,1.67*5+rob_diam/2,Np),ones(1,2*Np)*(rob_diam/2+1.67*5) ...
    ,linspace(rob_diam/2+1.67*5,10-rob_diam/2,Np),ones(1,2*Np)*(10-rob_diam/2)
    ];

plot(t, onePeriod, 'b*', 'LineWidth', 2);
path = [t',onePeriod'];
