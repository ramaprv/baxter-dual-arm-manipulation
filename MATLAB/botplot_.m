clc;
clear all;
close all;
%%
To obtain a workspace plot uncomment lines 19, 40 and 42, comment lines 27, 28 and 30
To obtain simulation of workspace plot, comment lines 19, 40 and 42, uncomment lines 28, 29 and 31
%%
L1 = Link('d', 270.35, 'a',69, 'alpha', -pi/2)
L2 = Link('d', 0, 'a',0, 'alpha', -pi/2, 'offset',-pi/2)
L3 = Link('d', 364.35, 'a',-69, 'alpha', pi/2)
L4 = Link('d', 0, 'a',0, 'alpha', -pi/2)
L5 = Link('d', 374.29, 'a',-10, 'alpha', pi/2)
L6 = Link('d', 0, 'a',0, 'alpha', -pi/2)
L7 = Link('d', 229.5, 'a',0, 'alpha', 0)

bot = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'my robot')

bot.n
hold on
for i=-97.494*pi/180:pi/2:97.494*pi/180
    for j=-123*pi/180:pi/2:pi/3
        for k=-174.987*pi/180:pi/2:174.987*pi/180
           for i1 = -2.864*pi/180:pi/2:150*pi/180
                for j1=-175.25*pi/180:pi/2:175.25*pi/180
                    for k1=-90*pi/180:pi/2:120*pi/180
                        for k2=-175.25*pi/180:pi/2:175.25*pi/180 
        p = bot.fkine([i  j  k 11 j1 k1 k2]).t;
        %bot.plot([i  j  k 11 j1 k1 k2]);
        %hold on
        scatter3(p(1),p(2),p(3));
        %drawnow
                        end
                    end
                end
           end
        end
    end
end

hold off;

bot.plot([0 0 0 0 0 0 0])