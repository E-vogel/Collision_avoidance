clear
close all

% Time setting
h = 0.1;
L = 50;

% Repeat Attempts Setting
n = 3000;
Avoider.Xstore = cell(n,1);
Chaser.Xstore = cell(n,1);
time_str = zeros(n,1);


% Each parameter was adjusted by genetic algorithm.
x_ans = [20.495398715092438   1.632917901997279  10.401344935843886];

% Repeat Attepmpts
Avoider.N = 1;
Avoider.r = 3;
Chaser.N = 20;

tic
for ii = 1:n
    ii
    [step,Avoider.Xstr,Chaser.Xstr] = collision_avoidance_tracking_fun(x_ans(1),x_ans(2),x_ans(3));
    Avoider.Xstore{ii} = Avoider.Xstr;
    Chaser.Xstore{ii} = Chaser.Xstr;
    time_str(ii) = step;
end
toc

% Select the simulation with the longest avoided time
[time_max,idx] = max(time_str);

disp("Maxtime is " + time_max)

% Video
video = VideoWriter('collision_avoidance_tracking.avi','Uncompressed AVI');
open(video)

fig = figure;
fig.Color = 'k';

% Plot
theta = linspace(0,2*pi,100);
Avoider.P = plot(Avoider.r*cos(theta)+Avoider.Xstore{idx}(:,1),Avoider.r*sin(theta)+Avoider.Xstore{idx}(:,2),'y','LineWidth',1.5);
hold on
Chaser.S = scatter(Chaser.Xstore{idx}(:,1),Chaser.Xstore{idx}(:,2),20,'w','filled');
Chaser.S.CData = ones(Chaser.N,3).*[1 0 0];
rectangle('Position',[-L -L 2*L 2*L],'EdgeColor','w')
daspect([1 1 1])
axis([-L L -L L])
axis off
yohaku

t = 0;
msg = "Time: " + t;
T = title(msg,'Interpreter','latex','FontSize',15,'Color','w');

for step = 1:time_max-1
    t = t + h;
    msg = "Time: " + t;
    T.String = msg;

    distance = zeros(Avoider.N,Chaser.N);
    for i = 1:Avoider.N
        for j = 1:Chaser.N
            distance(i,j) = norm(Avoider.Xstore{idx}(i,:,step) - Chaser.Xstore{idx}(j,:,step));
        end
    end
    
    for i = 1:Avoider.N
        for j = 1:Chaser.N
            if distance(i,j) < Avoider.Rang
                Chaser.S.CData(j,:) = [0.3 1 1];
            else
                Chaser.S.CData(j,:) = 'w';
            end
        end
    end
    Avoider.P.XData = Avoider.r*cos(theta) + Avoider.Xstore{idx}(:,1,step);
    Avoider.P.YData = Avoider.r*sin(theta) + Avoider.Xstore{idx}(:,2,step);
    Chaser.S.XData = Chaser.Xstore{idx}(:,1,step);
    Chaser.S.YData = Chaser.Xstore{idx}(:,2,step);
    frame = getframe(gcf);
    writeVideo(video,frame)
end
close(video)
