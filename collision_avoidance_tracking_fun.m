function [step,Avoider_Xstr_OUT,Chaser_Xstr_OUT] = collision_avoidance_tracking_fun(AR,Ak,CR)
% Time setting
h = 0.1;
L = 50;

% Initialization
Avoider.N = 1; % The number of Avoiders
Avoider.X = L*(rand(Avoider.N,2)*2 - 1)/10;
Avoider.Xstr = zeros(Avoider.N,2,10000);
Avoider.Xstr(:,:,1) = Avoider.X;
Avoider.V = zeros(Avoider.N,2);
Avoider.r = 3; % Avoider Radius
Avoider.Rang = AR; % For Chaser within this threshold, the avoidance action
Avoider.k = Ak; % Avoidance Strength

Chaser.N = 20; % The number of Chasers
Chaser.X = L*(2*rand(Chaser.N,2) - 1);
Chaser.Xstr = zeros(Chaser.N,2,10000);
Chaser.Xstr(:,:,1) = Chaser.X;
Chaser.V = 8*(2*rand(Chaser.N,2) - 1);
Chaser.Rang = CR; % Tracks Avoider within a threshold
Chaser.k = 8; % Tracking Strength

t = 0;
step = 0;

while 1
    step = step + 1;
    t = t + h;
    
    % Calculate Distance
    distance = zeros(Avoider.N,Chaser.N);
    for i = 1:Avoider.N
        for j = 1:Chaser.N
            distance(i,j) = norm(Avoider.X(i,:) - Chaser.X(j,:));
        end
    end
    
    % Termination Condition
    if any(distance < Avoider.r) || t > 5000
        break
    end
    
    % Collision Avoidance
    for i = 1:Avoider.N
        for j = 1:Chaser.N
            if distance(i,j) < Avoider.Rang
                vector_dif = Avoider.X(i,:) - Chaser.X(j,:);
                Dot = dot(Chaser.V(j,:),vector_dif);
                Cross = Chaser.V(j,1)*vector_dif(2) - Chaser.V(j,2)*vector_dif(1);
                if Dot >= 0
                    theta_avoidance = sign(Cross)*pi/2;
                elseif Dot < 0
                    theta_avoidance = 0;
                end
                R = [cos(theta_avoidance) -sin(theta_avoidance);
                     sin(theta_avoidance)  cos(theta_avoidance)];
                Avoider.V(i,:) = Avoider.V(i,:) + Avoider.k*Chaser.V(j,:)/(distance(i,j) - Avoider.r*0.9)^2 * R.';
            end
        end
    end
    Avoider.V = 0.9*Avoider.V; % deceleration

    % Tracking
    for i = 1:Avoider.N
        for j = 1:Chaser.N
            if distance(i,j) < Chaser.Rang
                Chaser.V(j,:) = Chaser.V(j,:) - Chaser.k*(Chaser.X(j,:) - Avoider.X(i,:))/distance(i,j)^2;
            end
        end
    end

    % Wall boundary
    Avoider.V = Avoider.V.*(2*(abs(Avoider.X) + Avoider.r < L) - 1);
    Chaser.V = Chaser.V.*(2*(abs(Chaser.X) < L) - 1);

    Avoider.X(abs(Avoider.X) + Avoider.r > L) = (L - Avoider.r)*sign(Avoider.X(abs(Avoider.X) + Avoider.r > L));
    Chaser.X(abs(Chaser.X) > L) = L*sign(Chaser.X(abs(Chaser.X) > L));
    
    % Update
    Avoider.X = Avoider.X + Avoider.V*h;
    Chaser.X = Chaser.X + Chaser.V*h;

    Avoider.Xstr(:,:,step) = Avoider.X;
    Chaser.Xstr(:,:,step) = Chaser.X;    
end

Avoider_Xstr_OUT = Avoider.Xstr;
Chaser_Xstr_OUT = Chaser.Xstr;

