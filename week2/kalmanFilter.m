function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
        if previous_t<0
            state = [x, y, 0, 0];
            param.P = 2 * eye(4);
            predictx = x;
            predicty = y;
            return;
        end
        param.Q = diag([0.2,0.2,1,1]);
        param.W = 0.01*eye(2);
        param.A = [1 0 0.330 0;
                   0 1 0 0.330;
                   0 0 1  0;
                   0 0 0  1];
        param.V = [0.330         0    0.15        0;
                       0    0.330         0    0.15;
                       0         0    0.330         0;
                       0         0         0    0.330];
        param.H = [1 0 0 0;
                   0 1 0 0];
        param.dt = 0.330;

    %%prediction step 
    Xk1 = param.A*state';
    Pk1 = param.A*param.P*param.A'+param.V*param.Q*param.V';
    
    %% kalman gain 
    K = Pk1*param.H'/(param.H*Pk1*param.H'+param.W);

    %% update step
    state = (Xk1 + K*([x;y] - param.H*Xk1))';
    param.P = param.P -K*param.H*param.P;
    % Predict 330ms into the future
    predictx = state(1) + state(3)*param.dt;
    predicty = state(2) + state(4)*param.dt;
end
