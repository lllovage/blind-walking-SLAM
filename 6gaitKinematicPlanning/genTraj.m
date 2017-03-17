function out = genTraj (constraints)
    % This function generates a trajectory. At least one initial and finel 
    % constraints in position required and higher order constraints always
    % inside the defined range by lower order ones. Pass the required constraints
    % at the input as a structure with the next fields:
    % -> type : Write here 'p' for position, 'v' for velocity,'a' for
    % acceleration or 'j' for jerk.
    % -> time : Time at which each constraint should be satisfied.
    % -> value : Value of the constraint. This must be:
    %           Position: Vector of dimension 3, the order of x,y,z is
    %           taken care of by yourself.
    %           Velocity: Vector of dimension 3, the order of x,y,z must
    %           correspond to that of position, if there are some.
    %           Acceleration: Vector of dimension 3, order of x,y,z must
    %           correspond to that of position and velocity, if there are
    %           some.
    %           Jerk: Vector of dimensison 3, order of x,y,z musst
    %           correspond to that of position, velocity and acceleration,
    %           if there are some.
    % Ts -> A sampling rate for the output simulation. Important depending
    % application.
    % OUTPUT: params is a structure containing the position, velocity and
    % acceleration parameters. A recapitulative of the times at which
    % constraints were imposed and the values of either position, velocity,
    % acceleration or jerk imposed by the user. Also, a simulation of the
    % trajectory is given in the output with the sampling rate chosen at
    % the input.
    %
    % NOTICE: Be careful with times given, if we are working with phasic
    % planning, then each initial time should be zero. If we are working
    % with continuous planning, then initial time will only be zero at the
    % beginning but in intermediate phases will always be more than zero
    % depending on the gait. Also, a  
    paramNum = size(constraints,2);
    
    M = inf(paramNum, paramNum);
    b = inf(paramNum,1);
    for i = 1: paramNum
        t = constraints(i).time;
        val = constraints(i).value;
        if strcmp(constraints(i).type,'p')
            for j = 0:paramNum-1
                M(i,j+1) = t^(paramNum-1-j);
                b(i) = val;
            end
        elseif strcmp(constraints(i).type,'v')
            mult = paramNum-1:-1:0;
            for j = 0:paramNum-1
                if paramNum-2-j < 0
                    mu = 0;
                else
                    mu = paramNum-2-j;
                end
                M(i,j+1) = mult(j+1) * t^(mu);
                b(i) = val;
            end
        elseif strcmp(constraints(i).type,'a')
            mult = paramNum-1:-1:0;
            mult = mult.*[mult(2:end),0];
            for j = 0:paramNum-1
                if paramNum-3-j < 0
                    mu = 0;
                else
                    mu = paramNum-3-j;
                end                
                M(i,j+1) = mult(j+1) * t^(mu);
                b(i) = val;
            end
        elseif strcmp(constraints.type,'j')
            mult = paramNum-1:-1:0;
            mult2 = mult.*[mult(2:end),0];
            mult = mult2.*[mult(3:end),0,0];
            for j = 0:paramNum-1
                if paramNum-4-j < 0
                    mu = 0;
                else
                    mu = paramNum-4-j;
                end
                M(i,j+1) = mult(j+1) * t^(mu);
                b(i) = val;
            end
        end
    end
    posParams = M\b;
    mult = paramNum-1:-1:0;
    velParams = mult.*posParams';
    velParams = [0,velParams(1:end-1)]'; 
    
    mult = paramNum-1:-1:0;
    mult = mult.*[mult(2:end),0];
    accParams = mult.*posParams';
    accParams = [0,0, accParams(1:end-2)]';
    % Not from top to down: 1, x, x^2, x^3, ... x^n 
    accParams = flipud(accParams);
    posParams = flipud(posParams);
    velParams = flipud(velParams);
    % Log constraints recapitulative
    out.time = [constraints(:).time];
    out.type = [constraints(:).type];
    out.value = [constraints(:).value];
    %Log simulation
    Ts = constraints(1).Ts;
    out.sim.t = min([out.time]):Ts:max([out.time]);
    
    for i = 1:size(out.sim.t,2)
         [out.sim.pos(i), tempParams] = solvePoly(posParams,out.sim.t(i),1);
         [out.sim.vel(i), tempParams] = solvePoly(posParams,out.sim.t(i),2);
         [out.sim.acc(i), tempParams] = solvePoly(posParams,out.sim.t(i),3);
    end
    % Log parameters
    out.posParams = posParams;
    out.velParams = velParams;
    out.accParams = accParams;
    out.Ts = Ts;
end