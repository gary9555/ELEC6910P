function s_des = trajectory_generator(t, path, h)

% 1-x
% 2-y
% 3-z
% 4-xdot
% 5-ydot
% 6-zdot
% 7-10 quat
% 11-p
% 12-q
% 13-r

persistent coef;
time_tol = 25;

if nargin > 1 % pre-process can be done here (given waypoints)
    
    num_seg = length(path)-1;
    seg_len = zeros(num_seg,1);
    T = zeros(num_seg+1,1);
    % compute segment lengths
    for n=1:num_seg
        seg_len(n) = norm(path(n+1,:) - path(n,:));
    end
    
    len_sum = sum(seg_len);
    
    % Time constraints
    for m = 1:num_seg
        T(m+1) = T(m) + time_tol * seg_len(m) / len_sum;
    end
    
    % main loop for construction the problem
    Q = []; % cost matrix
    A = []; % contraint matrix
    bx = []; % constraint vector
    by = [];
    bz = [];
    for i = 1:num_seg
        Qk = zeros(8,8);
        seg_time = T(i+1)-T(i);
        
        % construct the cost matrix
        for j = 1:8
            for k = 1:8
                if j<5 || k<5
                    Qk(j,k) = 0;
                else
                    Qk(j,k) = (j-1)*(j-2)*(j-3)*(j-4)*(k-1)*(k-2)*(k-3)*(k-4)/(j+k-9)*seg_time^(j+k-9);
                end               
            end            
        end
        Q = blkdiag(Q,Qk);
        
        % construct the contraint matrix
        A_endpt = [1, zeros(1,15); ...    
                    1, seg_time, seg_time^2, seg_time^3, seg_time^4, seg_time^5, seg_time^6, seg_time^7 zeros(1,8)];
        A_pos = [A_endpt(2,1:8) -1 zeros(1,7)];
        A_spd = [0, 1, 2*seg_time, 3*seg_time^2, 4*seg_time^3, 5*seg_time^4, 6*seg_time^5, 7*seg_time^6, 0, -1, zeros(1,6)];
        A_acc = [0, 0 , 2 , 6*seg_time, 12*seg_time^2 , 20*seg_time^3, 30*seg_time^4, 42*seg_time*5, 0, 0, -2, zeros(1,5)];
        
        
        % concatenate constraint matrices
        if i== 1 % first segment need to specify starting speed
            A_start_spd = [0 1 zeros(1,14)];
            A = [[A_endpt;A_start_spd;A_pos;A_spd;A_acc] zeros(6,8*(num_seg-1))];
            dx = [path(i,1); path(i+1,1); zeros(4,1)];
            dy = [path(i,2); path(i+1,2); zeros(4,1)];
            dz = [path(i,3); path(i+1,3); zeros(4,1)];
        elseif i == num_seg % last segment also need to specify ending speed
            A_end_spd = [0, 1, 2*seg_time, 3*seg_time^2, 4*seg_time^3, 5*seg_time^4, 6*seg_time^5, 7*seg_time^6, zeros(1,8)];
            A = [A;...
                 zeros(3,8*(num_seg-1))  [A_endpt;A_end_spd]];
            dx = [path(i,1); path(i+1,1); 0];
            dy = [path(i,2); path(i+1,2); 0];
            dz = [path(i,3); path(i+1,3); 0];
        else
            A = [A;...
                 zeros(5, (i-1)*8)   [A_endpt;A_pos;A_spd;A_acc]  zeros(5,8*(num_seg-i))];
            dx = [path(i,1); path(i+1,1); 0; 0; 0];
            dy = [path(i,2); path(i+1,2); 0; 0; 0];
            dz = [path(i,3); path(i+1,3); 0; 0; 0];
        end
        
        bx = [bx;dx];
        by = [by;dy];
        bz = [bz;dz];
        
    end
    
    save('A.mat','A');
    save('b.mat','bx');
    
    f = zeros(length(A),1);
    
    X = quadprog(Q, f, [], [], A, bx);
    Y = quadprog(Q, f, [], [], A, by);
    Z = quadprog(Q, f, [], [], A, bz);
    
    coef = [X Y Z];
    size(coef);

else % output desired trajectory here (given time)

    % find which segment t should be in
    
    
end

end


