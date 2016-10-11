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

persistent coef  T;
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
            A = [[A_endpt;A_start_spd;A_spd;A_acc] zeros(5,8*(num_seg-2))]; %A_pos
            dx = [path(i,1); path(i+1,1); zeros(3,1)];
            dy = [path(i,2); path(i+1,2); zeros(3,1)];
            dz = [path(i,3); path(i+1,3); zeros(3,1)];
        elseif i == num_seg % last segment also need to specify ending speed
            A_endpt = [1, zeros(1,7); ...    
                    1, seg_time, seg_time^2, seg_time^3, seg_time^4, seg_time^5, seg_time^6, seg_time^7];
            A_end_spd = [0, 1, 2*seg_time, 3*seg_time^2, 4*seg_time^3, 5*seg_time^4, 6*seg_time^5, 7*seg_time^6];
            
            A = [A;...
                 zeros(3,8*(num_seg-1))  [A_endpt;A_end_spd]];
            dx = [path(i,1); path(i+1,1); 0];
            dy = [path(i,2); path(i+1,2); 0];
            dz = [path(i,3); path(i+1,3); 0];
        else
            A = [A;...
                 zeros(4, (i-1)*8)   [A_endpt;A_spd;A_acc]  zeros(4,8*(num_seg-i-1))]; %A_pos
            dx = [path(i,1); path(i+1,1); 0; 0];
            dy = [path(i,2); path(i+1,2); 0; 0];
            dz = [path(i,3); path(i+1,3); 0; 0];
        end
        
        bx = [bx;dx];
        by = [by;dy];
        bz = [bz;dz];
        
    end
    
    save('A.mat','A');
    save('b.mat','bx');
    
    f = zeros(length(Q),1);
    
    X = quadprog(Q, f, [], [], A, bx);
    Y = quadprog(Q, f, [], [], A, by);
    Z = quadprog(Q, f, [], [], A, bz);
    
    coef = [X Y Z];
    
    

else % output desired trajectory here (given time)
    
    s_des = zeros(13,1);
    % find which segment t should be in
    seg = 1;
    for i = 2:length(T)
       if t<=T(i)
           seg = i-1;
           break;
       end           
    end

    poly = coef(8*seg-7:8*seg,:);
    time = t - T(seg);
    
    n = 0:7;
    s_des(1:3) = time.^n*poly;
    s_des(4:6) = [1 2*time 3*time^2 4*time^3 5*time^4 6*time^5 7*time^6]*poly(2:end,:);
    s_des(7:10) = R_to_quaternion(RPYtoRot_ZXY(0,0,0));
    s_des(11:13) = [0 0 0]';
%     
end

end


