function [ d_beta, d_yaw, w_c, w_g, dt ] = analysis_syn_imu_encoder( imu_f, enc_f, step)
% Nov.9 2016 David Z
%   Input: imu_f: imu file, enc_f: encoder file, 
%           step: every step imu measurement to synchronize
%   Output: d_beta: turn angle computed from encoder data 
%           d_yaw: turn angle computed from imu data  
%           w_c: angular velocity calculated from the encoder data
%           w_g: angular velocity calculated from imu 
%           t, dt [ms] = [dt_encoder; dt_imu] 

if nargin ~= 3
    step = 10; % 10 imu measurements 
    if nargin == 0
        imu_f = './case2/vn100_data.log'; 
        enc_f = './case2/encoder_output_case2.log';
    end
end

imu_d = load(imu_f); 
enc_d = load(enc_f); 

[imu_i, enc_i] = syn_time_stamps(imu_d, enc_d, step); 
t = imu_d(imu_i,1); 
dt_imu = 1000*(t-ones(size(t))*t(1));
t = enc_d(enc_i,1);
dt_enc = 1000*(t-ones(size(t))*t(1));
dt = [dt_enc; dt_imu];

d_beta = compute_beta(enc_d, enc_i);
d_yaw = compute_yaw(imu_d, imu_i);

w_c = compute_angular_velocity(d_beta, dt_enc)*1000;
w_g = compute_angular_velocity(d_yaw, dt_imu)*1000; 


dw = [0; w_g-w_c];
tt = dt_imu; 
plot(tt, [0; w_c], 'r*-');
grid on; 
hold on; 
plot(tt, [0; w_g], 'g*-');
plot(tt, dw,'b*-');
legend('Wc', 'Wg', 'Wg-Wc'); 
xlabel('Time (ms)'); 
ylabel(' Angular Velocity degree/s');
end

function v = compute_angular_velocity(angle, t)
    v = zeros(size(angle,1)-1,1); 
    for i=1:size(v,1)
        v(i) = angle(i)/(t(i+1)-t(i));
    end
end

function beta = compute_beta(enc_d, enc_i)
% 
%   d_alpha = dCount/128*360, how many degrees the motor rotates 
%   r = 0.028m , the radius of the rolling tip 
%   L = 1.39 m , the cane's length 
%   C = 26, reduction-gear ratio 
beta = zeros(size(enc_i));
r = 0.028; 
L = 1.39; 
C = 26; 
count = enc_d(:,2); 
    for i = 2:size(enc_i,1)
        dCount = count(enc_i(i)) - count(enc_i(i-1));
        d_alpha = (dCount/128)*90; % after test not 360, but 90
        beta(i) = (r*d_alpha)/(C*L);
    end
end

function yaw = compute_yaw(imu_d, imu_i)
    
y = imu_d(:,10); 
yaw = zeros(size(imu_i)); 
    for i = 2:size(imu_i,1)
        yaw(i) = y(imu_i(i)) - y(imu_i(i-1));
        if yaw(i) < -180
            yaw(i) = yaw(i) + 360;
        elseif yaw(i) > 180
            yaw(i) = yaw(i) - 360;
        end
    end
end

function [imu_i, enc_i] = syn_time_stamps(imu_d, enc_d, step)

iT = imu_d(:,1); 
eT = enc_d(:,1);

start_point = 0; 
if iT(1) < eT(1)
    start_point = iT(1); 
else
    start_point = eT(1);
end
iT = 1000*iT - ones(size(iT))*1000*start_point; 
eT = 1000*eT - ones(size(eT))*1000*start_point;
[is, es] = find_start_time(iT, eT); 
imu_i = [is]; 
enc_i = [es]; 

j = es;
for i=is+step:step:size(iT,1)
    j = find_matched_index(iT(i), eT, j); 
    if j > 0 
        imu_i = [imu_i; i];
        enc_i = [enc_i; j];
    else
        break;
    end
end
end

function cur_i = find_matched_index(qT, eT, cur_i)
    
for i=cur_i: size(eT,1)
    if eT(i) > qT
        if eT(i) - qT > qT - eT(i-1)
            cur_i = i - 1;
        else
            cur_i = i;
        end
        return;
    end
end

cur_i = -1;

end

function [is, es] = find_start_time(iT, eT)
    
is = 1; 
es = 1; 
if iT(is) < eT(es)
    for i=is:size(iT,1)
        if iT(i) > eT(es) 
            if iT(i) - eT(es) > eT(es) - iT(i-1) % 25 ms 
                is = i-1;
            else
                is = i;
            break;
            end
        end
    end
else
    for i=es:size(eT,1)
        if eT(i) > iT(is)
            if eT(i) - iT(is) > eT(es) - iT(i-1) % 25 ms
                es = i -1;
            else
                es = i;
            end
            break;
        end
    end
end

end