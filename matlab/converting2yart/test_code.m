%% addpath
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
addpath ../../../../Simulator/yet-another-robotics-toolbox/code/
addpath ../POENet_matlab/function

%% se3 & srodrigues
% se3 -> SE3
% compatilbilty test

S = rand(6,1);
SE3_original = expm(ToMatrix(S));
SE3 = srodrigues(S,1);

if norm(SE3_original-SE3) < 1e-10
    fprintf("compatibility is guaranteed \n" )
end

%% largeAdjoint % inv_x(t2x)

T = pr2t(rand(3,1),rpy2r(rand(3,1)));
X_original = largeAdjoint(T);
X = inv_x(t2x(T));

if norm(X_original-X) < 1e-10
    fprintf("compatibility is guaranteed \n" )
end

%% declare random revolute twist
s1 = [0,0,1,0,0,0]';

s2w = rand(3,1);
s2w = s2w/norm(s2w);
s2v = rand(3,1);
s2v = s2v-s2v'*s2w*s2w;
s2 = [s2w; s2v];

s3w = rand(3,1);
s3w = s3w/norm(s3w);
s3v = rand(3,1);
s3v = s3v-s3v'*s3w*s3w;
s3 = [s3w; s3v];

s = [s1,s2,s3];
%% 
q = rand(1,3);
p_zeros = zeros(3,3);
p_ = zeros(3,3);
poe = eye(4);

for i=(1:3)
    w = s(1:3,i);
    v = s(4:6,i);
    
    poe = poe * srodrigues(s(:,i),q(i));
    
    p_zeros(i,:) = skew(w) * v/norm(w)^2;
    p_temp = poe *[p_zeros(i,:),1]';
    p_(i,:) = p_temp(1:3);
end

n = 1;
p_zeros;
norm(p_(n,:)-p_(n+1,:))

%% 
ccc

s1 = rand(6,1);
% s1 = [0,0,1,0,0,0]';
% N = [s1(1:3)',s1(4:6)';
%     0,0,0,s1(1:3)'];
% N = [0,0,0,s1(1:3)'];
% N = [1,0,0,0,0,0];
% % nullN = null(N);
% e1 = nullN*rand(5,1);
% s2 = inv_x(t2x(srodrigues(e1, rand(1)))) * s1;
% s2 = pr2x(rand(3,1),rpy2r(rand(3,1))) * s1;
s2 = inv_x(t2x(srodrigues(rand(6,1),rand(1)))) * s1;
% s2 = rand(6,1);
a = (s1(1:3)' * s1(4:6)) / (s1(1:3)'*s1(1:3));
b = (s2(1:3)' * s2(4:6)) / (s2(1:3)'*s2(1:3));
abs(a - b) < 1e-6

%%
perpendicular = [0,0,0,1,0,0;
    0,0,0,0,0,0];
null(perpendicular)