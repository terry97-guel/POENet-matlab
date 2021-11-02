function adstar = ad_star(wrench)
if(length(wrench)~=6)
    error('ERROR: ad_star() takes 6-dim wrench as input!');
    adstar = nan;
end

tau = wrench(1:3);
f   = wrench(4:6);
adstar = zeros(6,6);
adstar(1:3,1:3) = skew(tau);
adstar(1:3,4:6) = skew(f);
adstar(4:6,1:3) = skew(f);
end