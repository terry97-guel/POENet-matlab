function avgSE3 = averageSE3(SE3_cell)
nData = length(SE3_cell);
avgSE3 = SE3_cell{1};

% initial
grad = zeros(4,4);
for i=1:nData
    grad = grad + logm(SE3_cell{i} / avgSE3);
end
avgSE3 = expm(grad/nData) * avgSE3;

% optimization
while(norm(grad)>1e-6)
    grad = zeros(4,4);
    for i=1:nData
        grad = grad + logm(SE3_cell{i} / avgSE3);
    end
    avgSE3 = expm(grad/nData) * avgSE3;
end
end