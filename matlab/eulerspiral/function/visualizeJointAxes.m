function visualizeJointAxes(screw, len, viewAxis, color)
nJoint = size(screw,2);
for i=1:nJoint
    w = screw(1:3,i);
    v = screw(4:6,i);
    q = skew(w) * v;
%     if(i==1)
%         q = q - 0.2*w;
%     elseif(i==2)
%         q = q + w;
%     elseif(i==3)
%         q = q + 0.5*w;
%     elseif(i==6 || i==9)
%         q = q + 0.0*w;
%     elseif(i==7 || i==8)
%         q = q + 1.0*w;    
%     elseif(i==nJoint)
%         q = q + 1.3*w;
%     end
    t = -len:0.1:len;
    jointaxis = q + w * t;
    plot3(jointaxis(1,:), jointaxis(2,:), jointaxis(3,:), 'LineWidth', 1, 'Color', color(i,:), 'LineWidth', 2)
    view(viewAxis)
    hold on
end
% legend('joint 1','joint 2','joint 3', 'joint 4', 'joint 5')
hold off
end