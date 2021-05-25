function [d,obj,c,angle,aperture_size]=test_primitive_fitting_2(pcl2)
d=zeros(1,3);
c=0;
%% Remove table plane
maxDistance = 0.005; %Maximum point-to-plane distance [m]
sampleIndices = (1:pcl2.Count)'; %Indices of the points in ROI


% Find the principal plane to extract it from the point cloud
[ground,~,object_idx] = pcfitplane(pcl2,maxDistance,'SampleIndices',sampleIndices,'MaxNumTrials',10000);
% plot(plane)
%Save the remaining point cloud (Only keep the object in the scene)
object_pcl = select(pcl2,object_idx);


%% Remove noise from the point cloud
minDistance = 0.01;
[labels,numClusters] = pcsegdist(object_pcl,minDistance);
% pcshow(object_pcl.Location,labels)
% colormap(hsv(numClusters))
for j=1:numClusters
    cluster=find(labels==j);
    
    if size(cluster,1)/object_pcl.Count>0.3
        f_object_pcl=select(object_pcl,cluster);
        break
    end
end
% figure
% pcshow(f_object_pcl)
% hold on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

%% Cylinder or sphere fitting
maxDistance = 0.003; %Maximum point-to-cylinder distance [m]
maxDistance2 = 0.001; %Maximum point-to-sphere distance [m]
sampleIndices = (1:f_object_pcl.Count)'; %Indices of the points in ROI
acc_cyl=0;%best percentage of cylinder inliers
acc_ball=0;%best percentage of sphere inliers


%Repeat the process 20 time and keep the best solution
for i=1:20
    %Detect the cylinder in the point cloud and extract it
    [cyl_model_2, cyl_inliers] = pcfitcylinder(f_object_pcl,maxDistance,'SampleIndices',sampleIndices,'MaxNumTrials',10000);
    [ball_model_2, ball_inliers] = pcfitsphere(f_object_pcl,maxDistance2,'SampleIndices',sampleIndices,'MaxNumTrials',10000);
    
    %Percentage of inliers
    acc2_cyl=size(cyl_inliers)/size(f_object_pcl.Location)*100;
    acc2_ball=size(ball_inliers)/size(f_object_pcl.Location)*100;

    
    %If the percentage of inliers if higher than the best solution found
    %before, we save the new model
    if acc2_cyl>acc_cyl
        cyl_model=cyl_model_2;
        acc_cyl=acc2_cyl;
    end
    if acc2_ball>acc_ball
        ball_model=ball_model_2;
        acc_ball=acc2_ball;
    end
    
    if acc2_cyl<40 && acc2_ball<40
        break
    end
    
end


%% Find box model
acc_box=0;%best percentage of box inliers

%Repeat the process 20 time and keep the best solution
for i=1:20
    %Find box model
    [dimension2,angle2,normal2,acc_box_2,centre2,aperture2] = pcfitbox_visualization_2(f_object_pcl,ground.Normal);
    
    
    
    %If this iteration is a better fit, keep the new one
    if acc_box_2>acc_box
        d=dimension2;
        normal=normal2;
        acc_box=acc_box_2;
        centre=centre2;
        angle=angle2;
        aperture=aperture2;
    end
    
    %If the percentage of inliers is <70, then we consider that there isn't a box
    if acc_box<70
        break
    end
end




%% Model decision

if acc_cyl > acc_ball && acc_cyl>acc_box
    d(1)=cyl_model.Radius*2*100;
    d(2)=cyl_model.Height*100+1;
    d(3)=0;
    obj=1;
    c=acc_cyl;
    aperture_size=d(1)+2;
    %         plot(cyl_model)
    %         xor=[cyl_model.Center(1)-cyl_model.Orientation(1)...
    %             cyl_model.Center(1)+cyl_model.Orientation(1)];
    %         yor=[cyl_model.Center(2)-cyl_model.Orientation(2)...
    %             cyl_model.Center(2)+cyl_model.Orientation(2)];
    %         zor=[cyl_model.Center(3)-cyl_model.Orientation(3)...
    %             cyl_model.Center(3)+cyl_model.Orientation(3)];
    %         line(xor,yor,zor,'color','red')
    if cyl_model.Orientation(2)<0
        cyl_ang=-cyl_model.Orientation;
    else
        cyl_ang=cyl_model.Orientation;
    end
    angle = atan2d(norm(cross([cyl_ang(1) cyl_ang(2) 0],[0 1 0])),dot([cyl_ang(1) cyl_ang(2) 0],[0 1 0]));
    
    
elseif acc_ball > acc_cyl && acc_ball>acc_box
    d(1)=ball_model.Radius*2*100;
    d(2)=0;
    d(3)=0;
    obj=2;
    c=acc_ball;
    angle=0;
    aperture_size=d(1)+2;
    %plot(ball_model)
    
    
elseif acc_box > acc_cyl && acc_box>acc_ball
    obj=3;
    c=acc_box;
    aperture_size=aperture+2;
    if normal(1,3)>0
        normal(1,:)=-normal(1,:);
    end
    if normal(2,3)>0
        normal(2,:)=-normal(2,:);
    end
    if normal(3,3)>0
        normal(3,:)=-normal(3,:);
    end
    centre=ones(3,1)*centre;
%     xor=[centre(:,1) centre(:,1)+0.2*normal(:,1)];
%     yor=[centre(:,2) centre(:,2)+0.2*normal(:,2)];
%     zor=[centre(:,3) centre(:,3)+0.2*normal(:,3)];
%     line(xor(1,:),yor(1,:),zor(1,:),'color','red')
%     line(xor(2,:),yor(2,:),zor(2,:),'color','blue')
%     line(xor(3,:),yor(3,:),zor(3,:),'color','green')
%     scatter3(centre(:,1),centre(:,2),centre(:,3))
    
    

end

end