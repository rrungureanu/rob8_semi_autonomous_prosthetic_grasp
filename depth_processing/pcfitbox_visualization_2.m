function [dimensions,angle,normals,acc_box,centre,aperture] = pcfitbox_visualization_2(f_object_pcl,ground)
color=["red", "blue", "green"]; %Colours of the planes

box_pcl=f_object_pcl;
normals=zeros(3); %Normal vectors of principal planes
plane_vertexs=zeros(12,3); %Vertex points of the box
f_dim=zeros(3,2); %Face dimensions
dim=zeros(6,1);
inNum=0; %Number of inlier points
k=0;

for i=1:3
    
    maxDistance = 0.005; %Maximum point-to-plane distance [m]
    sampleIndices = (1:box_pcl.Count)'; %Indices of the points in ROI
    
    % Find the principal plane to extract it from the point cloud
    [plane,inliers,outliers] = pcfitplane(box_pcl,maxDistance,'SampleIndices',sampleIndices,'MaxNumTrials',10000);
    %plot(plane,'Color',color(i))
    inNum=inNum+size(inliers,1); %accumulated number of inliers
    acc_box=inNum/size(f_object_pcl.Location,1)*100;%Percentage of inliers
    face_pcl = select(box_pcl,inliers);% Point cloud of the face
    box_pcl = select(box_pcl,outliers);% Remaining point cloud after removing inliers
    
    normals(i,:)=plane.Normal;
    
    [~,x1]=min(face_pcl.Location(:,1));
    [~,x2]=max(face_pcl.Location(:,1));
    [~,y1]=min(face_pcl.Location(:,2));
    [~,y2]=max(face_pcl.Location(:,2));
    plane_vertexs((1+4*(i-1)):(4+4*(i-1)),:)=[face_pcl.Location(x1,:);...
        face_pcl.Location(x2,:);...
        face_pcl.Location(y1,:);...
        face_pcl.Location(y2,:)];
    
   
    

    
    
    d1=sqrt(sum((face_pcl.Location(y2,:) - face_pcl.Location(x1,:) ) .^ 2));
    d2=sqrt(sum((face_pcl.Location(y2,:) - face_pcl.Location(x2,:) ) .^ 2));
    d3=sqrt(sum((face_pcl.Location(y1,:) - face_pcl.Location(x1,:) ) .^ 2));
    d4=sqrt(sum((face_pcl.Location(y1,:) - face_pcl.Location(x2,:) ) .^ 2));
    f_dim(i,:)=[sqrt((d1^2+d4^2)/2) sqrt((d2^2+d3^2)/2)];
    dim(1+(i-1)*2)=sqrt((d1^2+d4^2)/2);
    dim(2+(i-1)*2)=sqrt((d2^2+d3^2)/2);
    
    
    
    if inNum/size(f_object_pcl.Location,1)>0.93
        k=i;
        break
    end
end

centre=[mean(plane_vertexs(1:4,1)) mean(plane_vertexs(1:4,2)) mean(plane_vertexs(1:4,3))];
normal=normals(1,:);
if normal(2)<0
    normal=-normal;
end
switch k
    case 0
        dimensions=0;
        aperture=0;
        angle=0;
        acc_box=0;
    case 1
        dimensions=dim(1:2)*100;
        aperture=min(dimensions)*100;
        angle = 0;%atan2d(norm(cross([orientation(1) 0 orientation(3)],[0 1 0])),dot([orientation(1) 0 orientation(3)],[0 1 0]));
    case 2
        alpha = atan2d(norm(cross(ground,normals(1,:))),dot(ground,normals(1,:)));
        if alpha>20
            angle=atan2d(norm(cross([0 1 0],[normals(2,1:2) 0])),dot([0 1 0],[normals(2,1:2) 0]));
            aperture=min([dim(3) dim(4)])*100;
        else
            angle=atan2d(norm(cross([0 1 0],[normals(1,1:2) 0])),dot([0 1 0],[normals(1,1:2) 0]));
            aperture=min([dim(1) dim(2)])*100;
        end
        dimensions=[mean([dim(1) dim(3)]) dim(2) dim(4)]*100;
    case 3
        alpha = atan2d(norm(cross(ground,normals(1,:))),dot(ground,normals(1,:)));
        beta =atan2d(norm(cross(ground,normals(2,:))),dot(ground,normals(2,:)));
        if alpha<20
            angle=atan2d(norm(cross([0 1 0],[normals(1,1:2) 0])),dot([0 1 0],[normals(1,1:2) 0]));
            aperture=min([f_dim(1,1) f_dim(1,2)])*100;
        elseif beta<20
            angle=atan2d(norm(cross([0 1 0],[normals(2,1:2) 0])),dot([0 1 0],[normals(2,1:2) 0]));
            aperture=min([f_dim(2,1) f_dim(2,2)])*100;
        else
            angle=atan2d(norm(cross([0 1 0],[normals(3,1:2) 0])),dot([0 1 0],[normals(3,1:2) 0]));
            aperture=min([f_dim(3,1) f_dim(3,2)])*100;
        end
        dimensions=[mean([f_dim(1,1) f_dim(3,1)])+0.01...
            mean([f_dim(1,2) f_dim(2,2)]+0.005)...
            mean([f_dim(2,1),f_dim(3,2)])+0.02]*100;
end

gamma=atan2d(norm(cross(normals(1,:),normals(2,:))),dot(normals(1,:),normals(2,:)));
delta=atan2d(norm(cross(normals(1,:),normals(3,:))),dot(normals(1,:),normals(3,:)));
if abs(90-gamma)>20 || abs(90-delta)>20
    acc_box=acc_box-70;
end
end

