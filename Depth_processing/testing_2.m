clc;
%clear;
%%

rosinit
topiclist = rostopic("list");

%%
rgb_sub = rossubscriber('/camera/color/image_raw','sensor_msgs/Image');
pc_sub = rossubscriber('/camera/depth/color/points','sensor_msgs/PointCloud2');
ori_dim_pub = rospublisher('/ori_dim','std_msgs/Float64MultiArray');
ori_img_pub = rospublisher('/ori_img','sensor_msgs/Image');


pause(2) % Wait to ensure publisher is registered
ori_dim_msg = rosmessage(ori_dim_pub);



ori_img_msg = rosmessage('sensor_msgs/Image');
ori_img_msg.Encoding = 'rgb8';


% figure

% imshow(img)
%% Read and Plot the point cloud
tests=1;
j=0;
excel=zeros(tests,7);


while 1
    prompt = '1=frame , 2=exit: ';
    j=input(prompt);
    if j==1
        
        for k=1:tests
            msg = receive(rgb_sub,10);
            img = readImage(msg);
            writeImage(ori_img_msg, img);
            send(ori_img_pub, ori_img_msg);
            
            msg2 = receive(pc_sub,10);
            xyz = readXYZ(msg2);
            pcl= pointCloud(xyz);
            pcl2=pcdownsample(pcl,'random',0.8);
           
           
%             display point cloud
            pcshow(pcl2);
            hold on
            xlabel('X')
            ylabel('Y')
            zlabel('Z')
            
            %%
            
            
            [dimensions, object, confidence, angle, aperture_size]=test_primitive_fitting_2(pcl2);
           
            disp(num2str(object))
            
            ori_dim_msg.Data = [object aperture_size angle];
            send(ori_dim_pub,ori_dim_msg);
            %             pause(2)
        end
        
        %         filename = 'testdata.xlsx';
        %         writematrix(excel,filename,'Sheet',1,'Range','A1:G5')
        
        
    elseif j==2
        break
    end
end

rosshutdown







