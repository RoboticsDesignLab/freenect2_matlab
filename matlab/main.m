function main( pipeline )
%DISPLAY Summary of this function goes here
%   Detailed explanation goes here

    ctx = Freenect2();
    n = 10;
    
    fprintf('Retreived Freenect2 Context.\n');
    
    cleanupObj = onCleanup(@()delete(ctx));

    fprintf('Enumerating available devices.\n');
    
    dc = ctx.enumerateDevices();

    if (dc == 0) 
        fprintf('Unable to locate kinect device.\n');
        return;
    else
        fprintf('%i Kinect2 devices available.\n', dc);
    end

    fprintf('Opening Kinect2 device.\n');
    ctx.openDevice('', PipelineType.OPENGL);
    
    fprintf('Initializing Kinect2 device.\n');
    ctx.initializeDevice();

    %figure;
    %subplot(1, 2, 1);
    
    fprintf('Beginning main frame read loop.\n');
    while true
        tic;
        for i=1:n
            result = 0;
            while result == 0
                result = ctx.processFrame();
            end

            colorFrame = ctx.getFrame(FrameType.Color);
            depthFrame = ctx.getFrame(FrameType.Depth);
            irFrame = ctx.getFrame(FrameType.IR);
            undistortedFrame = ctx.getFrame(FrameType.Undistorted);
            registeredFrame = ctx.getFrame(FrameType.Registered);
            bigdepthFrame = ctx.getFrame(FrameType.BigDepth);

            ctx.cleanupFrame();

            %subplot(2, 1, 1);
            %imshow(colorFrame(:,:,1:3));    

            %subplot(2, 1, 2);
            %imshow(bigdepthFrame, []);

        %     subplot(2, 2, 1);
        %     imshow(irFrame, []);
        %     
        %     subplot(2, 2, 2);
        %     imshow(depthFrame, []);
        %     
        %     subplot(2, 2, 3);
        %     imshow(undistortedFrame, []);
        %     
        %     subplot(2, 2, 4);
        %     imshow(registeredFrame(:,:, 1:3));


            drawnow;
        end
        avg = toc / n;
        fprintf('Read %i frames (%g FPS, %g s/frame mean).\n', n, 1/avg, avg);
    end

end

