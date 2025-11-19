function dimMap = createDimMap(video)
    %CREATEDIMMAP Summary of this function goes here
    %   Detailed explanation goes here
    currFrame = 1;
    %Initialize optical flow objects
    topFlow = opticalFlowFarneback;
    bottomFlow = opticalFlowFarneback;
    leftFlow = opticalFlowFarneback;
    rightFlow = opticalFlowFarneback;
    frontFlow = opticalFlowFarneback;
    backFlow = opticalFlowFarneback;
    max_motion = zeros(video.Height/4, video.Height/4, 6, video.NumFrames, "uint8");
    highest_vel = 0;

    % find the highest magnitude
    while hasFrame(video)
        frame = readFrame(video);
        frame_gray = im2gray(frame);
        frame_cube = equi2cubic(frame_gray, video.Height/4); 
        max_motion(:,:, 1, currFrame) = estimateFlow(topFlow, frame_cube{1}).Magnitude;
        max_motion(:,:, 2, currFrame) = estimateFlow(bottomFlow, frame_cube{2}).Magnitude;
        max_motion(:,:, 3, currFrame) = estimateFlow(leftFlow, frame_cube{3}).Magnitude;
        max_motion(:,:, 4, currFrame) = estimateFlow(rightFlow, frame_cube{4}).Magnitude;
        max_motion(:,:, 5, currFrame) = estimateFlow(frontFlow, frame_cube{5}).Magnitude;
        max_motion(:,:, 6, currFrame) = estimateFlow(backFlow, frame_cube{6}).Magnitude;
        curr_highest_vel = max(max_motion, [], "all");
        if curr_highest_vel > highest_vel
            highest_vel = curr_highest_vel;
        end
        currFrame = currFrame + 1;
    end
    
    dimMap = 255-((max_motion/highest_vel) * 255);
    clear max_motion

    % for i = 1:1:6
    %     for j = 1:1:video.NumFrames
    %         dimMap(:, :, i, j) = imcomplement(dimMap(:, :, i, j));
    %     end
    % end
    dimMap = dimMap;
end