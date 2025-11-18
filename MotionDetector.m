v = VideoReader('input.mp4');

i = 1; %Video frame index
outIdx = 1; %Ouput index

if ~exist('frames', 'dir')
    mkdir('frames');
end

%Initialize optical flow objects
frontFlow = opticalFlowFarneback;
rightFlow = opticalFlowFarneback;
backFlow = opticalFlowFarneback;
leftFlow = opticalFlowFarneback;

%Custom figure to view the flow vectors
h = figure;
movegui(h);
hViewPanel = uipanel(h,'Position',[0 0 1 1],'Title','Plot of Optical Flow Vectors');
hPlot = axes(hViewPanel);

while hasFrame(v)
    frame = readFrame(v);
    frame_gray = im2gray(frame);
    frame_cube = equi2cubic(frame_gray, 500); 

    %Output one frame for every 5 frames.
    if mod(i,5) == 0
        %frontname = fullfile('frames', sprintf('frame_%05d_front.jpg', outIdx));
        % Update optical flow for the current frame
        %frontFlowVectors = estimateFlow(frontFlow, frame_cube{1});   
        %imwrite(frame_cube{1}, frontname, 'jpg', 'Quality', 95);
        
        
        %rightname = fullfile('frames', sprintf('frame_%05d_right.jpg', outIdx));
        rightFlowVectors = estimateFlow(rightFlow, frame_cube{2});
        %imwrite(frame_cube{2}, rightname, 'jpg', 'Quality', 95);
        %{
        backname = fullfile('frames', sprintf('frame_%05d_back.jpg', outIdx));
        backFlowVectors = estimateFlow(backFlow, frame_cube{3});
        imwrite(frame_cube{3}, backname, 'jpg', 'Quality', 95);

        leftname = fullfile('frames', sprintf('frame_%05d_left.jpg', outIdx));
        leftFlowVectors = estimateFlow(leftFlow, frame_cube{4});
        imwrite(frame_cube{4}, leftname, 'jpg', 'Quality', 95);
        %}
        
        %Show the right image flow as a demo
        imshow(frame_cube{2})
        hold on
        plot(rightFlowVectors,'DecimationFactor',[5 5],'ScaleFactor',2,'Parent',hPlot);
        hold off
        pause(0.1)
        
        outIdx = outIdx + 1;
    end
    i = i + 1;
end