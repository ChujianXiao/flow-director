%% Clear all
clear
close all
clc

%% Main
video = VideoReader('input.mp4');
vertRes = video.Height/4; %Vertical Resolution of Mask
dimPixelLife = uint8(video.FrameRate/2); %The duration the masks pixel stays on screen for
currFrame = 1;
threshold = 6; %Threshold for magnitude

%Initialize optical flow objects
topFlow = opticalFlowFarneback;
bottomFlow = opticalFlowFarneback;
leftFlow = opticalFlowFarneback;
rightFlow = opticalFlowFarneback;
frontFlow = opticalFlowFarneback;
backFlow = opticalFlowFarneback;

%Intialize Mask Faces
Top = zeros(vertRes, vertRes, dimPixelLife, "uint8");
Bottom = zeros(vertRes, vertRes, dimPixelLife, "uint8");
Left = zeros(vertRes, vertRes, dimPixelLife, "uint8");
Right = zeros(vertRes, vertRes, dimPixelLife, "uint8");
Front = zeros(vertRes, vertRes, dimPixelLife, "uint8");
Back = zeros(vertRes, vertRes, dimPixelLife, "uint8");

%Initialize Mask File
negDimMask = VideoWriter("negDimMask", "MPEG-4");
negDimMask.FrameRate = video.FrameRate;

open(negDimMask);
while hasFrame(video)
    %Extract grayscale cubemap faces
    frame = readFrame(video);
    frame_gray = im2gray(frame);
    frame_cube = equi2cubic(frame_gray, vertRes);

    %Calculate per pixel opticalFlow Magnitude
    toSubtractFront = estimateFlow(frontFlow, frame_cube{1}).Magnitude;
    toSubtractRight = estimateFlow(rightFlow, frame_cube{2}).Magnitude;
    toSubtractBack = estimateFlow(backFlow, frame_cube{3}).Magnitude;
    toSubtractLeft = estimateFlow(leftFlow, frame_cube{4}).Magnitude;
    toSubtractTop = estimateFlow(topFlow, frame_cube{5}).Magnitude;
    toSubtractBottom = estimateFlow(bottomFlow, frame_cube{6}).Magnitude;

    %Threshold Magnitudes
    toSubtractFront(toSubtractFront <= threshold) = 0;
    toSubtractRight(toSubtractRight <= threshold) = 0;
    toSubtractBack(toSubtractBack <= threshold) = 0;
    toSubtractLeft(toSubtractLeft <= threshold) = 0;
    toSubtractTop(toSubtractTop <= threshold) = 0;
    toSubtractBottom(toSubtractBottom <= threshold) = 0;

    %Find the Highest value of all faces this frame
    highestMagnitude = max([toSubtractTop toSubtractBottom toSubtractLeft; ...
        toSubtractRight toSubtractFront toSubtractBack], [], "all");
    
    %Bundle magnitudes into bins of frames according to dimPixelLife
    if currFrame <= dimPixelLife
        Top(:,:,currFrame) = uint8((toSubtractTop/highestMagnitude)*255);
        Bottom(:,:,currFrame) = uint8((toSubtractBottom/highestMagnitude)*255);
        Left(:,:,currFrame) = uint8((toSubtractLeft/highestMagnitude)*255);
        Right(:,:,currFrame) = uint8((toSubtractRight/highestMagnitude)*255);
        Front(:,:,currFrame) = uint8((toSubtractFront/highestMagnitude)*255);
        Back(:,:,currFrame) = uint8((toSubtractBack/highestMagnitude)*255);
    else
        Top = circshift(Top, [0 0 -1]);
        Bottom = circshift(Bottom, [0 0 -1]);
        Left = circshift(Left, [0 0 -1]);
        Right = circshift(Right, [0 0 -1]);
        Front = circshift(Front, [0 0 -1]);
        Back = circshift(Back, [0 0 -1]);
        Top(:,:,dimPixelLife) = uint8((toSubtractTop/highestMagnitude)*255);
        Bottom(:,:,dimPixelLife) = uint8((toSubtractBottom/highestMagnitude)*255);
        Left(:,:,dimPixelLife) = uint8((toSubtractLeft/highestMagnitude)*255);
        Right(:,:,dimPixelLife) = uint8((toSubtractRight/highestMagnitude)*255);
        Front(:,:,dimPixelLife) = uint8((toSubtractFront/highestMagnitude)*255);
        Back(:,:,dimPixelLife) = uint8((toSubtractBack/highestMagnitude)*255);
    end

    %Sum the scaled magnitudes to achieve a gradual effect
    sumTop = sum(Top, 3, "native");
    sumBottom = sum(Bottom, 3, "native");
    sumLeft = sum(Left, 3, "native");
    sumRight = sum(Right, 3, "native");
    sumFront = sum(Front, 3, "native");
    sumBack = sum(Back, 3, "native");

    %Stitch to an equilateral frame and write to the mask video file
    outIm = cubic2equi(sumTop(:,:,[1 1 1]), sumBottom(:,:,[1 1 1]), sumLeft(:,:,[1 1 1]), sumRight(:,:,[1 1 1]), sumFront(:,:,[1 1 1]), sumBack(:,:,[1 1 1]));
    writeVideo(negDimMask, outIm);
    currFrame = currFrame + 1;
end
close(negDimMask);

%% Apply the Mask
clear
video = VideoReader('input.mp4');
negDimMask = VideoReader('negDimMask.mp4');

dimmedVid = VideoWriter("dimmedVid", "MPEG-4");
dimmedVid.FrameRate = video.FrameRate;

open(dimmedVid);
while hasFrame(video) && hasFrame(negDimMask)
    frame = readFrame(video);
    frameMask = readFrame(negDimMask);

    newFrame = frame - imresize(frameMask, video.Height/negDimMask.Height);
    writeVideo(dimmedVid, newFrame);
end
close(dimmedVid);