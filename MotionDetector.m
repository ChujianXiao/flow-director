clear;
clc;

v = VideoReader('input.mp4');

%ADJUST PARAMETERS HERE
W = 500; %Pixel width of the cubemap

if ~exist('frames', 'dir')
    mkdir('frames');
end

%Initialize optical flow objects
frontFlow = opticalFlowFarneback;
rightFlow = opticalFlowFarneback;
backFlow = opticalFlowFarneback;
leftFlow = opticalFlowFarneback;
topFlow = opticalFlowFarneback;
bottomFlow = opticalFlowFarneback;

%Custom figure to view the flow vectors
h = figure;
movegui(h);
hViewPanel = uipanel(h,'Position',[0 0 1 1],'Title','Plot of Optical Flow Vectors');
hPlot = axes(hViewPanel);

%Initialize direction output
direction_history = [0,0,0,0];

%Loop through all the frames in the video
i = 1; %Video frame index
outIdx = 1; %Ouput index

while hasFrame(v)
    frame = readFrame(v);

    %Output one frame for every 5 frames.
    if mod(i,5) == 0
        frame_gray = im2gray(frame);
        frame_cube = equi2cubic(frame_gray, W); 

        %Update optical flow for the current frame
        frontFlowVectors = estimateFlow(frontFlow, frame_cube{1});
        %Estimate FOE
        frontFOE(outIdx) = estimateFOE(frontFlowVectors,W,'front');
        
        %Do this for each face
        rightFlowVectors = estimateFlow(rightFlow, frame_cube{2});
        rightFOE(outIdx) = estimateFOE(rightFlowVectors,W,'right');
        
        backFlowVectors = estimateFlow(backFlow, frame_cube{3});
        backFOE(outIdx) = estimateFOE(backFlowVectors,W,'back');
        
        leftFlowVectors = estimateFlow(leftFlow, frame_cube{4});
        leftFOE(outIdx) = estimateFOE(leftFlowVectors,W,'left');

        topFlowVectors = estimateFlow(topFlow, frame_cube{5});
        topFOE(outIdx) = estimateFOE(topFlowVectors,W,'top');

        bottomFlowVectors = estimateFlow(bottomFlow, frame_cube{6});
        bottomFOE(outIdx) = estimateFOE(bottomFlowVectors,W,'bottom');

        faces = [frontFOE(outIdx),rightFOE(outIdx),backFOE(outIdx),leftFOE(outIdx),topFOE(outIdx),bottomFOE(outIdx)];
        best_face(outIdx) = chooseBestFace(faces);

        %Create an array for direction output
        direction_history(outIdx,1) = i;
        direction_history(outIdx,2:4) = (best_face(outIdx).direction_final)';
        
        %Show the right image flow as a demo
        imshow(frame_cube{2})
        hold on
        plot(rightFlowVectors,'DecimationFactor',[5 5],'ScaleFactor',2,'Parent',hPlot);
        plot(rightFOE(outIdx).px_pix, rightFOE(outIdx).py_pix, 'ro', 'MarkerSize', 12, 'LineWidth', 2);
        hold off
        pause(0.1)
        
        outIdx = outIdx + 1;
    end
    i = i + 1;
end

%Interpolate FOE between frames
direction_all_frames = interpolateDirection(direction_history);

%Write the final directions to a file
%{
T = table(direction_history(:,1),direction_history(:,2),direction_history(:,3),direction_history(:,4), ...
    'VariableNames', {'Frame','X','Y','Z'});
writetable(T, 'directions.csv');
%}
%Begin creating the output video
writer = VideoWriter('output_with_foe.mp4', 'MPEG-4');
writer.FrameRate = v.FrameRate;
open(writer);

frame_count = 1;
outputv = VideoReader('input.mp4');
while hasFrame(outputv)
    frame = readFrame(outputv);

    frame_cube = equi2cubic(frame, W); 

    %Check if this frame has FOE direction
    idx = find(direction_all_frames(:,1) == frame_count, 1);

    if ~isempty(idx)
        dir3 = direction_all_frames(idx, 2:4);
        [faceIdx, px, py] = dirToFacePixel(dir3, W);
        faceImg = frame_cube{faceIdx};
        frame_cube{faceIdx} = drawRingOnImage(faceImg,px,py,20,15,[255,0,0]);
    end

    frame = cubic2equi(frame_cube{5},frame_cube{6},frame_cube{4},...
        frame_cube{2},frame_cube{1},frame_cube{3});
    writeVideo(writer, frame);
    frame_count = frame_count+1;
end
close(writer);

function FOE = estimateFOE(flow,flow_width,cube_face)
    %This function estimates the FOE for each cube face
    %INPUTS: 
    %flow - Optical flow object
    %flow_width - The width of the cube face
    %cube_face - Direction of cube face, e.g. front, back
    %OUTPUTS:
    %FOE - struct that contains the outputs(see output initialization)

    %ADJUST PARAMETERS HERE
    valid_thresh = 0.5; %min magnitude for the optical flow vector to filter out noise
    facing_thresh = 0.2; %threshold for checking if we're confident on whether it's expansion or contraction

    %initialize outputs
    FOE.cube_face = cube_face; %Direction of cube face, e.g. front, back
    FOE.px_pix = NaN; %Pixel coords for plotting the FOE
    FOE.py_pix = NaN;
    FOE.direction3D = []; %Estimated 3D motion direction based on FOE
    FOE.radial_strength = NaN;
    FOE.num_valid = NaN;
    FOE.score = NaN; %Score for evaluating which face has the best FOE
    FOE.exp_or_cont = 'invalid'; %Determine expansion or contraction
    FOE.status = 'invalid inputs'; %Status of the outputs
    
    valid = flow.Magnitude > valid_thresh; %Filter valid magnitudes
    num_valid = nnz(valid);
    FOE.num_valid = num_valid;
    if num_valid < 6
        FOE.status = 'not enough valid flow vectors';
        return
    end

    %Create a meshgrid of pixel coordinates(coordinates are centered at the center of the pixel, hence the -0.5)
    x_pixel = ((1:flow_width) - 0.5)/flow_width * 2 - 1;
    y_pixel = ((1:flow_width) - 0.5)/flow_width * 2 - 1;
    [X_pixel,Y_pixel] = meshgrid(x_pixel, y_pixel);
    
    %Extract valid vectors
    Mag_v = flow.Magnitude(valid);
    Vx_v = flow.Vx(valid);
    Vy_v = flow.Vy(valid);
    X_pixel_v = X_pixel(valid);
    Y_pixel_v = Y_pixel(valid);

    %Build linear system A * p = b
    A = [Vy_v, -Vx_v];
    b = X_pixel_v .* Vy_v - Y_pixel_v .* Vx_v;

    %Least squares solution
    [p, stdx, ~] = lscov(A,b);
    %optional:weighted least squares, inconclusive results
    %[p, stdx, ~] = lscov(A,b,Mag_v)
    px_est = p(1);
    py_est = p(2);
    
    %Score is based on standard errors and the number of valid vectors
    FOE.score = (stdx(1)+stdx(2));

    %Check if FOE makes sense(falls within image bounds)
    if px_est < -1 || px_est > 1 || py_est < -1 || py_est > 1
        FOE.status = 'estimated FOE outside image bounds';
        return
    end
    
    %Output pixel coordinates for plotting
    FOE.px_pix = (px_est + 1) * flow_width/2;
    FOE.py_pix = (py_est + 1) * flow_width/2;

    %Convert to 3D direction based on cube facing
    switch cube_face
        case 'front'
            dir3 = [px_est, py_est, 1];
        case 'back'
            dir3 = [px_est, py_est, -1];
        case 'left'
            dir3 = [-1, py_est, px_est];
        case 'right'
            dir3 = [1, py_est, -px_est];
        case 'top'
            dir3 = [px_est, 1, -py_est];
        case 'bottom'
            dir3 = [px_est, -1, py_est];
        otherwise
            FOE.status = 'cube_face invalid';
            return
    end
    %Normalize
    dir3 = dir3/norm(dir3);
    FOE.direction3D = dir3;

    %Now we need to know if this is expansion or contraction.
    %First compute the radial unit vectors
    rx = X_pixel_v - px_est;
    ry = Y_pixel_v - py_est;
    rnorm = sqrt(rx.^2 + ry.^2);
    %Might divide by 0 for very small rnorms, to be safe we set a valid
    %radius
    valid_radius = rnorm > 1e-6;
    rx_u = rx(valid_radius) ./ rnorm(valid_radius); 
    ry_u = ry(valid_radius) ./ rnorm(valid_radius);

    %Calculate unit motion vectors
    Vx_u = Vx_v ./ Mag_v;
    Vy_u = Vy_v ./ Mag_v;
    
    %Select same valid radius
    Vx_u_r = Vx_u(valid_radius);
    Vy_u_r = Vy_u(valid_radius);
    
    %If expanding, direction dot product with radial vector should be positive
    %If contracting, direction dot product with radial vector should be negative
    d =  Vx_u_r.* rx_u + Vy_u_r .* ry_u;
    
    %Take the median
    %Mag_r = Mag_v(valid_radius);
    d_med = median(d);
    %optional:weighted median, inconclusive results
    %d_med = median(d,Weights=Mag_r);

    %If the median is above the threshold, output the result and reverse
    %the 3D vector direction if needed
    if d_med > facing_thresh
        FOE.exp_or_cont = 'expansion';
    elseif d_med < -facing_thresh
        FOE.exp_or_cont = 'contraction';
        FOE.direction3D = -FOE.direction3D;
    else
        FOE.exp_or_cont = 'inconclusive';
    end

    %Divide score by the radial strength
    %Radial strength close to 1 is best
    %Heavily penalize low radial strength(close to 0)
    radialStrength = median(abs(d));
    FOE.radial_strength = radialStrength;
    FOE.score = FOE.score/(max(radialStrength,1e-3)^3);

    FOE.status = 'valid';
end

function best_face = chooseBestFace(faces)
    %Initialize outputs
    best_face.best_face_name = NaN;
    best_face.direction_final = [];
    best_face.status = 'invalid';

    candidates = [];
    scores = [];

    for f = 1:numel(faces)
        %Discard if invalid
        if ~isfield(faces(f),'status') || ~strcmp(faces(f).status,'valid')
            continue
        end
        face_score = faces(f).score;

        if strcmp(faces(f).exp_or_cont,'inconclusive')
            face_score = face_score * 4; %Penalize inconclusive expansions heavily
        elseif strcmp(faces(f).exp_or_cont,'contraction')
            face_score = face_score * 1.5; %Prefer expansions
        end

        candidates(end+1) = f;
        scores(end+1) = face_score;
    end
    %Return if no valid candidates
    if isempty(candidates)
        best_face.status = 'no valid cube face found';
        best_face.direction_final = [NaN; NaN; NaN];
        return
    else
        [~, idx] = min(scores);
        best = candidates(idx);
        best_face.direction_final = faces(best).direction3D;
        best_face.best_face_name = faces(best).cube_face;
    end
    best_face.status = 'valid';
end

function direction_all_frames = interpolateDirection(direction_history)
    %Interpolate direction between frames for smoothed FOE
    direction_all_frames = [];
    %Add direction to 1st frame
    %first_direction  = [1,0,0,0];
    %direction_history = [first_direction;direction_history];
    N = size(direction_history,1);

    for i = 1:N-1
        f0 = direction_history(i,1);
        f1 = direction_history(i+1,1);
        v0 = direction_history(i,2:4);
        v1 = direction_history(i+1,2:4);

        %Convert direction vectors to quaternions
        q0 = quaternion([0,v0]);
        q1 = quaternion([0,v1]);

        %interpolate frames between f0 and f1
        for f = f0:f1-1
            %The step for each frame, i.e. for 5 frames, this would iterate
            %from 0-0.2-0.4-0.6-0.8
            t = (f-f0)/(f1-f0);
            %Interpolate using Navigation toolbox's slerp function
            q_interp = slerp(q0, q1, t);
            %Convert back to unit direction
            q_compact = compact(q_interp);
            v_interp = q_compact(2:4);
            direction_all_frames = [direction_all_frames; f, v_interp];
        end
    end
    %Append last frame
    direction_all_frames = [direction_all_frames;direction_history(end,:)];
end

function [faceIdx, px_pix, py_pix] = dirToFacePixel(d, W)
    x = d(1); y = d(2); z = d(3);
    [~, m] = max(abs([x,y,z]));
    if m == 1
        p = d / abs(x);
        if x > 0
            faceIdx = 2;
            u = -p(3); v = p(2);
        else
            faceIdx = 4;
            u = p(3); v = p(2);
        end
    elseif m == 2
        p = d / abs(y);
        if y > 0
            faceIdx = 5;
            u = p(1); v = -p(3);
        else
            faceIdx = 6;
            u = p(1); v = p(3);
        end
    else
        p = d / abs(z);
        if z > 0
            faceIdx = 1;
            u = p(1); v = p(2);
        else
            faceIdx = 3;
            u = p(1); v = p(2);
        end
    end
    %clamp
    u = max(-1, min(1, u));
    v = max(-1, min(1, v));
    %Convert to pixel coords
    px_pix = (u+1)*W/2;
    py_pix = (v+1)*W/2;
end

function img = drawRingOnImage(img, cx, cy, rOuter, rInner, color)
    [h, w, ~] = size(img);
    cx = max(1, min(w, cx));
    cy = max(1, min(h, cy));
    [X, Y] = meshgrid(1:w, 1:h);
    d2 = (X - cx).^2 + (Y - cy).^2;
    mask = (d2 <= rOuter^2) & (d2 >= rInner^2);
    for c = 1:3
        ch = img(:,:,c);
        ch(mask) = color(c);
        img(:,:,c) = ch;
    end
end