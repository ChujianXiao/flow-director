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

bottomFlow = opticalFlowFarneback;

%Custom figure to view the flow vectors
h = figure;
movegui(h);
hViewPanel = uipanel(h,'Position',[0 0 1 1],'Title','Plot of Optical Flow Vectors');
hPlot = axes(hViewPanel);

%Loop through all the frames in the video
i = 1; %Video frame index
outIdx = 1; %Ouput index
while hasFrame(v)
    frame = readFrame(v);
    frame_gray = im2gray(frame);
    frame_cube = equi2cubic(frame_gray, W); 

    %Output one frame for every 5 frames.
    if mod(i,5) == 0
        %frontname = fullfile('frames', sprintf('frame_%05d_front.jpg', outIdx));
        %Update optical flow for the current frame
        %frontFlowVectors = estimateFlow(frontFlow, frame_cube{1});   
        %imwrite(frame_cube{1}, frontname, 'jpg', 'Quality', 95);
        
        %rightname = fullfile('frames', sprintf('frame_%05d_right.jpg', outIdx));
        rightFlowVectors = estimateFlow(rightFlow, frame_cube{2});
        rightFOE(outIdx) = estimateFOE(rightFlowVectors,W,'right');
        %imwrite(frame_cube{2}, rightname, 'jpg', 'Quality', 95);
        %{
        backname = fullfile('frames', sprintf('frame_%05d_back.jpg', outIdx));
        backFlowVectors = estimateFlow(backFlow, frame_cube{3});
        imwrite(frame_cube{3}, backname, 'jpg', 'Quality', 95);

        leftname = fullfile('frames', sprintf('frame_%05d_left.jpg', outIdx));
        leftFlowVectors = estimateFlow(leftFlow, frame_cube{4});
        imwrite(frame_cube{4}, leftname, 'jpg', 'Quality', 95);
        %}

        %bottomFlowVectors = estimateFlow(rightFlow, frame_cube{6});
        
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

function FOE = estimateFOE(flow,flow_width,cube_face)
    %ADJUST PARAMETERS HERE
    valid_thresh = 0.5; %min magnitude for the optical flow vector to filter out noise
    facing_thresh = 0.2; %threshold for checking if we're confident on whether it's expansion or contraction

    % initialize outputs
    FOE.px_pix = NaN;
    FOE.py_pix = NaN;
    FOE.direction3D = [];
    FOE.rms_error = NaN;
    FOE.exp_or_cont = 'invalid';
    FOE.status = 'invalid inputs';
    
    valid = flow.Magnitude > valid_thresh; %Filter valid magnitudes
    if nnz(valid) < 6
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

    %Weighted least squares solution, weight is based on magnitude of each
    %vector
    [p, ~, mse2] = lscov(A,b,Mag_v);
    px_est = p(1);
    py_est = p(2);
    FOE.rms_error = sqrt(mse2);

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
    
    %Take the weighted median based on magnitude
    Mag_r = Mag_v(valid_radius);
    d_med = median(d,Weights=Mag_r);

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

    FOE.status = 'valid';
end