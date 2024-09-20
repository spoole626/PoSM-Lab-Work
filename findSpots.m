function bbox_final = findSpots(filepath)
    video = VideoReader(filepath);

    firstFrame = read(video,1);

    [startX,endX] = findBounds(firstFrame);

    %Assumption that start and end are given in its matrix orientation
    bbox_final = zeros(0,4); %initalizes bounding box output (in pixel coordinates)

    %translate to matrix coordinates
    startCol = uint32(startX-0.5);
    endCol = uint32(endX - 0.5);

    firstFrame = read(video,1); %gets the first frame
    grayFrame = rgb2gray(firstFrame); %converts to gray scale
    grayFrame = grayFrame(:,startCol:endCol); %crops image

    BW_threshold = 0.5;
    darkBackground = false; %initializes variable 
    foregroundColor_list = []; %initializes variable
    dotColor_list = []; %initializes variable

    %Determines color of the foreground and dots
    while isempty(foregroundColor_list) && BW_threshold > 0
        [foregroundColor_list,dotColor_list] = getColorList(grayFrame, BW_threshold);
        BW_threshold = BW_threshold - 0.1;
    end

   %last resort solution (if edge detection couldn't sort out the dots)
    if isempty(foregroundColor_list) && BW_threshold <= 0
        if mean(grayFrame,'all') > 255/2 %light background
            darkBackground = false;
        else %dark background
            darkBackground = true;
        end
    end

    avgColorDiff = mean(foregroundColor_list - dotColor_list);

    %if foreground and dots were detected and the foreground on average is
    %lighter than the dots
    if ~isempty(foregroundColor_list) && avgColorDiff > 0
        darkBackground = false; %0 = bright background
    else 
        darkBackground = true;
    end

    if darkBackground
        %Finds the BRIGHT spots on the dark background
        sensitivity = 0;
        binaryFrame = imbinarize(grayFrame,'adaptive','ForegroundPolarity','bright','Sensitivity',sensitivity);
        while ~ismember(1,binaryFrame) %if there are no dots detected
            sensitivity = sensitivity + 0.05;
            binaryFrame = imbinarize(grayFrame,'adaptive','ForegroundPolarity','bright','Sensitivity',sensitivity);
        end
    else
        %Finds the DARK spots on the bright background
        sensitivity = 0.4;
        binaryFrame = imbinarize(grayFrame,'adaptive','ForegroundPolarity','dark','Sensitivity',sensitivity);

        %Inverts the black and white 
        binaryFrame = ~binaryFrame;

        while ~ismember(1,binaryFrame) %if there are no dots detected
            sensitivity = sensitivity + 0.05;
            binaryFrame = imbinarize(grayFrame,'adaptive','ForegroundPolarity','dark','Sensitivity',sensitivity);
            binaryFrame = ~binaryFrame;
        end
    end
    
    area_thresh = size(grayFrame,1)*0.2;
    binaryFrame = filter(binaryFrame,area_thresh);

    bbox_total = mergeBBox(binaryFrame);

    bbox_final = confirmBBox(binaryFrame,bbox_total);

    printBoundingBoxes(binaryFrame,bbox_final)
end

function [xStart,xEnd] = findBounds(firstFrame)
    boundsDrawn = false;
    attemptMade = false;

    idxStart = 1;
    idxEnd = size(firstFrame,2);

    while ~boundsDrawn
        clf; %clears previous figure

        imshow(firstFrame); %shows first frame of video
        hold on;
        if attemptMade
            xline(xStart,"Color",[0.5 0 0]);
            xline(xEnd,"Color",[0.5 0 0]);
        else
            attemptMade = true;
        end

        title("Draw left bound, click x when done");
        drawnow;
        lineStart = drawline;
        xStart = mean(lineStart.Position(:,1));
        waitfor(gcf,'CurrentCharacter','x') %wait until enter is pressed
        title("Draw right bound, click x when done");
        %xline(xStart,"Color",[0.5 0 0]); %Draws left bound for user to see
        lineEnd = drawline;
        xEnd = mean(lineEnd.Position(:,1));
        waitfor(gcf,'CurrentCharacter','x') %wait until enter is press

        clf; %clears previous figure

        idxStart = uint32(xStart-0.5);
        idxEnd = uint32(xEnd - 0.5);

        imshow(firstFrame(:,[idxStart:idxEnd],:)); %shows first frame of video
        redo_prompt = "Type 'r' if you want to redraw bounds or d if done: ";
        response = input(redo_prompt,"s");

        while ~strcmp(response,'d') && ~strcmp(response,'r')
            redo_prompt = "Type 'r' if you want to redraw bounds or d if done: ";
            response = input(redo_prompt,"s");
        end

        if strcmp(response,'d')
            boundsDrawn = true;
        end
    end

    close; %clears figure
end

function bbox_final = confirmBBox(binaryFrame,bbox)
    bbox_final = zeros(0,4);

    imshow(binaryFrame);
    hold on;
    title("Control + Click on dots you don't want to include. Click 'x' when done");
    ROI_rect = {};
    for i = 1:size(bbox,1)
        ROI_rect{end+1} = drawrectangle('Color',[1 0 0],'Position',bbox(i,:),'MarkerSize',1,'Selected',true,...
            'SelectedColor',[0 1 0]);
    end

    waitfor(gcf,'CurrentCharacter','x')

    for i = 1:size(bbox,1)
        if ROI_rect{i}.Selected
            bbox_final(end+1,:) = ROI_rect{i}.Position;
        end
    end

    close;

end

function binaryFrame = filter(binaryFrame,area_thresh)
    brightSpots = regionprops("table",binaryFrame, 'Centroid',"Area","BoundingBox");
    area = brightSpots.Area; %Gets area of each dot
    bbox = brightSpots.BoundingBox; %Gets bounding box of each dot

    for i = 1:size(area,1) %for each dot
        if area(i) < area_thresh %if it's considered too small to be an actual dot
            [row,col,height,width] = convertToSpatial(bbox(i,:));
            binaryFrame(row:row+height,col:col+width) = 0; %make black
        end
    end
end

function bbox_final = mergeBBox(binaryFrame)
    brightSpots = regionprops("table",binaryFrame, 'Centroid',"Area","BoundingBox");
    area = brightSpots.Area; %Gets area of each dot

    [area_sort,idx_sort] = sort(area,1,'descend');

    area = area(idx_sort);
    bbox = brightSpots.BoundingBox(idx_sort,:); %Gets bounding box of each dot
    outerBBox = createOuterBox(bbox);

    bbox_final = zeros(0,4);
    bbox_added = [];

    for i = 1:size(bbox,1)-1 %for every bounding box
        if ~ismember(i,bbox_added) %if not already added to the final bounding boxes
            verticalOverlap = [i]; %bounding box on top of each other (vertically)
            directOverlap = [i]; %bounding boxes that intercept each other
            directOverlap_inner = [i]; %outer bounding box intercepts with inner bounding box
            for j = i+1:size(bbox,1) %compares to every other bounding box
                i_left = uint32(outerBBox(i,1) + 0.5); %converts to spatial coordinates
                i_right = i_left + uint32(outerBBox(i,3)+ 0.5); %converts to spatial coordinates
                j_left = uint32(outerBBox(j,1) + 0.5); %converts to spatial coordinates
                j_right = j_left + uint32(outerBBox(j,3)+ 0.5); %converts to spatial coordinates
                if ~isempty(intersect(i_left:i_right,j_left:j_right)) %if boxes overlap vertically
                    verticalOverlap(end+1) = j;
                end
                if rectint(outerBBox(i,:),outerBBox(j,:)) > 0 %if boxes overlap (in general)
                    directOverlap(end+1) = j;
                    if rectint(outerBBox(i,:),bbox(j,:)) > 0
                        directOverlap_inner(end+1) = j;
                    end
                end
            end

            %for all direct overlaps, merge into one bounding box
            if length(directOverlap) > 1 %There is at least 1 direct overlap
                %to merge into one bounding box, find the smallest (x,y)
                %and largest (width,height)

                mergeBox_idx = intersect(directOverlap,directOverlap_inner);

                x_topLeft = min(bbox(mergeBox_idx,1),[],"all");
                y_topLeft = min(bbox(mergeBox_idx,2),[],"all");
                x_bottomRight = max(bbox(mergeBox_idx,1)+bbox(mergeBox_idx,3),[],"all");
                y_bottomRight = max(bbox(mergeBox_idx,2)+bbox(mergeBox_idx,4),[],"all");

                bbox_final(end+1,:) = [x_topLeft,y_topLeft,...
                    x_bottomRight-x_topLeft,y_bottomRight-y_topLeft];
                bbox_added = [bbox_added,mergeBox_idx]; 
            end
            %if there is only a vertical overlap (with no direct overlap),
            %keep only the largest box

            if length(verticalOverlap) > 1
                if length(directOverlap) <= 1 %if there were no direct overlaps (only vertical overlaps)

                    [M,I] = max(area(verticalOverlap),[],"all");
                    bbox_final(end+1,:) = bbox(verticalOverlap(I),:);
                    bbox_added = [bbox_added,verticalOverlap];
                else %there were also direct overlaps (biggest box would have already been added)
                    bbox_added = [bbox_added,verticalOverlap];
                end
            end
            %If there are no overlaps
            if length(directOverlap) <= 1 && length(verticalOverlap) <= 1
                bbox_final(end+1,:) = bbox(i,:);
                bbox_added(end+1) = i;
            end
        end
    end
    %checks the last element
    lastElement = size(bbox,1);
    if ~ismember(lastElement,bbox_added)
        bbox_final(end+1,:) = bbox(lastElement,:);
        bbox_added(end+1) = size(bbox,2);
    end
end

function outerBox = createOuterBox(bbox)
    %bbox is a matrix with each row: [x,y,width,height]
    outerBox = zeros(size(bbox,1),size(bbox,2));
    horizontalCoeff = 0.5;
    verticalCoeff = 0;
    outerBox(:,1) = bbox(:,1) - bbox(:,3)*horizontalCoeff;
    outerBox(:,2) = bbox(:,2) - bbox(:,4)*verticalCoeff;
    outerBox(:,3) = bbox(:,3) + 2*bbox(:,3)*horizontalCoeff;
    outerBox(:,4) = bbox(:,4) + 2*bbox(:,4)*verticalCoeff;
end

function [row,col,height,width] = convertToSpatial(bbox_arr)
    %bbox_arr is a row array: [x,y,width,height]
    row = uint32(bbox_arr(1,2) + 0.5);
    col = uint32(bbox_arr(1,1) + 0.5);
    height = uint32(bbox_arr(1,4) + 0.5);
    width = uint32(bbox_arr(1,3) + 0.5);
end

function [foregroundColor_list,dotColor_list] = getColorList(grayFrame,BW_threshold)
    BW = edge(grayFrame,'canny',BW_threshold);

    dotColor_list = zeros(0,1);
    foregroundColor_list = zeros(0,1);

    for col = 1:size(BW,2)
        if ismember(1,BW(:,col)) %sees if an edge exists
            topEdge_start = false;
            topEdge_end = false;
            bottomEdge_start = false;
            bottomEdge_end = false;
            notAdot = false;
            idx = zeros(1,4);
            for row = 1:size(BW,1)
                if BW(row,col) == 1 && ~topEdge_start && ~topEdge_end &&...
                        ~bottomEdge_start && ~bottomEdge_end && ~notAdot
                    idx(1,1) = row;
                    topEdge_start = true;
                elseif BW(row,col) == 0 && topEdge_start && ~topEdge_end &&...
                        ~bottomEdge_start && ~bottomEdge_end && ~notAdot
                    idx(1,2) = row;
                    topEdge_end = true;
                elseif BW(row,col) == 1 && topEdge_start && topEdge_end &&...
                        ~bottomEdge_start && ~bottomEdge_end && ~notAdot
                    idx(1,3) = row;
                    bottomEdge_start = true;
                elseif BW(row,col) == 0 && topEdge_start && topEdge_end &&...
                        bottomEdge_start && ~bottomEdge_end && ~notAdot
                    idx(1,4) = row;
                    bottomEdge_end = true;
                elseif BW(row,col) == 1 && topEdge_start && topEdge_end &&...
                        bottomEdge_start && bottomEdge_end && ~notAdot
                    notAdot = true;
                end
            end
            if topEdge_start && topEdge_end && bottomEdge_start && bottomEdge_end && ~notAdot
                aboveDot = mean(grayFrame(1:idx(1,1),col));
                belowDot = mean(grayFrame(idx(1,4):size(grayFrame,1),col));
                insideDot = mean(grayFrame(idx(1,2):idx(1,3),col));
    
                foregroundColor_list(end+1) = mean([aboveDot,belowDot]);
                dotColor_list(end+1) = insideDot;
            end
        end
    end
end

function printBoundingBoxes(binaryFrame,bbox)
    imshow(binaryFrame)
    hold on;
    title("Bounding boxes drawn. Click 'x' to close.");
    for i = 1:size(bbox,1)
        fourPoints = [bbox(i,1),bbox(i,2);...
            bbox(i,1)+bbox(i,3),bbox(i,2);...
            bbox(i,1)+bbox(i,3),bbox(i,2)+bbox(i,4);...
            bbox(i,1),bbox(i,2)+bbox(i,4);...
            bbox(i,1),bbox(i,2)];
        plot(fourPoints(:,1),fourPoints(:,2),"r-",'LineWidth',1);
    end
    waitfor(gcf,'CurrentCharacter','x');
    close;
end
