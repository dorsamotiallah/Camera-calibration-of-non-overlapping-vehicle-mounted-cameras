% Visualize exported Agilex ORB-SLAM maps in MATLAB.
%
% Run from the orbcalib-master repo root:
%   run("tools/visualize_agilex_maps_matlab.m")

scriptPath = string(mfilename("fullpath"));
if strlength(scriptPath) > 0
    scriptDir = string(fileparts(scriptPath));
    repoDir = string(fileparts(scriptDir));
else
    repoDir = "/home/civit/Desktop/Dorsa/orbcalib-master";
end
runDir = fullfile(repoDir, "results_agilex", "2026-06-02_front_back_fisheye_full");
frontPly = fullfile(runDir, "front_map.ply");
backPly = fullfile(runDir, "back_map.ply");

[frontPts, frontRgb] = readAsciiPly(frontPly);
[backPts, backRgb] = readAsciiPly(backPly);

figure("Name", "Agilex ORB-SLAM Maps", "Color", "w");
hold on;
grid on;
axis equal;

% The exporter stores map points in gray and keyframe centers in blue.
% Tint each camera map slightly so front/back are easy to separate.
frontColor = tintColors(frontRgb, [0.1, 0.45, 1.0]);
backColor = tintColors(backRgb, [1.0, 0.25, 0.15]);

scatter3(frontPts(:,1), frontPts(:,2), frontPts(:,3), 3, frontColor, ".");
scatter3(backPts(:,1), backPts(:,2), backPts(:,3), 3, backColor, ".");

xlabel("x");
ylabel("y");
zlabel("z");
title("Agilex front/back ORB-SLAM sparse maps");
legend("front map", "back map", "Location", "best");
view(3);
rotate3d on;

plotSingleMap("Agilex front ORB-SLAM map", frontPts, frontColor, frontPly);
plotSingleMap("Agilex back ORB-SLAM map", backPts, backColor, backPly);

fprintf("Front: %d vertices from %s\n", size(frontPts, 1), frontPly);
fprintf("Back : %d vertices from %s\n", size(backPts, 1), backPly);

function plotSingleMap(name, points, colors, sourcePath)
    figure("Name", name, "Color", "w");
    scatter3(points(:,1), points(:,2), points(:,3), 3, colors, ".");
    grid on;
    axis equal;
    xlabel("x");
    ylabel("y");
    zlabel("z");
    title(name, "Interpreter", "none");
    subtitle(sourcePath, "Interpreter", "none");
    view(3);
    rotate3d on;
end

function colors = tintColors(rgb, tint)
    rgb = double(rgb) / 255.0;
    tint = reshape(tint, 1, 3);
    colors = 0.35 * rgb + 0.65 * tint;
    colors = max(0, min(1, colors));
end

function [points, rgb] = readAsciiPly(path)
    path = char(path);
    fid = fopen(path, "r");
    if fid < 0
        error("Could not open %s", path);
    end

    cleanup = onCleanup(@() fclose(fid));
    vertexCount = [];
    while true
        line = fgetl(fid);
        if ~ischar(line)
            error("Unexpected EOF while reading PLY header: %s", path);
        end
        if startsWith(line, "element vertex")
            parts = split(strtrim(line));
            vertexCount = str2double(parts{3});
        elseif strcmp(strtrim(line), "end_header")
            break;
        end
    end

    if isempty(vertexCount) || ~isfinite(vertexCount)
        error("Could not find vertex count in PLY header: %s", path);
    end

    data = textscan(fid, "%f %f %f %f %f %f", vertexCount);
    data = cell2mat(data);
    if size(data, 1) ~= vertexCount
        warning("Expected %d vertices but read %d from %s", vertexCount, size(data, 1), path);
    end

    points = data(:, 1:3);
    rgb = uint8(data(:, 4:6));
end
