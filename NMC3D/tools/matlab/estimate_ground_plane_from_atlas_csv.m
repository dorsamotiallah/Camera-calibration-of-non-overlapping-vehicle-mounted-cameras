function result = estimate_ground_plane_from_atlas_csv(csvPath, cameraHeightMeters, varargin)
%ESTIMATE_GROUND_PLANE_FROM_ATLAS_CSV Find a ground-like plane from ORB-SLAM atlas observations.
%
% This tool reads the CSV produced by NMC3D's atlas_ground_export executable,
% uses MATLAB pcfitplane to generate plane candidates, scores them with
% ground-plane cues, visualizes the selected plane, and reports the metric
% scale implied by a known camera height.
%
% Example:
%   result = estimate_ground_plane_from_atlas_csv( ...
%       "results_ground_scale/back_mono/atlas_observations.csv", 0.66, ...
%       "MaxDistance", 0.03, ...
%       "BottomThreshold", 0.55, ...
%       "NumPlanes", 8, ...
%       "Visualize", true);
%
% Optional name-value arguments:
%   MaxDistance       RANSAC/MSAC inlier distance in SLAM units. Default 0.03.
%   BottomThreshold   Normalized image y threshold for likely floor. Default 0.55.
%   NumPlanes         Number of sequential plane candidates. Default 8.
%   MinInliers        Minimum inliers for a valid plane. Default 50.
%   UseBottomCandidates Fit planes only from bottom-image map points. Default true.
%   ReferenceNormal   1x3 normal in SLAM frame. Default [].
%   MaxAngularDistance Degrees for ReferenceNormal constraint. Default 20.
%   Visualize         Show MATLAB figure. Default true.
%   OutputDir         Optional directory for selected-plane summary files.

parser = inputParser;
parser.addRequired("csvPath", @(x) ischar(x) || isstring(x));
parser.addRequired("cameraHeightMeters", @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("MaxDistance", 0.03, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("BottomThreshold", 0.55, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
parser.addParameter("NumPlanes", 8, @(x) isnumeric(x) && isscalar(x) && x >= 1);
parser.addParameter("MinInliers", 50, @(x) isnumeric(x) && isscalar(x) && x >= 3);
parser.addParameter("UseBottomCandidates", true, @(x) islogical(x) && isscalar(x));
parser.addParameter("ReferenceNormal", [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 3));
parser.addParameter("MaxAngularDistance", 20, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("Visualize", true, @(x) islogical(x) && isscalar(x));
parser.addParameter("OutputDir", "", @(x) ischar(x) || isstring(x));
parser.parse(csvPath, cameraHeightMeters, varargin{:});
opts = parser.Results;

if exist("pcfitplane", "file") ~= 2
    error("pcfitplane was not found. MATLAB Computer Vision Toolbox is required.");
end

obs = readtable(string(csvPath));
if height(obs) == 0
    error("Observation CSV is empty: %s", csvPath);
end

[mpIds, ~, mpGroup] = unique(obs.mp_id);
nMP = numel(mpIds);
points = zeros(nMP, 3);
bottomRatio = zeros(nMP, 1);

for i = 1:nMP
    rows = (mpGroup == i);
    first = find(rows, 1, "first");
    points(i, :) = [obs.pw_x(first), obs.pw_y(first), obs.pw_z(first)];

    imgHeight = max(obs.img_max_y(rows) - obs.img_min_y(rows), 1);
    yNorm = (obs.kp_y(rows) - obs.img_min_y(rows)) ./ imgHeight;
    bottomRatio(i) = mean(yNorm >= opts.BottomThreshold);
end

[~, camFirstRows] = unique(obs.kf_id, "stable");
cameras = [obs.cx(camFirstRows), obs.cy(camFirstRows), obs.cz(camFirstRows)];

candidateMask = true(nMP, 1);
if opts.UseBottomCandidates
    candidateMask = bottomRatio > 0;
end

remaining = find(candidateMask);
planes = struct([]);

for iPlane = 1:opts.NumPlanes
    if numel(remaining) < opts.MinInliers
        break;
    end

    cloud = pointCloud(points(remaining, :));

    try
        if isempty(opts.ReferenceNormal)
            [model, localInliers] = pcfitplane(cloud, opts.MaxDistance);
        else
            ref = double(reshape(opts.ReferenceNormal, 1, 3));
            ref = ref / max(norm(ref), eps);
            [model, localInliers] = pcfitplane(cloud, opts.MaxDistance, ref, opts.MaxAngularDistance);
        end
    catch err
        warning("pcfitplane failed for candidate %d: %s", iPlane, err.message);
        break;
    end

    inlierIdx = remaining(localInliers);
    if numel(inlierIdx) < opts.MinInliers
        remaining(localInliers) = [];
        continue;
    end

    params = double(model.Parameters);
    normal = params(1:3);
    normal = normal / max(norm(normal), eps);
    d = params(4) / max(norm(params(1:3)), eps);

    candidate = score_plane(points, cameras, bottomRatio, inlierIdx, normal, d);
    planes = [planes; candidate]; %#ok<AGROW>

    remaining(localInliers) = [];
end

if isempty(planes)
    error("No valid plane candidates found.");
end

[~, bestIdx] = max([planes.score]);
best = planes(bestIdx);
scale = cameraHeightMeters / best.medianHeightSlam;

result = best;
result.scale = scale;
result.cameraHeightMeters = cameraHeightMeters;
result.csvPath = string(csvPath);
result.numMapPoints = nMP;
result.numCameraCenters = size(cameras, 1);
result.allPlanes = planes;

fprintf("\nMATLAB ground-plane estimate\n");
fprintf("  csv: %s\n", csvPath);
fprintf("  unique map points: %d\n", nMP);
fprintf("  keyframe camera centers: %d\n", size(cameras, 1));
fprintf("  plane: n=(%.8g, %.8g, %.8g), d=%.8g\n", best.normal(1), best.normal(2), best.normal(3), best.d);
fprintf("  score: %.4f\n", best.score);
fprintf("  inliers: %d (%.2f%%)\n", numel(best.inlierIdx), 100 * numel(best.inlierIdx) / nMP);
fprintf("  bottom image ratio: %.2f%%\n", 100 * best.bottomScore);
fprintf("  camera side ratio: %.2f%%\n", 100 * best.cameraSideRatio);
fprintf("  height consistency: %.4f\n", best.heightConsistency);
fprintf("  spatial coverage: %.4f\n", best.coverage);
fprintf("  median camera-plane height: %.8g SLAM units\n", best.medianHeightSlam);
fprintf("  MAD camera-plane height: %.8g SLAM units\n", best.madHeightSlam);
fprintf("  real camera height: %.8g m\n", cameraHeightMeters);
fprintf("  metric scale to apply to translations: %.8g m / SLAM unit\n", scale);

if strlength(string(opts.OutputDir)) > 0
    outDir = string(opts.OutputDir);
    if ~exist(outDir, "dir")
        [madeDir, mkdirMsg] = mkdir(outDir);
        if ~madeDir
            warning("Could not create OutputDir '%s': %s. Skipping summary files.", outDir, mkdirMsg);
            outDir = "";
        end
    end
    if strlength(outDir) > 0
        write_summary(fullfile(outDir, "matlab_selected_plane_summary.txt"), result);
        writematrix(points(best.inlierIdx, :), fullfile(outDir, "matlab_selected_plane_inliers_xyz.csv"));
    end
end

if opts.Visualize
    visualize_plane(points, cameras, best);
end

end

function candidate = score_plane(points, cameras, bottomRatio, inlierIdx, normal, d)
cameraSigned = cameras * normal(:) + d;
if median(cameraSigned) < 0
    normal = -normal;
    d = -d;
    cameraSigned = -cameraSigned;
end

cameraDistances = abs(cameraSigned);
medianHeight = median(cameraDistances);
madHeight = median(abs(cameraDistances - medianHeight));
heightConsistency = 1 / (1 + madHeight / max(medianHeight, eps));
cameraSideRatio = mean(cameraSigned > 0);

bottomScore = mean(bottomRatio(inlierIdx));
inlierRatio = numel(inlierIdx) / size(points, 1);
coverage = plane_coverage(points(inlierIdx, :));

score = 1.50 * inlierRatio ...
      + 1.25 * bottomScore ...
      + 1.25 * cameraSideRatio ...
      + 1.00 * heightConsistency ...
      + 0.75 * coverage;

candidate = struct( ...
    "normal", normal, ...
    "d", d, ...
    "score", score, ...
    "bottomScore", bottomScore, ...
    "cameraSideRatio", cameraSideRatio, ...
    "heightConsistency", heightConsistency, ...
    "coverage", coverage, ...
    "medianHeightSlam", medianHeight, ...
    "madHeightSlam", madHeight, ...
    "inlierIdx", inlierIdx(:));
end

function coverage = plane_coverage(inlierPoints)
if size(inlierPoints, 1) < 3
    coverage = 0;
    return;
end

centered = inlierPoints - mean(inlierPoints, 1);
[~, s, v] = svd(centered, "econ");
singularValues = diag(s);
if numel(singularValues) < 2 || singularValues(1) <= eps
    coverage = 0;
    return;
end

uv = centered * v(:, 1:2);
mins = min(uv, [], 1);
spans = max(max(uv, [], 1) - mins, eps);
gridSize = 8;
ij = floor((uv - mins) ./ spans * gridSize) + 1;
ij = max(1, min(gridSize, ij));
occupied = unique(ij, "rows");
occupancy = size(occupied, 1) / (gridSize * gridSize);
shape = min(singularValues(2) / singularValues(1), 1);
coverage = sqrt(occupancy * shape);
end

function visualize_plane(points, cameras, best)
figure("Name", "Ground plane candidate");
hold on;
axis equal;
grid on;

colors = repmat([0.45 0.45 0.45], size(points, 1), 1);
colors(best.inlierIdx, :) = repmat([0.05 0.8 0.2], numel(best.inlierIdx), 1);
pcshow(pointCloud(points, "Color", uint8(255 * colors)), "MarkerSize", 35);
plot3(cameras(:, 1), cameras(:, 2), cameras(:, 3), "bo", "MarkerSize", 6, "MarkerFaceColor", "b");

patchVerts = plane_patch(points(best.inlierIdx, :), best.normal, best.d);
patch("Vertices", patchVerts, "Faces", [1 2 3 4], ...
      "FaceColor", [1.0 0.82 0.1], "FaceAlpha", 0.35, "EdgeColor", [0.7 0.5 0.0]);

title("Selected ground-plane candidate: green=inliers, gray=other points, blue=cameras");
xlabel("x"); ylabel("y"); zlabel("z");
hold off;
end

function verts = plane_patch(inlierPoints, normal, d)
center = mean(inlierPoints, 1);
normal = normal(:)' / max(norm(normal), eps);
axis = [1 0 0];
if abs(dot(axis, normal)) > 0.9
    axis = [0 1 0];
end
u = cross(normal, axis);
u = u / max(norm(u), eps);
v = cross(normal, u);
v = v / max(norm(v), eps);

uv = [(inlierPoints - center) * u(:), (inlierPoints - center) * v(:)];
extent = prctile(abs(uv), 95, 1);
extent = max(extent, [0.1 0.1]);

centerOnPlane = center - (dot(normal, center) + d) * normal;
verts = [
    centerOnPlane - extent(1) * u - extent(2) * v
    centerOnPlane + extent(1) * u - extent(2) * v
    centerOnPlane + extent(1) * u + extent(2) * v
    centerOnPlane - extent(1) * u + extent(2) * v
];
end

function write_summary(path, result)
fid = fopen(path, "w");
if fid < 0
    warning("Could not write summary file: %s", path);
    return;
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "MATLAB ground-plane estimate\n");
fprintf(fid, "csv: %s\n", result.csvPath);
fprintf(fid, "num_map_points: %d\n", result.numMapPoints);
fprintf(fid, "num_camera_centers: %d\n", result.numCameraCenters);
fprintf(fid, "normal: %.12g %.12g %.12g\n", result.normal(1), result.normal(2), result.normal(3));
fprintf(fid, "d: %.12g\n", result.d);
fprintf(fid, "score: %.12g\n", result.score);
fprintf(fid, "inliers: %d\n", numel(result.inlierIdx));
fprintf(fid, "bottom_image_ratio: %.12g\n", result.bottomScore);
fprintf(fid, "camera_side_ratio: %.12g\n", result.cameraSideRatio);
fprintf(fid, "height_consistency: %.12g\n", result.heightConsistency);
fprintf(fid, "spatial_coverage: %.12g\n", result.coverage);
fprintf(fid, "median_camera_plane_height_slam: %.12g\n", result.medianHeightSlam);
fprintf(fid, "mad_camera_plane_height_slam: %.12g\n", result.madHeightSlam);
fprintf(fid, "camera_height_m: %.12g\n", result.cameraHeightMeters);
fprintf(fid, "scale_m_per_slam_unit: %.12g\n", result.scale);
end
