function result = estimate_ground_plane_traj(csvPath, cameraHeightMeters, varargin)
%ESTIMATE_GROUND_PLANE_WITH_TRAJECTORY_CONSTRAINT Ground scale with trajectory-plane parallelism.
%
% This is a separate experimental variant of estimate_ground_plane_from_atlas_csv.
% It fits a plane to the keyframe camera centers, then prefers ground-plane
% candidates whose normals are parallel to that trajectory plane.

parser = inputParser;
parser.addRequired("csvPath", @(x) ischar(x) || isstring(x));
parser.addRequired("cameraHeightMeters", @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("MaxDistance", 0.03, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("BottomThreshold", 0.55, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
parser.addParameter("NumPlanes", 8, @(x) isnumeric(x) && isscalar(x) && x >= 1);
parser.addParameter("MinInliers", 50, @(x) isnumeric(x) && isscalar(x) && x >= 3);
parser.addParameter("UseBottomCandidates", true, @(x) islogical(x) && isscalar(x));
parser.addParameter("MaxParallelAngle", 15, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("Visualize", true, @(x) islogical(x) && isscalar(x));
parser.parse(csvPath, cameraHeightMeters, varargin{:});
opts = parser.Results;

if exist("pcfitplane", "file") ~= 2
    error("pcfitplane was not found. MATLAB Computer Vision Toolbox is required.");
end

obs = readtable(string(csvPath));
if height(obs) == 0
    error("Observation CSV is empty: %s", csvPath);
end

[mpIds, ~, mpGroup] = unique(obs.mp_id); %#ok<ASGLU>
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
[trajNormal, trajD, trajShape] = fit_plane_svd(cameras);

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
        [model, localInliers] = pcfitplane(cloud, opts.MaxDistance);
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

    candidate = score_plane(points, cameras, bottomRatio, inlierIdx, normal, d, trajNormal, trajD);
    planes = [planes; candidate]; %#ok<AGROW>
    remaining(localInliers) = [];
end

if isempty(planes)
    error("No valid plane candidates found.");
end

maxAngle = opts.MaxParallelAngle;
parallelOk = [planes.parallelAngleDeg] <= maxAngle;
if any(parallelOk)
    validIdx = find(parallelOk);
    [~, localBest] = max([planes(validIdx).score]);
    bestIdx = validIdx(localBest);
else
    warning("No candidate satisfied MaxParallelAngle=%.3g deg; using best score anyway.", maxAngle);
    [~, bestIdx] = max([planes.score]);
end

best = planes(bestIdx);
scaleMedian = cameraHeightMeters / best.medianHeightSlam;
scalePlaneDistance = cameraHeightMeters / best.trajectoryPlaneDistanceSlam;

result = best;
result.scale = scaleMedian;
result.scaleFromTrajectoryPlaneDistance = scalePlaneDistance;
result.cameraHeightMeters = cameraHeightMeters;
result.csvPath = string(csvPath);
result.numMapPoints = nMP;
result.numCameraCenters = size(cameras, 1);
result.trajectoryNormal = trajNormal;
result.trajectoryD = trajD;
result.trajectoryShape = trajShape;
result.allPlanes = planes;

fprintf("\nMATLAB trajectory-constrained ground-plane estimate\n");
fprintf("  csv: %s\n", csvPath);
fprintf("  unique map points: %d\n", nMP);
fprintf("  keyframe camera centers: %d\n", size(cameras, 1));
fprintf("  trajectory plane: n=(%.8g, %.8g, %.8g), d=%.8g, shape=%.4f\n", ...
    trajNormal(1), trajNormal(2), trajNormal(3), trajD, trajShape);
fprintf("  selected candidate: %d of %d\n", bestIdx, numel(planes));
fprintf("  ground plane: n=(%.8g, %.8g, %.8g), d=%.8g\n", best.normal(1), best.normal(2), best.normal(3), best.d);
fprintf("  score: %.4f\n", best.score);
fprintf("  parallel angle: %.4f deg\n", best.parallelAngleDeg);
fprintf("  inliers: %d (%.2f%%)\n", numel(best.inlierIdx), 100 * numel(best.inlierIdx) / nMP);
fprintf("  bottom image ratio: %.2f%%\n", 100 * best.bottomScore);
fprintf("  camera side ratio: %.2f%%\n", 100 * best.cameraSideRatio);
fprintf("  height consistency: %.4f\n", best.heightConsistency);
fprintf("  spatial coverage: %.4f\n", best.coverage);
fprintf("  median camera-plane height: %.8g SLAM units\n", best.medianHeightSlam);
fprintf("  trajectory-plane distance: %.8g SLAM units\n", best.trajectoryPlaneDistanceSlam);
fprintf("  real camera height: %.8g m\n", cameraHeightMeters);
fprintf("  scale from median camera height: %.8g m / SLAM unit\n", scaleMedian);
fprintf("  scale from plane-plane distance: %.8g m / SLAM unit\n", scalePlaneDistance);

if numel(planes) > 1
    fprintf("\nTrajectory-constrained candidates\n");
    for i = 1:numel(planes)
        p = planes(i);
        fprintf("  %d: score=%.4f, angle=%.2f deg, inliers=%d, height=%.6g, planeDist=%.6g, scale=%.6g\n", ...
            i, p.score, p.parallelAngleDeg, numel(p.inlierIdx), p.medianHeightSlam, ...
            p.trajectoryPlaneDistanceSlam, cameraHeightMeters / p.medianHeightSlam);
    end
end

if opts.Visualize
    visualize_plane(points, cameras, best, trajNormal, trajD);
end

end

function candidate = score_plane(points, cameras, bottomRatio, inlierIdx, normal, d, trajNormal, trajD)
cameraSigned = cameras * normal(:) + d;
if median(cameraSigned) < 0
    normal = -normal;
    d = -d;
    cameraSigned = -cameraSigned;
end

if dot(normal, trajNormal) < 0
    trajNormal = -trajNormal;
    trajD = -trajD;
end

cameraDistances = abs(cameraSigned);
medianHeight = median(cameraDistances);
madHeight = median(abs(cameraDistances - medianHeight));
heightConsistency = 1 / (1 + madHeight / max(medianHeight, eps));
cameraSideRatio = mean(cameraSigned > 0);

bottomScore = mean(bottomRatio(inlierIdx));
inlierRatio = numel(inlierIdx) / size(points, 1);
coverage = plane_coverage(points(inlierIdx, :));
parallelCos = min(max(abs(dot(normal, trajNormal)), 0), 1);
parallelAngleDeg = acosd(parallelCos);
parallelScore = parallelCos;

trajectoryPlaneDistance = abs(d - trajD);
if trajectoryPlaneDistance <= eps
    trajectoryPlaneDistance = medianHeight;
end

score = 1.25 * inlierRatio ...
      + 1.00 * bottomScore ...
      + 1.00 * cameraSideRatio ...
      + 1.00 * heightConsistency ...
      + 0.75 * coverage ...
      + 1.75 * parallelScore;

candidate = struct( ...
    "normal", normal, ...
    "d", d, ...
    "score", score, ...
    "bottomScore", bottomScore, ...
    "cameraSideRatio", cameraSideRatio, ...
    "heightConsistency", heightConsistency, ...
    "coverage", coverage, ...
    "parallelScore", parallelScore, ...
    "parallelAngleDeg", parallelAngleDeg, ...
    "medianHeightSlam", medianHeight, ...
    "madHeightSlam", madHeight, ...
    "trajectoryPlaneDistanceSlam", trajectoryPlaneDistance, ...
    "inlierIdx", inlierIdx(:));
end

function [normal, d, shape] = fit_plane_svd(points)
center = mean(points, 1);
centered = points - center;
[~, s, v] = svd(centered, "econ");
normal = v(:, end)';
normal = normal / max(norm(normal), eps);
d = -dot(normal, center);
singularValues = diag(s);
if numel(singularValues) < 2 || singularValues(1) <= eps
    shape = 0;
else
    shape = min(singularValues(2) / singularValues(1), 1);
end
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

function visualize_plane(points, cameras, best, trajNormal, trajD)
figure("Name", "Trajectory-constrained ground plane");
hold on;
axis equal;
grid on;

colors = repmat([0.45 0.45 0.45], size(points, 1), 1);
colors(best.inlierIdx, :) = repmat([0.05 0.8 0.2], numel(best.inlierIdx), 1);
pcshow(pointCloud(points, "Color", uint8(255 * colors)), "MarkerSize", 35);
plot3(cameras(:, 1), cameras(:, 2), cameras(:, 3), "bo", "MarkerSize", 6, "MarkerFaceColor", "b");

groundVerts = plane_patch(points(best.inlierIdx, :), best.normal, best.d);
patch("Vertices", groundVerts, "Faces", [1 2 3 4], ...
      "FaceColor", [1.0 0.82 0.1], "FaceAlpha", 0.35, "EdgeColor", [0.7 0.5 0.0]);

trajVerts = plane_patch(cameras, trajNormal, trajD);
patch("Vertices", trajVerts, "Faces", [1 2 3 4], ...
      "FaceColor", [0.1 0.45 1.0], "FaceAlpha", 0.20, "EdgeColor", [0.05 0.25 0.7]);

title("Ground plane with trajectory-plane constraint");
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



