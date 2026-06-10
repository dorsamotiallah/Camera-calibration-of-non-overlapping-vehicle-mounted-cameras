function result = estimate_keyframe_scales_from_atlas_csv(csvPath, cameraHeightMeters, varargin)
%ESTIMATE_KEYFRAME_SCALES_FROM_ATLAS_CSV Estimate local keyframe scale CSV.
%
% Reads the atlas_observations.csv exported from NMC3D/ORB-SLAM atlas data.
% For each keyframe, it fits a local ground-like plane using MATLAB
% pcfitplane, computes the camera-to-plane height in SLAM units, and writes a
% per-keyframe scale CSV that calibration code can consume later.
%
% Example:
%   result = estimate_keyframe_scales_from_atlas_csv( ...
%       "results_ground_scale/back_mono/atlas_observations.csv", 0.66, ...
%       "OutputCsv", "results_ground_scale/back_mono/keyframe_scale_matlab.csv", ...
%       "MaxDistance", 0.03, ...
%       "BottomThreshold", 0.55, ...
%       "ReferenceNormal", [0 -1 0]);

parser = inputParser;
parser.addRequired("csvPath", @(x) ischar(x) || isstring(x));
parser.addRequired("cameraHeightMeters", @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("OutputCsv", "", @(x) ischar(x) || isstring(x));
parser.addParameter("MaxDistance", 0.03, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("BottomThreshold", 0.55, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
parser.addParameter("NumPlanes", 6, @(x) isnumeric(x) && isscalar(x) && x >= 1);
parser.addParameter("MinInliers", 20, @(x) isnumeric(x) && isscalar(x) && x >= 3);
parser.addParameter("MinPlaneScore", 2.5, @(x) isnumeric(x) && isscalar(x));
parser.addParameter("UseBottomCandidates", true, @(x) islogical(x) && isscalar(x));
parser.addParameter("ReferenceNormal", [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 3));
parser.addParameter("MaxAngularDistance", 20, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("SmoothingWindow", 5, @(x) isnumeric(x) && isscalar(x) && x >= 1);
parser.addParameter("ScaleLowChange", 0.10, @(x) isnumeric(x) && isscalar(x) && x >= 0);
parser.addParameter("ScaleHighChange", 0.50, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter("Visualize", false, @(x) islogical(x) && isscalar(x));
parser.parse(csvPath, cameraHeightMeters, varargin{:});
opts = parser.Results;

if exist("pcfitplane", "file") ~= 2
    error("pcfitplane was not found. MATLAB Computer Vision Toolbox is required.");
end

obs = readtable(string(csvPath));
if height(obs) == 0
    error("Observation CSV is empty: %s", csvPath);
end

[kfIds, firstRows, kfGroup] = unique(obs.kf_id, "stable");
nKf = numel(kfIds);
rows = initialize_rows(nKf);
acceptedScales = [];

for iKf = 1:nKf
    idx = find(kfGroup == iKf);
    kfObs = obs(idx, :);
    [candidate, nObs, nCandidates, reason] = fit_keyframe_plane(kfObs, opts);

    rows(iKf).kf_id = kfIds(iKf);
    rows(iKf).kf_time = obs.kf_time(firstRows(iKf));
    rows(iKf).num_observations = nObs;
    rows(iKf).num_candidates = nCandidates;

    if isempty(candidate)
        rows(iKf).reason = string(reason);
        rows(iKf).smoothed_scale = current_smooth(acceptedScales);
        continue;
    end

    rawScale = cameraHeightMeters / candidate.heightSlam;
    [accepted, smoothed, reason] = accept_and_smooth(rawScale, candidate.score, acceptedScales, opts);
    if accepted
        acceptedScales(end + 1) = rawScale; %#ok<AGROW>
    end

    rows(iKf).raw_scale = rawScale;
    rows(iKf).smoothed_scale = smoothed;
    rows(iKf).height_slam = candidate.heightSlam;
    rows(iKf).plane_score = candidate.score;
    rows(iKf).num_inliers = candidate.numInliers;
    rows(iKf).inlier_ratio = candidate.inlierRatio;
    rows(iKf).bottom_ratio = candidate.bottomRatio;
    rows(iKf).camera_side = candidate.cameraSide;
    rows(iKf).coverage = candidate.coverage;
    rows(iKf).accepted = accepted;
    rows(iKf).reason = string(reason);
end

result = struct();
result.csvPath = string(csvPath);
result.cameraHeightMeters = cameraHeightMeters;
result.table = struct2table(rows);
result.accepted = result.table(result.table.accepted == true, :);

fprintf("\nMATLAB per-keyframe scale estimate\n");
fprintf("  csv: %s\n", csvPath);
fprintf("  keyframes: %d\n", nKf);
fprintf("  accepted: %d\n", height(result.accepted));
fprintf("  rejected: %d\n", nKf - height(result.accepted));
if height(result.accepted) > 0
    raw = result.accepted.raw_scale;
    fprintf("  raw scale median: %.8g m / SLAM unit\n", median(raw));
    fprintf("  raw scale MAD: %.8g\n", median(abs(raw - median(raw))));
    fprintf("  smoothed final scale: %.8g m / SLAM unit\n", result.accepted.smoothed_scale(end));
    fprintf("  median camera-plane height: %.8g SLAM units\n", median(result.accepted.height_slam));
end

if strlength(string(opts.OutputCsv)) > 0
    outPath = string(opts.OutputCsv);
    outDir = fileparts(outPath);
    if strlength(outDir) > 0 && exist(outDir, "dir") ~= 7
        mkdir(outDir);
    end
    writetable(result.table, outPath);
    fprintf("  per-keyframe CSV: %s\n", outPath);
end

if opts.Visualize
    visualize_keyframe_scales(result.table);
end
end

function rows = initialize_rows(nRows)
template = struct( ...
    "kf_id", 0, ...
    "kf_time", NaN, ...
    "raw_scale", NaN, ...
    "smoothed_scale", NaN, ...
    "height_slam", NaN, ...
    "plane_score", 0, ...
    "num_observations", 0, ...
    "num_candidates", 0, ...
    "num_inliers", 0, ...
    "inlier_ratio", 0, ...
    "bottom_ratio", 0, ...
    "camera_side", 0, ...
    "coverage", 0, ...
    "accepted", false, ...
    "reason", "not_processed");
rows = repmat(template, nRows, 1);
end

function [candidate, nObs, nCandidates, reason] = fit_keyframe_plane(kfObs, opts)
[~, uniqueRows] = unique(kfObs.mp_id, "stable");
kfObs = kfObs(uniqueRows, :);
nObs = height(kfObs);

points = [kfObs.pw_x, kfObs.pw_y, kfObs.pw_z];
imgHeight = max(kfObs.img_max_y - kfObs.img_min_y, 1);
yNorm = (kfObs.kp_y - kfObs.img_min_y) ./ imgHeight;
camera = [kfObs.cx(1), kfObs.cy(1), kfObs.cz(1)];

candidateMask = true(nObs, 1);
if opts.UseBottomCandidates
    candidateMask = yNorm >= opts.BottomThreshold;
end
candidatePoints = points(candidateMask, :);
candidateY = yNorm(candidateMask);
nCandidates = size(candidatePoints, 1);

candidate = [];
reason = "ok";
if nCandidates < opts.MinInliers
    reason = "too_few_bottom_candidates";
    return;
end

remaining = (1:nCandidates)';
for iPlane = 1:opts.NumPlanes
    if numel(remaining) < opts.MinInliers
        break;
    end

    cloud = pointCloud(candidatePoints(remaining, :));
    try
        if isempty(opts.ReferenceNormal)
            [model, localInliers] = pcfitplane(cloud, opts.MaxDistance);
        else
            ref = double(reshape(opts.ReferenceNormal, 1, 3));
            ref = ref / max(norm(ref), eps);
            [model, localInliers] = pcfitplane(cloud, opts.MaxDistance, ref, opts.MaxAngularDistance);
        end
    catch err
        warning("pcfitplane failed for keyframe %.0f candidate %d: %s", kfObs.kf_id(1), iPlane, err.message);
        reason = "ransac_failed";
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
    localCandidate = score_local_plane(candidatePoints, candidateY, camera, inlierIdx, normal, d, opts.BottomThreshold);

    if isempty(candidate) || localCandidate.score > candidate.score
        candidate = localCandidate;
    end
    remaining(localInliers) = [];
end

if isempty(candidate)
    reason = "ransac_failed";
end
end

function candidate = score_local_plane(points, yNorm, camera, inlierIdx, normal, d, bottomThreshold)
cameraSigned = camera * normal(:) + d;
if cameraSigned < 0
    normal = -normal;
    d = -d;
    cameraSigned = -cameraSigned;
end

heightSlam = abs(cameraSigned);
cameraSide = double(cameraSigned > 0);
bottomRatio = mean(yNorm(inlierIdx) >= bottomThreshold);
inlierRatio = numel(inlierIdx) / size(points, 1);
coverage = plane_coverage(points(inlierIdx, :));

score = 1.50 * inlierRatio ...
      + 1.25 * bottomRatio ...
      + 1.25 * cameraSide ...
      + 0.75 * coverage;

candidate = struct( ...
    "normal", normal, ...
    "d", d, ...
    "heightSlam", heightSlam, ...
    "score", score, ...
    "numInliers", numel(inlierIdx), ...
    "inlierRatio", inlierRatio, ...
    "bottomRatio", bottomRatio, ...
    "cameraSide", cameraSide, ...
    "coverage", coverage);
end

function [accepted, smoothed, reason] = accept_and_smooth(rawScale, score, acceptedScales, opts)
accepted = true;
reason = "accepted";

if ~isfinite(rawScale) || rawScale <= 0
    accepted = false;
    smoothed = current_smooth(acceptedScales);
    reason = "invalid_scale";
    return;
end

if score < opts.MinPlaneScore
    accepted = false;
    smoothed = current_smooth(acceptedScales);
    reason = "low_plane_score";
    return;
end

if isempty(acceptedScales)
    smoothed = rawScale;
    return;
end

meanRecent = mean(acceptedScales(max(1, end - opts.SmoothingWindow + 1):end));
change = abs(rawScale - meanRecent) / max(abs(meanRecent), eps);
if change > opts.ScaleHighChange
    accepted = false;
    smoothed = meanRecent;
    reason = "scale_jump_rejected";
    return;
end

if change <= opts.ScaleLowChange
    rawWeight = 1.0;
else
    span = max(opts.ScaleHighChange - opts.ScaleLowChange, eps);
    alpha = min(max((change - opts.ScaleLowChange) / span, 0), 1);
    rawWeight = 1.0 - alpha;
end
smoothed = rawWeight * rawScale + (1.0 - rawWeight) * meanRecent;
end

function smoothed = current_smooth(acceptedScales)
if isempty(acceptedScales)
    smoothed = NaN;
else
    smoothed = acceptedScales(end);
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

function visualize_keyframe_scales(scaleTable)
figure("Name", "MATLAB per-keyframe scale curve");
accepted = scaleTable.accepted == true;
plot(scaleTable.kf_id, scaleTable.raw_scale, ".", "Color", [0.6 0.6 0.6]);
hold on;
plot(scaleTable.kf_id(accepted), scaleTable.raw_scale(accepted), "bo");
plot(scaleTable.kf_id, scaleTable.smoothed_scale, "r-");
grid on;
xlabel("Keyframe id");
ylabel("Scale, m / SLAM unit");
legend("raw", "accepted raw", "smoothed", "Location", "best");
title("Per-keyframe local scale estimates");
end
