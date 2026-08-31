% path_generator.m 鈥?ONE-TIME PATH GENERATION TOOL (offline, not part of build)
%
% This MATLAB script pre-computes the shared candidate path set for the CMU
% local planner. It generates the three PLY data files in this directory:
%   paths.ply          鈥?343 candidate paths 脳 36 rotation directions
%   pathList.ply       鈥?group_id mapping for each path_id
%   startPaths.ply     鈥?start-segment for each of the 7 path groups
%
% generate_library.py then creates one collision-correspondence library for
% each robot profile. The generated files are consumed at runtime by the C++
% CMU backend (via Planner::configure()).
%
% To regenerate the path set (e.g. after changing vehicle geometry):
%   1. Open MATLAB, run this script.
%   2. Run: python generate_library.py
%   3. Commit the updated robot directories.
%   4. Rebuild the native endpoint: bash scripts/build/build_nav_endpoint.sh
%
% DO NOT delete this file 鈥?it is the canonical specification of the path set.

clc;
clear all;
close all;
set(0, 'DefaultFigureVisible', 'off');

outputDir = getenv('LINGTU_CMU_OUTPUT_DIR');
if isempty(outputDir)
    outputDir = pwd;
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

%% generate path
%{.
dis = 1.0;
angle = 27;
deltaAngle = angle / 3;
scale = 0.65;

pathStartAll = zeros(4, 0);
pathAll = zeros(5, 0);
pathList = zeros(5, 0);
pathID = 0;
groupID = 0;

figure;
hold on;
box on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

fprintf('\nGenerating paths\n');

for shift1 = -angle : deltaAngle : angle
    wayptsStart = [0, 0, 0;
                   dis, shift1, 0];

    pathStartR = 0 : 0.01 : dis;
    pathStartShift = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);

    pathStartX = pathStartR .* cos(pathStartShift * pi / 180);
    pathStartY = pathStartR .* sin(pathStartShift * pi / 180);
    pathStartZ = zeros(size(pathStartX));

    pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID];
    pathStartAll = [pathStartAll, pathStart];

    for shift2 = -angle * scale + shift1 : deltaAngle * scale : angle * scale + shift1
        for shift3 = -angle * scale^2 + shift2 : deltaAngle * scale^2 : angle * scale^2 + shift2
                waypts = [pathStartR', pathStartShift', pathStartZ';
                          2 * dis, shift2, 0;
                          3 * dis - 0.001, shift3, 0;
                          3 * dis, shift3, 0];

                pathR = 0 : 0.01 : waypts(end, 1);
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];

                pathID = pathID + 1;

                plot3(pathX, pathY, pathZ);
        end
    end

    groupID = groupID + 1
end

pathID

fileID = fopen(fullfile(outputDir, 'startPaths.ply'), 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathStartAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d\n', pathStartAll);
fclose(fileID);

fileID = fopen(fullfile(outputDir, 'paths.ply'), 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathAll);
fclose(fileID);

fileID = fopen(fullfile(outputDir, 'pathList.ply'), 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathList, 2));
fprintf(fileID, 'property float end_x\n');
fprintf(fileID, 'property float end_y\n');
fprintf(fileID, 'property float end_z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathList);
fclose(fileID);

pause(1.0);
%}

fprintf('\nShared path generation complete. Run generate_library.py next.\n');
