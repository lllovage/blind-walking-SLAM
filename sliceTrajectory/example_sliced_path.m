%EXAMPLE SLICED PATH

%% Example 1:
clear;clc;
stride = 0.25; % In meters
timeRes = 0.0001;   % In seconds
%completePath.xParams = [1 1]';
%completePath.yParams = [1 1]';
completePath.xParams = [0.1 0.1 1 -0.1]';
completePath.yParams = [0.1 -0.1 -0.1]';
completePath.t0 = 0; % In seconds
completePath.tf = 2;
slicedPath = slicePath (completePath,stride,timeRes);
showSliced(completePath,slicedPath)