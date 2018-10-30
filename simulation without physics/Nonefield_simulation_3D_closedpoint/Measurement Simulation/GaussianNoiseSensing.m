function [ MeasuredPosition,LastMeasureTime ] = GaussianNoiseSensing( Position, time, LastMeasureTime,LastMeasuredPosition )
%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification: 6/4/2018

%PERFECTSENSING This funtion simulate a noisy position sensing


MeasurePeriod=0.002; %[s]

if (time-LastMeasureTime)>MeasurePeriod

    VarianceOfNoise=0.0001; %meters

    NoiseXYPosition = VarianceOfNoise.*randn(1,3);

    MeasuredPosition=Position+NoiseXYPosition;
    LastMeasureTime=time;

else
    
    MeasuredPosition=LastMeasuredPosition;
    LastMeasureTime=LastMeasureTime;
end

