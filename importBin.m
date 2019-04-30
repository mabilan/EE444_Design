% Michael Bilan & Kyle Tam
% EE 444 Design Project
% Temp. Sensor to NFC Tag Reader
%
% Simple program that reads binary data file exported from ST25 NFC
% tag reader (mobile application) for processing.
% Resulting data structure is based on NFC Device:
%   - ST25DV04K: 512 byte *.bin files
%   - ST25DV16K: 2084 byte *.bin files
%   - ST25DV64K: 8192 byte *.bin files


% Set relative file path - all binary files should be stored in this local
%   directory
filePath = '.\binfiles\';

% Set file naming scheme to be read - default is set to 'data'
%   THIS PARAMETER MUST BE USER-DEFINED
fileNameScheme = 'data';

% Set file naming scheme index values to read between (e.g. data to data5)
%   THESE PARAMETERS MUST BE USER-DEFINED
fileIndexStart = 1;
fileIndexEnd = 5;

% Set binary data type to be read
% Default value corresponds to data type in our implementation
format = 'uint8';

% Empty matrix for data storage
Data = [];

for i = fileIndexStart:fileIndexEnd
    % Create full file location from path and name
    filename = fullfile(filePath, [fileNameScheme num2str(i) '.bin']);

    % Open file:
    %   - set to read-only
    fileID = fopen(filename, 'r');

    if fileID == -1
        error('Cannot open file: %s. Terminating session.', filename);
        break;
    end
    
    % Read current file data
    TempData = fread(fileID, Inf, format);
    
    % Concatenate new data with previously stored data
    Data = [Data; TempData];
    
    fclose(fileID);
end

plot(Data);
ylim([0 100]);
ylabel('Temperature (Degrees C)');
xlabel('Time (s)');
title('Data Collection: Temp. vs. Time');