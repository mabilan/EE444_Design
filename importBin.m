% Michael Bilan & Kyle Tam
% EE 444 Design Project
% Temp. Sensor to NFC Tag Reader
%
% Simple program that reads binary data file exported from ST25 NFC
% tag reader (mobile application) for processing.
% Our project uses ST25DV04K:
%   - memory capacity is 512 bytes, reading data.bin should produce
%     a 512x1 matrix of integer values

% Set file absolute path if not in local dir
fPath = 'D:\Users\mabilan\Documents\EE 444\Design Project';

% Set file name to be read - ST25 default is data.bin
fName = 'data.bin';

% Create full file location from path and name
filename = fullfile(fPath, fName);

% Set binary data type to be read
format = 'uint8';

% Open file:
%   - set to read-only
fileID = fopen(filename, 'r');

if fileID == -1
    error('Cannot open file: %s', filename);
end

Data = fread(fileID, Inf, format);
fclose(fileID);