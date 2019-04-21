% Michael Bilan & Kyle Tam
% EE 444 Design Project
% Temp. Sensor to NFC Tag Reader
%
% Simple program that reads binary data file exported from ST25 NFC
% tag reader (mobile application) for processing.
% Resulting data structure is based on NFC Device:
%   - ST25DV04K: 512 byte *.bin file -> 512x1 matrix
%   - ST25DV16K: 2084 byte *.bin file -> 2084x1 matrix
%   - ST25DV64K: 8192 byte *.bin file -> 8192x1 matrix


% Set file absolute path if not in local dir
fPath = 'C:\Sample\Directory\Path';

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
