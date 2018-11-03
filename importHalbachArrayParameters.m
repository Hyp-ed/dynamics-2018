function [ Parameter ] = importHalbachArrayParameters()
%  importHalbachArrayParameters Imports the Halbach array parameters
%  Inputs:
%  Output: 
%    [ Parameter ]      Parameter structure containing imported variables
%  @author              Emil Hannsen, Ivan Chan
%                       HypED 28/01/2017
%  Modified:            Ivan Chan (11/04/2017)
%  Modified:            Andreas Malekos (01/11/2017)
%  Modified:            Rafael Anderka (02/11/2018)

% Set filepath to the excel workbook
params_filepath = './Parameters/HalbachArray_parameters.xlsx';

% Import spreadsheets using the importSpreadsheet function
% Input arguments of importSpreadsheet:
%   1 Filepath to .xlsx, 
%   2 spreadsheet name, 
%   3 range of ROWS of the variable names to be imported
%   4 COLUMN of the variable names to be imported
%   5 range of ROWS of the variable values to be imported
%   6 COLUMN of the variable values to be imported
Constants = importSpreadsheet(params_filepath, 'Constants',         3:13, 3, 3:13, 4);
Magnets   = importSpreadsheet(params_filepath, 'Magnets',           3:11, 3, 3:11, 4);
LevArr    = importSpreadsheet(params_filepath, 'Levitation Arrays', 9:13, 3, 9:13, 4);
BrakeArr  = importSpreadsheet(params_filepath, 'Braking Arrays',     9:11, 3, 9:11, 4);
Traj      = importSpreadsheet(params_filepath, 'Trajectory',        3:5,  3, 3:5,  4);
CoM       = importSpreadsheet(params_filepath, 'Centre of Mass',    1:10, 1, 1:10, 3);
Susp      = importSpreadsheet(params_filepath, 'Suspension',        3:5,  3, 3:5,  4);

% Put all variables in a single cell array and create a parameter structure
FullArray = [Constants;Magnets;LevArr;BrakeArr;Traj;CoM;Susp];  % Create a single array with all variables
for index = 1 : length(FullArray)
    if (~isnan(FullArray{index,2}))                             % Check if variable value (column 2) is valid
         Parameter.(FullArray{index,1}) = FullArray{index,2};   % Assign variable value (column 2) to corresponding variable name (column 1)
    end
end
end