function [ Parameter ] = importHalbachWheelParameters()
%  importHalbachWheelParameters Imports the Halbach wheel parameters
%  Inputs:
%  Output: 
%    [ Parameter ]      Parameter structure containing imported variables
%  @author              ?, HypED
%  Modified:            Rafael Anderka 02/11/2018
%  Modified:            Simona Prokopovic 07/11/2018

    % Set filepath to the excel workbook
    params_filepath = './Parameters/HalbachWheel_parameters.xlsx';

    % Import spreadsheets using the importSpreadsheet function
    % Input arguments of importSpreadsheet:
    %   1 Filepath to .xlsx, 
    %   2 spreadsheet name, 
    %   3 range of ROWS of the variable names to be imported
    %   4 COLUMN of the variable names to be imported
    %   5 range of ROWS of the variable values to be imported
    %   6 COLUMN of the variable values to be imported
    Constants = importSpreadsheet(params_filepath, 'Constants', 3:8,  3, 3:8,  4);
    Variables = importSpreadsheet(params_filepath, 'Variables', 3:4,  3, 3:4,  4);
    Tube      = importSpreadsheet(params_filepath, 'Tube',      3:3,  3, 3:3,  4);
    Pod       = importSpreadsheet(params_filepath, 'Pod',       3:11, 3, 3:11, 4);

    % Put all variables in a single cell array and create a parameter structure
    FullArray = [Constants;Variables;Tube;Pod];                     % Create a single array with all variables
    for index = 1 : length(FullArray)
        if (~isnan(FullArray{index,2}))                             % Check if variable value (column 2) is valid
             Parameter.(FullArray{index,1}) = FullArray{index,2};   % Assign variable value (column 2) to corresponding variable name (column 1)
        end 
    end
end