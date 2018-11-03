function VarArray = importSpreadsheet(filepath, spreadsheet, nameRows, nameColumn, valueRows, valueColumn)
% importSpreadsheet Function to import variables from a specific excel 
%                   spreadsheet. In the excel spreadsheet variable names 
%                   and variable values need to be in the SAME ORDER.
% Inputs: 
%   filepath        Filepath to .xlsx, 
%   spreadsheet     spreadsheet name, 
%   nameRows        range of ROWS of the variable names to be imported
%   nameColumn      COLUMN of the variable names to be imported
%   valueRows       range of ROWS of the variable values to be imported
%   valueColumn     COLUMN of the variable values to be imported
% Output: 
%   VarArray       Array of variable names and corresponding values
% @author          Rafael Anderka
%                  HypED, 02/11/2018
% Modified:        -

% Import the raw data from spreadsheet
[~, ~, raw] = xlsread(filepath,spreadsheet);

% Get variable names from specified range
Variables = raw(nameRows,nameColumn);                                              % Read names from raw spreadsheet data
Variables(cellfun(@(x) ~isempty(x) && isnumeric(x) && isnan(x),Variables)) = {''}; % Check that each entry is valid

% Get variable values from specified range
Values = raw(valueRows,valueColumn);                                               % Read values from raw spreadsheet data
Values = reshape([Values{:}],size(Values));                                        % Reshape values matrix

clearvars raw;

% Put the variable names and corresponding values in a single array
VarArray = [Variables num2cell(Values)];

end