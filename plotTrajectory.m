function plotTrajectory(time,distance,velocity,power_input,rpm,torque)
% plotTrajectory   Plots the trajectory graphs from distance and velocity arrays over time
%                  And saves them to 'TrajectoryPlot.fig'
% Inputs:
%   time           Base time array
%   distance       Distance over time array
%   velocity       Velocity over time array
% Output: 
% @author          Rafael Anderka
%                  HypED, 03/11/2018
% Modified:        -

    % Set plot size
    plotWidth = 1000;
    plotHeight = 600;

    % Get screen resolution in pixels
    set(0,'units','pixels');            % Set screen units to pixels
    screenRes = get(0,'screensize');    % Get screen size in pixels
    screenWidth = screenRes(3);         % Width is 3rd entry
    screenHeight = screenRes(4);        % Height is 4th entry

    % Find xy coordinates of plot window to center on screen
    x = (screenWidth - plotWidth) / 2;
    y = (screenHeight - plotHeight) / 2;

    % Create the plots and save them to 'TrajectoryPlot.fig'
    figure('position',[x y plotWidth plotHeight]);
    ax1 = subplot(1,6,1);
    plot(ax1, time, distance); axis tight; ylim([0 1400]); title('Displacement vs Time'); ylabel('Displacement(m)'); xlabel('Time(s)');
    ax2 = subplot(1,6,2);
    plot(ax2, time, velocity); axis tight; ylim([0 95]); title('Velocity vs Time'); ylabel('Velocity(m/s)'); xlabel('Time(s)');
    ax3 = subplot(1,6,3);
    plot(ax3, distance, velocity); axis tight; ylim([0 95]); title('Velocity vs Displacement'); ylabel('Velocity(m/s)'); xlabel('Displacement(m)');
    ax4 = subplot(1,6,4);
    plot(ax4, time, power_input); axis tight; ylim([-150000 250000]); title('Power input vs Time'); ylabel('Power input(W)'); xlabel('Time(s)');
    ax5 = subplot(1,6,5);
    plot(ax5, rpm, power_input); axis tight; ylim([-150000 250000]); title('Power input vs RPM'); ylabel('Power input(W)'); xlabel('Time(s)');    
    ax6 = subplot(1,6,6);
    plot(ax6, time, torque); axis tight; ylim([-80 80]); title('Thrust torque vs Time'); ylabel('Thrust torque [Nm]'); xlabel('Time [s]');    
    savefig('TrajectoryPlot');
end

