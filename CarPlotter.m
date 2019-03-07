% Copyright (c) 2019 Bruno Tiberio
%                    
%
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the
% "Software"), to deal in the Software without restriction, including
% without limitation the rights to use, copy, modify, merge, publish,
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the
% following conditions:
%
% The above copyright notice and this permission notice shall be included
% in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
% NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
% DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
% USE OR OTHER DEALINGS IN THE SOFTWARE.
%
%
%
%
%==========================================================================
%
% CAR PLOTER
% 
% Assumed axis convection in car body frame is x pointing towards front
% y towards left door and z up (although currently not used)
%
%           x ^
%             :
%      \======:======\
%             :        
%             :
%             :
%             :
%             :
%           o :
%  <----|-----:=======|
%  y                
%
%==========================================================================
classdef CarPlotter < handle
    properties
        %------------------------------------------------------------------
        % Constants section
        %------------------------------------------------------------------
        
        % distance between axis
        wheel_base;
        % distance between wheels in same axis
        front_track;
        % car body frame origin in world frame
        car_position = [nan nan];
        % car orientation angle in world frame
        car_pose = nan;
    end
    properties(SetAccess = private)
        %------------------------------------------------------------------
        % figure handles for rapid plotting and updating
        %------------------------------------------------------------------
        
        % main figure handle
        figure_handle;
        % axes handle for zoom
        zoomed_axes_handle;
        % axes handle for full view
        full_map_axes_handle;
        % handle for ploting car trajectory        
        car_path_handle;
        
        %------------------------------------------------------------------
        % zoomed view handles for plots
        %------------------------------------------------------------------
        
        % handle for plotting car path in zoomed axes
        car_path_zoomed_handle;
        % handle for plotting car position
        car_position_handle;
        % handle for patch representing car
        car_patch_handle;
        % handle for plotting gps1 points
        gps1_handle;
        % handle for plotting gps2 points
        gps2_handle;
        % number of points for circular buffers
        buffSize;
        % circular buffer for car position
        circBuff_car = [];
        % circular buffer for gps1
        circBuff_gps1 = [];
        % circular buffer for gps2 
        circBuff_gps2 = [];
        % enable gps plots
        use_gps = 0;
        % get default color order as matrix
        co = get(groot,'defaultAxesColorOrder');
    end
    
     methods
        %% Public methods
        function obj = CarPlotter(varargin)
            % CARPLOTTER Create a figure to represent the car movement with
            % a zoomed view and a full map also.
            %
            %   CARPLOTTER() use default values
            %
            %   CARPLOTTER( NAME, VALUE) accepts one or more of the 
            %   following name-value pair arguments.
            %           * 'useGPS' with logical value
            %           * 'frontTrack' distance between wheels in front
            %           axis
            %           * 'wheelBase' distance between both wheel axes
            %           * 'bufferSize' number of points in circular buffers
            args = inputParser();
            
            args.addParameter('useGPS', false, @(x)validateattributes(x,{'logical'},...
                {'scalar'}));
            args.addParameter('frontTrack',1.3,...
                @(x)validateattributes(x,{'numeric'},...
                {'positive','real','scalar'}));
            args.addParameter('wheelBase',2.2,...
                @(x)validateattributes(x,{'numeric'},...
                {'positive','real','scalar'}));
            args.addParameter('bufferSize', 300,...
                @(x)validateattributes(x,{'numeric'},...
                {'positive','real','scalar'}));
            args.parse(varargin{:});
            
            obj.buffSize = args.Results.bufferSize;
            obj.wheel_base = args.Results.wheelBase;
            obj.front_track = args.Results.frontTrack;
            obj.use_gps = args.Results.useGPS;
            
            obj.circBuff_car = nan(obj.buffSize,2);
            if obj.use_gps
                obj.circBuff_gps1 = nan(obj.buffSize,2);
                obj.circBuff_gps2 = nan(obj.buffSize,2);
            end
            % just in case of using old matlab versions
            % update to the new color order which is more visual appelative
            if any(~( obj.co(1,:) == [ 0    0.4470    0.7410]))
                obj.co = [0    0.4470    0.7410;
                    0.8500    0.3250    0.0980;
                    0.9290    0.6940    0.1250;
                    0.4940    0.1840    0.5560;
                    0.4660    0.6740    0.1880;
                    0.3010    0.7450    0.9330;
                    0.6350    0.0780    0.1840];
            end
        end
        
        function prepare_figure(obj, map_points, car_position, pose)
            % PREPARE_FIGURE prepare a figure to receive further data.
            %
            %   PREPARE_FIGURE(map_points, car_position, pose) create plots
            %   with reference trajectory, starting position and pose of
            %   car in world frame.
            
            %--------------------------------------------------------------
            % check if figure is still active or not
            %--------------------------------------------------------------
            if isempty(obj.figure_handle) || ~isvalid(obj.figure_handle)
                % create a new one
                obj.figure_handle = figure('name','Car Plotter');
                sizes = num2cell(obj.figure_handle.Position);
                [x, y, w, h]=sizes{:};
                obj.figure_handle.Position = [round(x/2) y w*2 h];
            else
                % bring to front and clear it
                figure(obj.figure_handle);
                clf(obj.figure_handle);
            end
            % set first position of car and pose;
            obj.car_position = car_position;
            obj.car_pose = pose;
            % for sake of simplicity
            L = obj.wheel_base;
            W = obj.front_track;
            % calculate rotation matrix
            rotMatrix = [cos(pose) -sin(pose); sin(pose) cos(pose)];
            % calculate vertices of the car as if it was a triangle
            PF = ([car_position(1) car_position(2)]'+rotMatrix*[L, 0]')';
            PL = ([car_position(1) car_position(2)]'+rotMatrix*[0, W/2]')';
            PR = ([car_position(1) car_position(2)]'+rotMatrix*[0, -W/2]')';
            car_vertices = [PL;PR;PF];
            
            % zoomed
            obj.zoomed_axes_handle = subplot(1,2,1);
            obj.car_patch_handle = patch('Faces',[1 2 3],'Vertices',car_vertices,'FaceColor','cyan');
            xlabel('East [m]','FontSize',9, 'FontWeight','normal');
            ylabel('North [m]','FontSize',9, 'FontWeight','normal');
            title('Estimated position in 2D - zoom','FontSize',9, 'FontWeight','normal');
            % axis equal;
            daspect([1 1 1]);
            % zoomed must also contain all reference path data.
            hold on;
            plot(map_points(:,1), map_points(:,2), 'Color', obj.co(2,:));
            obj.car_path_zoomed_handle = plot(nan, nan, '--','Color', obj.co(1,:));
            box on;
            axis(obj.zoomed_axes_handle, [car_position(1)-15 car_position(1)+15 car_position(2)-15 car_position(2)+15]);
            
          
            % full map
            obj.full_map_axes_handle = subplot(1,2,2);
            plot(map_points(:,1), map_points(:,2), 'Color', obj.co(2,:));
            hold on;
            plot(map_points(1,1), map_points(1,2), 'Marker','s', 'MarkerFaceColor', obj.co(3,:));
            plot(map_points(end,1), map_points(end,2), 'Marker','s', 'MarkerFaceColor', obj.co(5,:) );
           
            obj.car_path_handle = plot(obj.car_position(1), obj.car_position(2),...
                 'Color', obj.co(1,:), 'LineStyle', '--');
            %obj.legend_handle=legend('Ref.', 'Start', 'Stop','Car', 'location','north','Orientation','horizontal');
            obj.car_position_handle = plot(obj.car_position(1), obj.car_position(2),...
                'o', 'Color', obj.co(1,:), 'MarkerFaceColor',obj.co(1,:)); 
            % axis equal;
            daspect([1 1 1]);
            xlabel('East [m]','FontSize',9, 'FontWeight','normal');
            ylabel('North [m]','FontSize',9, 'FontWeight','normal');
            title({'Estimated position in 2D',['\color[rgb]{0.8500    0.3250    0.0980}Ref.',...
                '\color[rgb]{0.9290    0.6940    0.1250} Start',...
                '\color[rgb]{0.4660    0.6740    0.1880} Stop',...
                '\color[rgb]{0    0.4470    0.7410} Car']},'FontSize',9, 'FontWeight','normal');
            % keep legend simple. Remove redundant references.
            %obj.legend_handle.String = obj.legend_handle.String(1:4);
            ax2_limits = [min(map_points(:,1))-50, max(map_points(:,1))+50, min(map_points(:,2))-50, max(map_points(:,2))+50];
            axis(obj.full_map_axes_handle,ax2_limits);
            drawnow;
            
        end
        function draw_car(obj)
            % DRAW_CAR update patch vertices to represent car.
            L = obj.wheel_base;
            W = obj.front_track;
            x = obj.car_position(1);
            y = obj.car_position(2);
            psi = obj.car_pose;
            rotMatrix = [cos(psi) -sin(psi); sin(psi) cos(psi)];
            PF = ([x y]'+rotMatrix*[L, 0]')';
            PL = ([x y]'+rotMatrix*[0, W/2]')';
            PR = ([x y]'+rotMatrix*[0, -W/2]')';
            car = [PL;PR;PF];
            obj.car_patch_handle.Vertices = car;
            axis(obj.zoomed_axes_handle, [x-15 x+15 y-15 y+15]);
            
        end
        function update_data(obj, car, pose, gps)
            % UPDATE_DATA update data used in graphics
            %   UPDATE_DATA(car, pose) update plot data with car position 
            %   [x y] and orientation angle, both in world frame.
            %
            %   UPDATE_DATA(car, pose, gps) update plot data with car
            %   position, orientation angle and gps data, all in world
            %   frame. The gps is given by a 4x1 matrix where the
            %   containing x, y for each gps in world frame.
            
            % update car position
            obj.car_position = car;
            obj.car_pose = pose;
            % append to full map car path data
            obj.car_path_handle.XData = [obj.car_path_handle.XData car(1)];
            obj.car_path_handle.YData = [obj.car_path_handle.YData car(2)];
            % update car path circular buffer by adding car position at end
            obj.circBuff_car = [obj.circBuff_car(2:end,:); car];
            if obj.use_gps
                obj.circBuff_gps1 = [obj.circBuff_gps1(2:end,:); gps(1:2)];
                obj.circBuff_gps2 = [obj.circBuff_gps2(2:end,:); gps(3:4)];
            end
            
        end
        
        function update_plots(obj)
            % UPDATE_PLOTS refresh figures with stored data
            if isempty(obj.car_position_handle)
                error('figures are empty');
            end
            % update car position
            obj.car_position_handle.XData = obj.car_position(1);
            obj.car_position_handle.YData = obj.car_position(2);
            if obj.use_gps
                % update gps1 plot
                obj.gps1_handle.XData = obj.circBuff_gps1(:,1);
                obj.gps1_handle.YData = obj.circBuff_gps1(:,2);
                % update gps2 plot
                obj.gps2_handle.XData = obj.circBuff_gps2(:,1);
                obj.gps2_handle.YData = obj.circBuff_gps2(:,2);
            end
            % update car patch vertices
            obj.draw_car();
            obj.car_path_zoomed_handle.XData = obj.circBuff_car(:,1);
            obj.car_path_zoomed_handle.YData = obj.circBuff_car(:,2);
            % update figures
            drawnow('limitrate');
        end
     end
end