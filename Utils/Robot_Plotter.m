classdef Robot_Plotter
    %Robot_Plotter Plot the robot
    %   
    properties
        Robot
        q
    end
    
    methods
        % Class constructor
        function obj = Robot_Plotter(Robot, q0)
            if nargin > 1
                obj.Robot = Robot;
                obj.q = q0;
            else
                error('Not enough input arg')
            end
        end
        
        function Init(obj,a,b,c)
            
            figure()
            plot3(a, b, c)
            hold on
            obj.update_plot
        end
        
        function update(obj,angles)
            obj.q = angles;
            obj.update_plot
        end
        
        function update_plot(obj)
            previous = [0 0 0];
            [EE, T_all] = obj.Robot.fkine(obj.q);
            for I=1:length(obj.Robot.theta)
                plot3(previous(1),previous(2),previous(3),'ro')
                link =[previous; T_all(1:3,4,I)'] 
                line(link(:,1)',link(:,2)',link(:,3)')
                previous = T_all(1:3,4,I)';
            end
        end
    end
    
end


