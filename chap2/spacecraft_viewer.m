classdef spacecraft_viewer < handle
    %
    % Aircraft 3D viewer for Chapter 2
    %
    %--------------------------------
    properties
        body_handle
        Vertices
        Faces
        facecolors
        plot_initialized
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = spacecraft_viewer()
            self.body_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_aircraft();
            self.plot_initialized = 0;           
        end
        
        %---------------------------
        function self = update(self, state)
            if self.plot_initialized == 0
                figure(1); clf;
                self.drawBody(state.pn, state.pe, -state.h, ...
                              state.phi, state.theta, state.psi);
                title('Aircraft Viewer')
                xlabel('East')
                ylabel('North')
                zlabel('-Down')
                view(32,47)
                axis([-50, 50, -50, 50, -20, 20]);
                grid on
                hold on
                self.plot_initialized = 1;
            else
                self.drawBody(state.pn, state.pe, -state.h, ...
                              state.phi, state.theta, state.psi);
            end
        end
        
        %---------------------------
        function self = drawBody(self, pn, pe, pd, phi, theta, psi)
            % rotate rigid body
            Vertices = self.rotate(self.Vertices, phi, theta, psi);
            % translate after rotation
            Vertices = self.translate(Vertices, pn, pe, pd);
            % convert from NED to MATLAB XYZ frame
            Rned2xyz = [0 1 0; 1 0 0; 0 0 -1];
            Vertices = Rned2xyz * Vertices;

            if isempty(self.body_handle)
                self.body_handle = patch( ...
                    'Vertices', Vertices', ...
                    'Faces', self.Faces, ...
                    'FaceVertexCData', self.facecolors, ...
                    'FaceColor','flat');
            else
                set(self.body_handle, ...
                    'Vertices', Vertices', ...
                    'Faces', self.Faces);
                drawnow
            end
        end
        
        %---------------------------
        function pts = rotate(self, pts, phi, theta, psi)
            R_roll = [ ...
                1 0 0;
                0 cos(phi) sin(phi);
                0 -sin(phi) cos(phi)];
            
            R_pitch = [ ...
                cos(theta) 0 -sin(theta);
                0 1 0;
                sin(theta) 0 cos(theta)];
            
            R_yaw = [ ...
                cos(psi) sin(psi) 0;
                -sin(psi) cos(psi) 0;
                0 0 1];
            
            R = (R_roll * R_pitch * R_yaw)';
            pts = R * pts;
        end
        
        %---------------------------
        function pts = translate(~, pts, pn, pe, pd)
            pts = pts + repmat([pn; pe; pd], 1, size(pts,2));
        end
        
        %---------------------------
        function [V, F, colors] = define_aircraft(self)
            scale = 1.5;
            fuse_l1 = 5; fuse_l2 = 3; fuse_l3 = 12;
            fuse_w = 1.5;
            wing_l = 5; wing_w = 14;
            tail_l = 2.5; tail_w = 7; tail_h = 2;

            % vertices
            V = [ ...
                fuse_l1, 0, 0;
                fuse_l2, -fuse_w, -fuse_w/2;
                fuse_l2, fuse_w, -fuse_w/2;
                fuse_l2, fuse_w, fuse_w/2;
                fuse_l2, -fuse_w, fuse_w/2;
                -fuse_l3, 0, 0;
                0, wing_w, 0;
                -wing_l, wing_w, 0;
                -wing_l, -wing_w, 0;
                0, -wing_w, 0;
                -fuse_l3, tail_w, 0;
                -(fuse_l3+tail_l), tail_w, 0;
                -(fuse_l3+tail_l), -tail_w, 0;
                -fuse_l3, -tail_w, 0;
                -(fuse_l3+tail_l), 0, 0;
                -(fuse_l3+tail_l), 0, -tail_h;
                -fuse_l3, 0, -tail_h;
                ]';
            
            V = scale * V;

            F = [ ...
                1 2 3 1;
                1 3 4 1;
                1 4 5 1;
                1 5 2 1;
                2 3 6 2;
                3 6 4 3;
                4 6 5 4;
                2 5 6 2;
                7 8 9 10;
                11 12 13 14;
                6 15 17 17;
                ];
            
            yellow = [1,1,0];
            blue   = [0,0,1];
            red    = [1,0,0];
            green  = [0,1,0];
            
            colors = [ ...
                yellow;
                yellow;
                yellow;
                yellow;
                blue;
                blue;
                red;
                blue;
                green;
                green;
                blue;
                ];
        end
        
    end
end
