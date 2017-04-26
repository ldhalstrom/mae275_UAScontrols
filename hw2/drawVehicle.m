
function drawVehicle(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position
    pe       = uu(2);       % inertial East position
    pd       = uu(3);
    u        = uu(4);
    v        = uu(5);
    w        = uu(6);
    phi      = uu(7);       % roll angle
    theta    = uu(8);       % pitch angle
    psi      = uu(9);       % yaw angle
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate
    r        = uu(12);       % yaw rate
    t        = uu(13);       % time

    % define persistent variables
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors

    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-10,10,-10,10,-10,10]);
        hold on

    % at every other time step, redraw base and rod
    else
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end


%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;

  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;

end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
La = 2;         % length of rotor support arm
Rr = 0.5;          % rotor radius



V = [...
     0,  0, 0;...  % pt 1 (center)
    %  La,  0, 0;...  % pt 2 (front)
    % -La,  0, 0;...  % pt 3 (back)
    %  0,  La, 0;...  % pt 4 (right)
    %  0, -La, 0;...  % pt 5 (left)
    %  La+Rr, 0, 0;...  %
    %  La-Rr, 0, 0;...  %
    %  La, La+Rr, 0;...  %
    %  La, La-Rr, 0;...  %
    ]'; % ' is transpose

%Rotor center locations
RotCent = [...
    La,  0, 0;...  % pt 2 (front)
    -La,  0, 0;...  % pt 3 (back)
     0,  La, 0;...  % pt 4 (right)
     0, -La, 0;...  % pt 5 (left)
     ]';

%Append to vertices
V = horzcat(V,RotCent);




% %Make vertices for 4 rotors
% nRotVerts = 10 %Number of verticies in rotor circle
% nRotVerts = 4 %Number of verticies in rotor circle
% for i=1:4
%     % for j=1:nRotVerts


%         xc = RotCent(1,i);  %x-center of rotor
%         yc = RotCent(2,i);  %y-center of rotor

%         RotVerts = [...
%             xc+Rr, yc, 0;...  %front
%             xc,    yc+Rr, 0;...  %right
%             xc-Rr, yc, 0;...  %back
%             xc,    yc-Rr, 0;...  %left
%             ]';
%     % end

%     %append to vertices
%     V = horzcat(V,RotVerts)
% end

% %Make vertices for 4 rotors
nRotVerts = 10; %Number of verticies in rotor circle
dang = 360 / nRotVerts; %arc angle between each vertex in circle
for i=1:4

    xc = RotCent(1,i);  %x-center of rotor
    yc = RotCent(2,i);  %y-center of rotor

    RotVerts = zeros(3, nRotVerts); %Store rotor vertices for current rotor

    %calculate each vertex in circle incrementally
    for j=1:nRotVerts
        RotVerts(1,j) = xc + Rr * cosd(dang * (j -1)); %x disp. from center
        RotVerts(2,j) = yc + Rr * sind(dang * (j -1)); %y disp. from center
    end

    %append to vertices
    V = horzcat(V,RotVerts);
end

% define faces as a list of vertices numbered above
F = ones(8, nRotVerts)

%Make 4 faces for 4 arms (need to be same number of vers as rotors)
for i=1:4
    F(i,2:nRotVerts) = F(i,2:nRotVerts) * i+1
end

%Make 4 faces for 4 rotors (circles made of nRotVert vertices)
nvert = 6; %start at this vertex and increase by one
for i=1:4
    for j=1:nRotVerts
        F(i+4,j) = nvert;
        nvert = nvert +1;
    end
end

disp(F)

% define colors for each face
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];
  myblack = [0, 0, 0];

  facecolors = [...
    myblack;...    % front arm
    myblack;...      % rear arm
    myblack;...     % right arm
    myblack;...     % left arm
    mygreen;...     % Front rotor
    myred;...     % Rear rotor
    myblue;...     % Right rotor
    mycyan;...     % Left rotor
    ];
end

