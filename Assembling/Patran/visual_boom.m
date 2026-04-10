% 1. Create a basic cylinder (radius 0.1, 50 surface facets)
% We swap the output order to make X the length axis, matching your Nastran model!
[X, Y, Z] = cylinder(0.1, 50); 

% 2. Scale the length along the X-axis to 4.7 meters
Z = Z * 4.7; 

% 3. Convert the surface data into triangular faces and vertices
TR = surf2patch(X, Y, Z, 'triangles');
TR_obj = triangulation(TR.faces, TR.vertices);

% 4. Export it to your current folder as an STL file
stlwrite(TR_obj, 'boom_visual.stl');

disp('Success! boom_visual.stl has been created in your current folder.');