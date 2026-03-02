function structure = get_reduced_FEM_structure(options)
    structure = [];
    
    feModel = createpde('structural','modal-solid');
    importGeometry(feModel,options.FileName);

    structuralProperties(feModel, ...
        'YoungsModulus',options.E, ...
        'PoissonsRatio',options.nu, ...
        'MassDensity',options.rho);
    mesh = generateMesh(feModel, ...
        'GeometricOrder','quadratic', ...
        'Hmax',options.Hmax, ...
        'Hmin',options.Hmin);

    if isempty(options.faceIDs)
        figure
        hc = pdegplot(feModel,'FaceLabels','on');
        hc(1).FaceAlpha = 0.5;
        title('Plate with Face Labels')
        %return
    end

    numFrames = length(options.faceIDs);
    origins = [];
    for i = 1:numFrames
       structuralBC(feModel, ...
                     'Face',options.faceIDs(i), ...
                     'Constraint','multipoint');
    end
    
    rom = reduce(feModel,'FrequencyRange',[0 options.maxModalFreq]);

    structure.P = rom.ReferenceLocations';  % Interface frame locations (n x 3 matrix)
    structure.M = rom.M;                    % Reduced mass matrix
    structure.K = rom.K;                    % Reduced stiffness matrix
    structure.C = 2e-3* structure.M + 5e-3 * structure.K;  
    
%     return
    
    figure
    pdemesh(feModel,'FaceAlpha',0.5)
    hold on
    colors = parula(numFrames);
    assert(numel(options.faceIDs) == numFrames);
    for k = 1:numFrames
        nodeIdxs = findNodes(feModel.Mesh,'region','Face',options.faceIDs(k));
        scatter3( ...
            feModel.Mesh.Nodes(1,nodeIdxs), ...
            feModel.Mesh.Nodes(2,nodeIdxs), ...
            feModel.Mesh.Nodes(3,nodeIdxs), ...
            10,'ok','MarkerFaceColor',colors(k, :))
        scatter3( ...
            structure.P(k,1), ...
            structure.P(k,2), ...
            structure.P(k,3), ...
            80,'k','filled','s')
    end
    hold off
    pause(0.1);
    
%     return
    
    result = solve(feModel,'FrequencyRange',[-0.1  options.maxModalFreq]);
    freqHz = result.NaturalFrequencies;
    numToPrint = length(freqHz);
    for i = 1:numToPrint
        figure
        pdeplot3D(feModel,'ColorMapData',result.ModeShapes.uz(:,i));
        axis equal
        pause(0.1)
    end
%     
%     frmPerm = zeros(numFrames,1);    % Frame permutation vector
%     dofPerm = 1:size(structure.K,1);       % DOF permutation vector
% 
%     assert(size(structure.P,1) == numFrames);
%     for i = 1:numFrames
%         for j = 1:numFrames
%             if isequal(structure.P(j,:),structure.P(i,:))
%                 frmPerm(i) = j;
%                 dofPerm(6*(i-1)+(1:6)) = 6*(j-1)+(1:6);
%                 continue;
%             end
%         end
%     end
% 
%     structure.P = structure.P(frmPerm,:);
%     structure.K = structure.K(dofPerm,:);
%     structure.K = structure.K(:,dofPerm);
%     structure.M = structure.M(dofPerm,:);
%     structure.M = structure.M(:,dofPerm);
%     structure.C = structure.C(dofPerm,:);
%     structure.C = structure.C(:,dofPerm);
end