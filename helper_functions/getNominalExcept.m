function P = getNominalExcept(G,ulist)
    P = G;
    if ~isuncertain(G)
        return
    end
    uncs = struct2cell(G.Uncertainty);
    
    REP = [];
    for i = 1:length(uncs)
        u = uncs{i};
        if startsWith(u.Name,ulist)
            continue;
        end
        REP.(u.Name) = u.Nominal;
%         P = usubs(P,u.Name,u.Nominal);
    end
    
    if ~isempty(REP)
        P = usubs(P,REP);
    end
end