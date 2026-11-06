function [vector,fields]=hpipm_pack_qp(s)
% Column-major wire contract; dimensions and trust-region structure in template.
fields={'A','B','b','Q','q','R','r','lu','uu'}; vector=[];
for k=1:numel(fields),v=s.(fields{k});vector=[vector;v(:)];end %#ok<AGROW>
end
