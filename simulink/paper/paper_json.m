function paper_json(path,value)
f=fopen(path,'w','n','UTF-8');assert(f>0);c=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(value,'PrettyPrint',true));
end
