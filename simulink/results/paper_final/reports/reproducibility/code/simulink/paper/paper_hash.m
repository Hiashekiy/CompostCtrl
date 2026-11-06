function h=paper_hash(path)
fid=fopen(path,'rb');assert(fid>0);c=onCleanup(@()fclose(fid));bytes=fread(fid,inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
