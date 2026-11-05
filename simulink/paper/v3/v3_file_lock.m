function guard=v3_file_lock(path)
f=java.io.File(path);started=tic;while ~f.createNewFile(),assert(toc(started)<3600,'v3:lock','Timed out waiting for experiment ownership: %s',path);pause(.5);end
paper_json(path,struct('pid',feature('getpid'),'created',char(datetime('now','TimeZone','Asia/Shanghai'))));guard=onCleanup(@()release(path));
end
function release(path),if isfile(path),delete(path);end;end
