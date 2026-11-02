function result=solve_hpipm_snapshot(s)
% Solve an explicitly supplied OCP-QP through the official HPIPM wrappers.
% This adapter is for fixed-data validation, not the online LTV assembler.
hpipm_startup;
N=double(s.N); nx=double(s.nx); nu=double(s.nu); nt=size(s.tr_J,1);
dim=hpipm_ocp_qp_dim(N); dim.set('nx',nx,0,N); dim.set('nu',nu,0,N-1);
dim.set('nbx',nx,0); dim.set('nbxe',nx,0); dim.set('nbu',nu,0,N-1);
for k=1:N, dim.set('nbx',nt,k); end
qp=hpipm_ocp_qp(dim); sol=hpipm_ocp_qp_sol(dim);
arg=hpipm_ocp_qp_solver_arg(dim,'balance');
arg.set('iter_max',40); arg.set('reg_prim',1e-10);
for f={'tol_stat','tol_eq','tol_ineq','tol_comp'}, arg.set(f{1},1e-6); end
solver=hpipm_ocp_qp_solver(dim,arg);
% Explicit destruction order protects native pointer lifetimes.
guard=onCleanup(@()cleanup_objects(solver,sol,qp,arg,dim));
for k=0:N-1
 qp.set('A',s.A(:,:,k+1),k); qp.set('B',s.B(:,:,k+1),k); qp.set('b',s.b(:,k+1),k);
 qp.set('R',s.R(:,:,k+1),k); qp.set('r',s.r(:,k+1),k);
 qp.set('Ju',eye(nu),k); qp.set('lu',s.lu(:,k+1),k); qp.set('uu',s.uu(:,k+1),k);
end
for k=0:N
 qp.set('Q',s.Q(:,:,k+1),k); qp.set('q',s.q(:,k+1),k);
 if k==0
  qp.set('Jx',eye(nx),k); qp.set('lx',zeros(nx,1),k); qp.set('ux',zeros(nx,1),k);
 elseif nt>0
  qp.set('Jx',s.tr_J,k); qp.set('lx',-s.tr_bound(:),k); qp.set('ux',s.tr_bound(:),k);
 end
end
% Round-trip the actual native structures to verify dimension/order conventions.
matrix_error=0;
for k=0:N
 matrix_error=max(matrix_error,max(abs(hpipm_qp_read(qp.C_qp,'Q',k)-s.Q(:,:,k+1)),[],'all'));
 matrix_error=max(matrix_error,max(abs(hpipm_qp_read(qp.C_qp,'q',k)-s.q(:,k+1)),[],'all'));
 if k<N
  for f={'A','B','R'}
   key=f{1}; value=s.(key); matrix_error=max(matrix_error,max(abs(hpipm_qp_read(qp.C_qp,key,k)-value(:,:,k+1)),[],'all'));
  end
  for f={'b','r','lu','uu'}
   key=f{1}; value=s.(key); matrix_error=max(matrix_error,max(abs(hpipm_qp_read(qp.C_qp,key,k)-value(:,k+1)),[],'all'));
  end
 end
end
solver.solve(qp,sol);
result.status=solver.get('status'); result.time_s=solver.get('time_ext');
result.residuals=cellfun(@(f)solver.get(f),{'max_res_stat','max_res_eq','max_res_ineq','max_res_comp'});
result.X=zeros(nx,N+1); result.U=zeros(nu,N);
for k=0:N, result.X(:,k+1)=sol.get('x',k); end
for k=0:N-1, result.U(:,k+1)=sol.get('u',k); end
result.matrix_error=matrix_error; result.objective=0;
for k=1:N+1, x=result.X(:,k); result.objective=result.objective+.5*x'*s.Q(:,:,k)*x+s.q(:,k)'*x; end
for k=1:N, u=result.U(:,k); result.objective=result.objective+.5*u'*s.R(:,:,k)*u+s.r(:,k)'*u; end
end
function cleanup_objects(solver,sol,qp,arg,dim)
delete(solver); delete(sol); delete(qp); delete(arg); delete(dim);
end
