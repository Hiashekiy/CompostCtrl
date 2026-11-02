/* Read matrices from official HPIPM structures for cross-interface QA.
 * Input handle is exclusively supplied by hpipm_ocp_qp.C_qp. */
#include "mex.h"
#include "hpipm_d_ocp_qp.h"
#include <stdint.h>
#include <string.h>
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {
 if(nrhs!=3 || nlhs!=1 || !mxIsInt64(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
  mexErrMsgIdAndTxt("hpipm:readArgs","Expected official int64 QP handle, field, stage");
 struct d_ocp_qp *qp=(struct d_ocp_qp *)(uintptr_t)(*(int64_t *)mxGetData(prhs[0]));
 char field[32]; if(mxGetString(prhs[1],field,sizeof(field))) mexErrMsgIdAndTxt("hpipm:field","Invalid field");
 int k=(int)mxGetScalar(prhs[2]),N=qp->dim->N;
 if(k<0 || k>N) mexErrMsgIdAndTxt("hpipm:stage","Invalid stage");
 int rows=0,cols=1,nx=qp->dim->nx[k],nu=qp->dim->nu[k];
 if(!strcmp(field,"Q")){rows=nx;cols=nx;}
 else if(!strcmp(field,"q")) rows=nx;
 else if(!strcmp(field,"lx") || !strcmp(field,"ux")) rows=qp->dim->nbx[k];
 else if(k<N){
  if(!strcmp(field,"A")){rows=qp->dim->nx[k+1];cols=nx;}
  else if(!strcmp(field,"B")){rows=qp->dim->nx[k+1];cols=nu;}
  else if(!strcmp(field,"b")) rows=qp->dim->nx[k+1];
  else if(!strcmp(field,"R")){rows=nu;cols=nu;}
  else if(!strcmp(field,"r")) rows=nu;
  else if(!strcmp(field,"lu") || !strcmp(field,"uu")) rows=qp->dim->nbu[k];
 }
 if(!rows) mexErrMsgIdAndTxt("hpipm:field","Unsupported field or empty stage");
 plhs[0]=mxCreateDoubleMatrix(rows,cols,mxREAL);
 double *v=mxGetDoubles(plhs[0]);
 if(!strcmp(field,"Q")) d_ocp_qp_get_Q(k,qp,v);
 else if(!strcmp(field,"q")) d_ocp_qp_get_q(k,qp,v);
 else if(!strcmp(field,"A")) d_ocp_qp_get_A(k,qp,v);
 else if(!strcmp(field,"B")) d_ocp_qp_get_B(k,qp,v);
 else if(!strcmp(field,"b")) d_ocp_qp_get_b(k,qp,v);
 else if(!strcmp(field,"R")) d_ocp_qp_get_R(k,qp,v);
 else if(!strcmp(field,"r")) d_ocp_qp_get_r(k,qp,v);
 else if(!strcmp(field,"lu")) d_ocp_qp_get_lbu(k,qp,v);
 else if(!strcmp(field,"uu")) d_ocp_qp_get_ubu(k,qp,v);
 else if(!strcmp(field,"lx")) d_ocp_qp_get_lbx(k,qp,v);
 else if(!strcmp(field,"ux")) d_ocp_qp_get_ubx(k,qp,v);
}
