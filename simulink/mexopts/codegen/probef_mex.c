/* probef_mex.c -- gateway proving the chain
 *   CasADi generate() -> MinGW GCC (w64devkit) -> MATLAB mex
 *
 * Notes that cost me a build:
 *  - the C entry points are named after the *casadi.Function* name (`probef`),
 *    not after the generated file name (`probef_plain.c`);
 *  - the generated C is self-contained (only <math.h>), but the gateway must
 *    use the same typedefs: on win64 `casadi_int` is `long long int`;
 *  - CasADi's C API hands the output buffer in as a second input as well, so the
 *    sizes must come from `<name>_work()` instead of being assumed.
 *
 * Build:  mex probef_mex.c probef_plain.c
 * Call:   y = probef_mex([3; 0])   ->  9
 */
#include "mex.h"
#include <stdlib.h>

#define casadi_real double
#define casadi_int long long int

extern int probef_work(casadi_int *sz_arg, casadi_int* sz_res,
                       casadi_int *sz_iw, casadi_int *sz_w);
extern int probef(const casadi_real** arg, casadi_real** res,
                  casadi_int* iw, casadi_real* w, int mem);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    casadi_int sz_arg = 0, sz_res = 0, sz_iw = 0, sz_w = 0;
    casadi_real x[2], y[1];
    const casadi_real** arg;
    casadi_real** res;
    casadi_int* iw;
    casadi_real* w;
    const double* px;
    int k;

    if (nrhs < 1 || mxGetNumberOfElements(prhs[0]) < 2)
        mexErrMsgTxt("probef_mex expects a 2-element vector");

    px = mxGetPr(prhs[0]);
    x[0] = px[0];
    x[1] = px[1];
    y[0] = 0.0;

    probef_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
    arg = (const casadi_real**) calloc((size_t) sz_arg, sizeof(casadi_real*));
    res = (casadi_real**) calloc((size_t) sz_res, sizeof(casadi_real*));
    iw = (casadi_int*) calloc((size_t) (sz_iw > 0 ? sz_iw : 1), sizeof(casadi_int));
    w = (casadi_real*) calloc((size_t) (sz_w > 0 ? sz_w : 1), sizeof(casadi_real));
    if (!arg || !res || !iw || !w) mexErrMsgTxt("out of memory");

    for (k = 0; k < (int) sz_arg; ++k) arg[k] = x;   /* input, and output alias */
    arg[sz_arg - 1] = y;
    for (k = 0; k < (int) sz_res; ++k) res[k] = y;

    if (probef(arg, res, iw, w, 0) != 0)
        mexErrMsgTxt("probef() returned an error");

    free(arg);
    free(res);
    free(iw);
    free(w);

    plhs[0] = mxCreateDoubleScalar(y[0]);
}
