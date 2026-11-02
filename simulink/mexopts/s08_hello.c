#include "mex.h"
#include <stdio.h>
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const char *cc = "unknown";
#ifdef __GNUC__
    cc = "gcc " __VERSION__;
#endif
    mexPrintf("hello from %s (sizeof(double)=%d)\n", cc, (int)sizeof(double));
    if (nlhs >= 1) {
        plhs[0] = mxCreateDoubleScalar(2.0 * (nrhs >= 1 ? mxGetScalar(prhs[0]) : 21.0));
    }
}
