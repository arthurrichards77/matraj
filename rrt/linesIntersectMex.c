/*=================================================================
 *
 * YPRIME.C	Sample .MEX file corresponding to YPRIME.M
 *	        Solves simple 3 body orbit problem
 *
 * The calling syntax is:
 *
 *		[yp] = yprime(t, y)
 *
 *  You may also want to look at the corresponding M-code, yprime.m.
 *
 * This is a MEX-file for MATLAB.
 * Copyright 1984-2004 The MathWorks, Inc.
 *
 *=================================================================*/
/* $Revision: 1.10.6.2 $ */
#include "mex.h"

/* Input Arguments */

#define	T_IN	prhs[0]
#define	Y_IN	prhs[1]


/* Output Arguments */

#define	YP_OUT	plhs[0]

static void linesIntersect(
        double a1[],
        double a2[],
        double b1[],
        double b2[],
        double *flag
        )

{
    
    double dx, dy, da, db, lam, gam;
    
    *flag=0;
    
    dx = a2[0] - a1[0];
    dy = a2[1] - a1[1];
    da = b2[0] - b1[0];
    db = b2[1] - b1[1];
    
    if ((da * dy - db * dx) != 0) {
        lam = (dx * (b1[1] - a1[1]) + dy * (a1[0] - b1[0])) / (da * dy - db * dx);
        gam = (da * (a1[1] - b1[1]) + db * (b1[0] - a1[0])) / (db * dx - da * dy);
        if ((lam>=0)&&(lam<=1)&&(gam>=0)&&(gam<=1)) *flag = 1;
    }
    return;
}

void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    double *a1, *a2, *b1, *b2;
    double *flag;
    
    /* Check for proper number of arguments */
    
    if (nrhs != 4) {
        mexErrMsgTxt("Four input arguments required.");
    } else if (nlhs > 1) {
        mexErrMsgTxt("Too many output arguments.");
    }
    
    /* All RHS should be 2x1 */
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
            (mxGetM(prhs[0]) != 2) || (mxGetN(prhs[0]) != 1)) {
        mexErrMsgTxt("Every input should be a 2 x 1 vector.");
    }
    if (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) ||
            (mxGetM(prhs[1]) != 2) || (mxGetN(prhs[1]) != 1)) {
        mexErrMsgTxt("Every input should be a 2 x 1 vector.");
    }
    if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) ||
            (mxGetM(prhs[2]) != 2) || (mxGetN(prhs[2]) != 1)) {
        mexErrMsgTxt("Every input should be a 2 x 1 vector.");
    }
    if (!mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) ||
            (mxGetM(prhs[3]) != 2) || (mxGetN(prhs[3]) != 1)) {
        mexErrMsgTxt("Every input should be a 2 x 1 vector.");
    }
    
    /* Create a matrix for the return argument */
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    
    /* Assign pointers to the various parameters */
    flag = mxGetPr(plhs[0]);
    
    a1 = mxGetPr(prhs[0]);
    a2 = mxGetPr(prhs[1]);
    b1 = mxGetPr(prhs[2]);
    b2 = mxGetPr(prhs[3]);
    
    /* Do the actual computations in a subroutine */
    linesIntersect(a1,a2,b1,b2,flag);
    return;
    
}


