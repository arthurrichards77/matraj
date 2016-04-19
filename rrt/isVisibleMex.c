/*=================================================================
 *
 * The calling syntax is:
 *
 *		flag = isVisible(x1,x2,walls)
 *
 *=================================================================*/
#include "mex.h"

static void linesIntersect(
        double a1[],
        double a2[],
        double b1[],
        double b2[],
        double *flag
        )

{
    
    double dx, dy, da, db, lam, gam;
    
    /* mexPrintf("Checking (%lf,%lf)-(%lf,%lf) vs (%lf,%lf)-(%lf,%lf)\n",a1[0],a1[1],a2[0],a2[1],b1[0],b1[1],b2[0],b2[1]); */
    
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

static void isVisible(
        double x1[],
        double x2[],
        double walls[][4],
        int nWalls,
        double *flag
        )

{
 
    int ww,jj;
    double intFlag;
    
    /* mexPrintf("Wall 1: (%lf,%lf)-(%lf,%lf)\n",walls[0][0],walls[0][1],walls[0][2],walls[0][3]); */
    
    *flag=1;
    for(ww=0;ww<nWalls;ww++) {
        linesIntersect(x1,x2,walls[ww],&walls[ww][2],&intFlag);
        if (intFlag==1) {
            *flag=0;
            break;
        }
    }
        
    return;
    
}

void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    double *x1, *x2, *walls;
    double *flag;
    int nWalls;
    
    /* Check for proper number of arguments */
    
    if (nrhs != 3) {
        mexErrMsgTxt("Three input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgTxt("One output argument required.");
    }
    
    /* All RHS should be 2x1 */
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) ||
            (mxGetM(prhs[0]) != 2) || (mxGetN(prhs[0]) != 1)) {
        mexErrMsgTxt("First input should be a 2 x 1 vector.");
    }
    if (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) ||
            (mxGetM(prhs[1]) != 2) || (mxGetN(prhs[1]) != 1)) {
        mexErrMsgTxt("Second input should be a 2 x 1 vector.");
    }
    if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) ||
            (mxGetM(prhs[2]) != 4)) {
        mexErrMsgTxt("Third input should be a 4 x n Matrix.");
    }
    
    /* extract number of walls - number of rows in third input */
    nWalls=mxGetN(prhs[2]);
    
    /* Create a matrix for the return argument */
    plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
    
    /* Assign pointers to the various parameters */
    flag = mxGetPr(plhs[0]);
    
    x1 = mxGetPr(prhs[0]);
    x2 = mxGetPr(prhs[1]);
    walls = mxGetPr(prhs[2]);
    
    /* Do the actual computations in a subroutine */
    isVisible(x1,x2,walls,nWalls,flag);
    return;
    
}


