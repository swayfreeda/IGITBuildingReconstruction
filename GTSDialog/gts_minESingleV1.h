#ifndef GTS_MEXMINESINGLEV1_H
#define GTS_MEXMINESINGLEV1_H

#include "math.h"
#include <limits>
//  For Block Defined DP
void MyMin(double *Vs, double *Us, double *VCUtq, int len, double VC0, int n);


void MyMin(double *Vs, double *Us, double *VCUtq, int len, double VC0, int n, int w);


void CalCost(double *ItgY, double *DiffYX, double *Y, double *X, int h, int w, int n);


//  For Column Defined DP
void MyMin(double *OptV, int *SId, double *VtStp, int N);

void CalCtUtq(double* CtUtq, double *Y, double *X, int rows, int cols, int h);


void CalCtUtq(double *CtUtq, double *Y, double *X, int rows, int cols, int h,
              int c1, int c2);


void MinESingle(double *Q, double *Y, double *X, int H, int W, int N, int IfBlock);


void MinESingle(double *Q, double *Y, double *X, int H, int W, int N, int c1, int c2, int IfBlock);



#endif // MEXMINESINGLEV1_H
