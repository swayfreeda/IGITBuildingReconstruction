#include"gts_minESingleV1.h"
#include<iostream>
#include<cassert>
using namespace std;
//=====================Input==================================
//  in[0~3]: Y, X, n, IfBlock.
//  in[0~5]: Y, X, n, c1,c2, IfBlock.
//  Y: h-by-w matrix, the initial label.
//  X: h-by-n matrix, the repeated element of generalized translation symmetry.
//
//=====================Output=================================
//  out[0~3]: IfBlock==0, then Q, CtUtq, VtStp, UtStp.
//  out[0~4]: IfBlock==1, then Q, ItgY, DiffYX, VtStp, UtStp.
//  Q: n-by-w matrix, the optimal solution
//
//===========Description-1: Column Defined DP==================
//
//===========Description-2: Block Defined DP==================


//  For Block Defined DP
void MyMin(double *Vs, double *Us, double *VCUtq, int len, double VC0, int n)
{
    int i;
    double tmp;
    *Vs = VC0;
    *Us = 0;
    for(i=0; i<len; i++)
    {
        tmp = VCUtq[i];
        if (*Vs>tmp)
        {
            *Vs = tmp;
            *Us = n+i;
        }
    }
}

void MyMin(double *Vs, double *Us, double *VCUtq, int len, double VC0, int n, int w)
{
    int i,id;
    double tmp;
    *Vs = VC0;
    *Us = 0;
    for(i=0; i<len; i++)
    {
        tmp = VCUtq[i];
        if (*Vs>tmp)
        {
            *Vs = tmp;
            id = i;
        }
    }
    if (id<len)
    {
        if (id>=w)
            *Us = id-w+1;
        else
            *Us = n+id;
    }
}

void CalCost(double *ItgY, double *DiffYX, double *Y, double *X, int h, int w, int n)
{
    // Y: h-by-w; X: h-by-n; ItgY: w+1-by-1; DiffYX: w+n-1-by-1
    int i,j,k;
    double tmp;
    //  1) Integral Image of Y, ItgY(i+1)=sum(sum( Y(:,1:i)~=0 ))
    ItgY[0] = 0;
    for(j=0; j<w; j++)
    {
        tmp = 0;
        for (i=0; i<h; i++)
            tmp += (Y[i+j*h]!=0)?1:0;

        ItgY[j+1] = ItgY[j]+tmp;
    }
    //  2) DiffYX. DiffYX(j):    Func(Y(:,j-N+1:j)-X), N<=j<=W
    int cc;
    for(j=n; j<=w; j++)
    {
        tmp = 0;
        for (k=0; k<n; k++)
        {
            cc = j-n+k;
            for (i=0; i<h; i++)
                tmp += (Y[i+cc*h]!=X[i+k*h])?1:0;
        }
        DiffYX[j-1] = tmp;
    }
}

//  For Column Defined DP
void MyMin(double *OptV, int *SId, double *VtStp, int N)
{
    int i;
    double tmp;
    *OptV = VtStp[0];
    *SId = 0;
    for (i=1; i<N; i++)
    {
        tmp = VtStp[i];
        if (*OptV>tmp)
        {
            *OptV = tmp;
            *SId = i;
        }
    }
}

void CalCtUtq(double* CtUtq, double *Y, double *X, int rows, int cols, int h)
{
    // Y: H-by-W; X: H-by-N; CtUtq: N+1-by-W
    // cols==W; rows==N+1; h==H;
    int j,q,i;
    double tmp;
    for (j=0; j<cols; j++)
    {
        for (q=0; q<rows-1;q++)
        {
            tmp = 0;
            for (i=0; i<h; i++)
            {
                tmp += (Y[i+j*h]!=X[i+q*h])?1:0;
            }
            CtUtq[q+j*rows] = tmp;
        }
        //q = rows-1;
        tmp = 0;
        for (i=0; i<h; i++)
        {
            tmp += (Y[i+j*h]!=0)?1:0;
        }
        CtUtq[q+j*rows] = tmp;
    }
}

void CalCtUtq(double *CtUtq, double *Y, double *X, int rows, int cols, int h,
              int c1, int c2)
{
    // Y: H-by-W; X: H-by-N; CtUtq: N+1-by-W
    // cols==W; rows==N+1; h==H;
    CalCtUtq(CtUtq,Y,X,rows,cols,h);
    double inf = std::numeric_limits<double>::infinity();
    int i,j,r;
    r = 0;
    for (j=c1; j<=c2; j++)
    {
        for (i=0; i<rows; i++)
        {
            if (i!=r)
            {
                CtUtq[i+(j-1)*rows] = inf;
            }
        }
        r++;
    }
}

void MinESingle(double *Q, double *Y, double *X, int H, int W, int N, int IfBlock)
{
    int i,j;
    if (IfBlock==1)
    {
        int s,u;
        int SId,UId;
        //  1.a) Outputs Memory
        double *ItgY = new double[W+1];
        double *DiffYX = new double[W+N-1];
        double *VtStp = new double[W+1];
        double *UtStp = new double[W+1];
        // 1.b) Initialization.
        VtStp[W] = 0;

        //  2.b) CalCost
        CalCost(ItgY,DiffYX,Y,X,H,W,N);

        int LenU = W-N+1;
        double *VCtUtq = new double [LenU];
        //  2.c) VtStp, UtStp
        if (W>2*N-2)
        {
            for (s=W-N; s<=W-2; s++)
            {
                VtStp[s+1] = ItgY[W]-ItgY[s+1];
                UtStp[s+1] = 0;
            }
            for (s = W-N-1; s>=N-1; s--)
            {
                for (u=N; u<=W-1-s; u++)
                    VCtUtq[u-N] = ItgY[s+u-N+1]-ItgY[s+1]+DiffYX[s+u]+VtStp[s+u+1];
                MyMin(&VtStp[s+1],&UtStp[s+1],VCtUtq,W-s-N,ItgY[W]-ItgY[s+1],N);
            }
            // s=-1
            for (u=N; u<=W; u++)
                VCtUtq[u-N] = ItgY[u-N]+DiffYX[u-1]+VtStp[u];
            MyMin(VtStp,UtStp,VCtUtq,LenU,ItgY[W],N);
        }
        else
        {
            for (s=N-1; s<=W-2; s++)
            {
                VtStp[s+1] = ItgY[W]-ItgY[s+1];
                UtStp[s+1] = 0;
            }
            // s=-1
            for (u=N; u<=W; u++)
                VCtUtq[u-N] = ItgY[u-N]+DiffYX[u-1]+VtStp[u];
            MyMin(VtStp,UtStp,VCtUtq,LenU,ItgY[W],N);
        }

        //  2.d) Optimal Solution
        SId = 0;
        UId = (int)UtStp[0];
        int cc;
        while (1)
        {
            if (UId==0)
                break;
            else
            {
                cc = SId+UId;
                for (j=0; j<N; j++)
                    Q[j+(cc-N+j)*N] = 1;
                SId = cc;
                UId = (int)UtStp[SId];
            }
        }

        delete [] VCtUtq;
        delete [] UtStp;
        delete [] VtStp;
        delete [] DiffYX;
        delete [] ItgY;
    }
    else//IfBlock==0;
    {
        double tmp1,tmp2;
        int SId,UId,t,p;
        //  2.a) Outputs Memory.
        int T=W;
        double *CtUtq = new double[(N+1)*T];
        double *VtStp = new double[N*(T+1)];
        double *UtStp = new double[N*T];
        //  2.a.ii) Initializing VtStp & UtStp &Q
        for (i=0; i<N; i++)
            VtStp[i+T*N] = 0;
        for (i=0; i<N-1; i++)
            for (j=0; j<T; j++)
                UtStp[i+j*N] = i+2;

        //  2.b) CalCtUtq
        CalCtUtq(CtUtq,Y,X,N+1,T,H);

        //  2.c) VtStp, UtStp
        if (T>=2*N-2)
        {
            for (t=T-1; t>=T-N+1; t--)
            {
                for (p=N-(T-t+1); p<N-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                VtStp[N-1+t*N] = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                UtStp[N-1+t*N] = 0;
            }
            for(t=T-N; t>=N-1; t--)
            {
                for(p=0; p<N-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                tmp1 = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                tmp2 = VtStp[(t+1)*N]+CtUtq[t*(N+1)];
                if (tmp1<=tmp2)
                {
                    VtStp[N-1+t*N] = tmp1;
                    UtStp[N-1+t*N] = 0;
                }
                else
                {
                    VtStp[N-1+t*N] = tmp2;
                    UtStp[N-1+t*N] = 1;
                }
            }
            for(t=N-2; t>=0; t--)
            {
                for(p=0; p<=t-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                tmp1 = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                tmp2 = VtStp[(t+1)*N]+CtUtq[t*(N+1)];
                if (tmp1<=tmp2)
                {
                    VtStp[N-1+t*N] = tmp1;
                    UtStp[N-1+t*N] = 0;
                }
                else
                {
                    VtStp[N-1+t*N] = tmp2;
                    UtStp[N-1+t*N] = 1;
                }
            }
        }
        else
        {
            for (t=T-1; t>=N-1; t--)
            {
                for (p=N-(T-t+1); p<N-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                VtStp[N-1+t*N] = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                UtStp[N-1+t*N] = 0;
            }
            for (t=N-2; t>=T-N+1; t--)
            {
                for (p=N-(T-t+1); p<=t-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                VtStp[N-1+t*N] = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                UtStp[N-1+t*N] = 0;
            }
            for(t=T-N; t>=0; t--)
            {
                for(p=0; p<=t-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                tmp1 = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                tmp2 = VtStp[(t+1)*N]+CtUtq[t*(N+1)];
                if (tmp1<=tmp2)
                {
                    VtStp[N-1+t*N] = tmp1;
                    UtStp[N-1+t*N] = 0;
                }
                else
                {
                    VtStp[N-1+t*N] = tmp2;
                    UtStp[N-1+t*N] = 1;
                }
            }
        }

        //  2.d) Optimal Solution
        SId = N-1;
        for (t=0; t<T; t++)
        {
            UId = (int)UtStp[SId+t*N];
            if (UId>0)
            {
                Q[UId-1+t*N] = 1;
                SId = UId-1;
            }
            else
                SId = N-1;
        }

        delete [] UtStp;
        delete [] VtStp;
        delete [] CtUtq;
    }
}

void MinESingle(double *Q, double *Y, double *X, int H, int W, int N, int c1, int c2, int IfBlock)
{
    int i,j;
    if (IfBlock==1)
    {
        int s,u;
        int SId,UId;
        //  1.a) Outputs Memory
        double *ItgY = new double[W+1];
        double *DiffYX = new double[W+N-1];
        double *VtStp = new double[W+1];
        double *UtStp = new double[W+1];
        // 1.b) Initialization.
        VtStp[W] = 0;

        //  2.b) CalCost
        CalCost(ItgY,DiffYX,Y,X,H,W,N);

        if(W>2*N-2)
        {
            cout<<"Not Implemented...With c1 and c2, Please Set IfBlock=0..."<<endl;

        }
        else
        {
            for (i=0; i<N; i++)
                Q[i+(c1-1+i)*N] = 1;
        }

        delete [] UtStp;
        delete [] VtStp;
        delete [] DiffYX;
        delete [] ItgY;
    }
    else//IfBlock==0;
    {
        double tmp1,tmp2;
        int SId,UId,t,p;
        //  2.a) Outputs Memory.
        // Initial Value of mxCreateDoubleMatrix is 0.
        int T=W;
        double *CtUtq = new double[(N+1)*T];
        double *VtStp = new double[N*(T+1)];
        double *UtStp = new double[N*T];
        //  2.a.ii) Initializing VtStp & UtStp &Q
        for (i=0; i<N; i++)
            VtStp[i+T*N] = 0;
        for (i=0; i<N-1; i++)
            for (j=0; j<T; j++)
                UtStp[i+j*N] = i+2;

        //  2.b) CalCtUtq
        if (T<2*N-2)
        {
            for (i=0; i<N; i++)
                Q[i+(c1-1+i)*N] = 1;
            return;
        }
        CalCtUtq(CtUtq,Y,X,N+1,T,H,c1,c2);

        //  2.c) VtStp, UtStp
        if (T>=2*N-2)
        {
            for (t=T-1; t>=T-N+1; t--)
            {
                for (p=N-(T-t+1); p<N-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                VtStp[N-1+t*N] = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                UtStp[N-1+t*N] = 0;
            }
            for(t=T-N; t>=N-1; t--)
            {
                for(p=0; p<N-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                tmp1 = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                tmp2 = VtStp[(t+1)*N]+CtUtq[t*(N+1)];
                if (tmp1<=tmp2)
                {
                    VtStp[N-1+t*N] = tmp1;
                    UtStp[N-1+t*N] = 0;
                }
                else
                {
                    VtStp[N-1+t*N] = tmp2;
                    UtStp[N-1+t*N] = 1;
                }
            }
            for(t=N-2; t>=0; t--)
            {
                for(p=0; p<=t-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                tmp1 = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                tmp2 = VtStp[(t+1)*N]+CtUtq[t*(N+1)];
                if (tmp1<=tmp2)
                {
                    VtStp[N-1+t*N] = tmp1;
                    UtStp[N-1+t*N] = 0;
                }
                else
                {
                    VtStp[N-1+t*N] = tmp2;
                    UtStp[N-1+t*N] = 1;
                }
            }
        }
        else
        {
            for (t=T-1; t>=N-1; t--)
            {
                for (p=N-(T-t+1); p<N-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                VtStp[N-1+t*N] = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                UtStp[N-1+t*N] = 0;
            }
            for (t=N-2; t>=T-N+1; t--)
            {
                for (p=N-(T-t+1); p<=t-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                VtStp[N-1+t*N] = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                UtStp[N-1+t*N] = 0;
            }
            for(t=T-N; t>=0; t--)
            {
                for(p=0; p<=t-1; p++)
                    VtStp[p+t*N] = VtStp[p+1+(t+1)*N]+CtUtq[p+1+t*(N+1)];
                tmp1 = VtStp[N-1+(t+1)*N]+CtUtq[N+t*(N+1)];
                tmp2 = VtStp[(t+1)*N]+CtUtq[t*(N+1)];
                if (tmp1<=tmp2)
                {
                    VtStp[N-1+t*N] = tmp1;
                    UtStp[N-1+t*N] = 0;
                }
                else
                {
                    VtStp[N-1+t*N] = tmp2;
                    UtStp[N-1+t*N] = 1;
                }
            }
        }

        //  2.d) Optimal Solution
        SId = N-1;
        for (t=0; t<T; t++)
        {
            UId = (int)UtStp[SId+t*N];
            if (UId>0)
            {
                Q[UId-1+t*N] = 1;
                SId = UId-1;
            }
            else
                SId = N-1;
        }

        delete [] UtStp;
        delete [] VtStp;
        delete [] CtUtq;
    }
}
