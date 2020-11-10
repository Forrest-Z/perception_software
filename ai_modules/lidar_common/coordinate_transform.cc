#include "lidar_common/coordinate_transform.h"
#include <cmath>

#define FE_WGS84    (1.0/298.257223563)
#define RE_WGS84    (6378137.0)

namespace perception {
namespace lidar {

CoordinateTransform::CoordinateTransform()
{
    basePose[0] = 0;
    basePose[1] = 0;
    basePose[2] = 0;
}

CoordinateTransform::~CoordinateTransform()
{

}

void CoordinateTransform::setBasePose(double latitude, double longitude, double altitude)
{
    Eigen::Vector3d temp;
    temp[0] = latitude;
    temp[1] = longitude;
    temp[2] = altitude;
    basePose = poseToecef(temp);
}

Eigen::Vector3d CoordinateTransform::poseToEnu(double latitude, double longitude, double altitude)
{
    Eigen::Vector3d result;
    Eigen::Vector3d pose;
    double enu[3], dlt_ecef[3];
    pose[0] = latitude;
    pose[1] = longitude;
    pose[2] = altitude;
    Eigen::Vector3d ecef = poseToecef(pose);
    dlt_ecef[0] = ecef[0] - basePose[0];
    dlt_ecef[1] = ecef[1] - basePose[1];
    dlt_ecef[2] = ecef[2] - basePose[2];
    ecef2enu(pose, dlt_ecef, enu);
    result[0] = enu[0];
    result[1] = enu[1];
    result[2] = enu[2];
    return result;
}

/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
* return : ecef position {x,y,z} (m)
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
Eigen::Vector3d CoordinateTransform::poseToecef(const Eigen::Vector3d &pose)
{
    Eigen::Vector3d resut;
    Eigen::Vector3d pos_r;
    pos_r[0] = pose[0] * M_PI / 180;
    pos_r[1] = pose[1] * M_PI / 180;
    pos_r[2] = pose[2] * M_PI / 180;
    double sinp= sin(pos_r[0]);
    double cosp=cos(pos_r[0]);
    double sinl=sin(pos_r[1]);
    double cosl=cos(pos_r[1]);
    double e2= FE_WGS84*(2.0-FE_WGS84);
    double v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);

    resut[0]=(v+pos_r[2])*cosp*cosl;
    resut[1]=(v+pos_r[2])*cosp*sinl;
    resut[2]=(v*(1.0-e2)+pos_r[2])*sinp;
    return resut;
}

/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
void CoordinateTransform::ecef2enu(const Eigen::Vector3d &pose, const double *r, double *e)
{
    double E[9];

    xyz2enu(pose, E);
    matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

Eigen::Quaternionf CoordinateTransform::getRotation(float angularX, float angularY, float angularZ)
{
    float w = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    w = std::cos(angularX/2) * std::cos(angularY/2) * std::cos(angularZ/2) +
            std::sin(angularX/2) * std::sin(angularY/2) * std::sin(angularZ/2);
    x = std::sin(angularX/2) * std::cos(angularY/2) * std::cos(angularZ/2) -
            std::cos(angularX/2) * std::sin(angularY/2) * std::sin(angularZ/2);
    y = std::cos(angularX/2) * std::sin(angularY/2) * std::cos(angularZ/2) +
            std::sin(angularX/2) * std::cos(angularY/2) * std::sin(angularZ/2);
    z = std::cos(angularX/2) * std::cos(angularY/2) * std::sin(angularZ/2) -
            std::sin(angularX/2) * std::sin(angularY/2) * std::cos(angularZ/2);
    return Eigen::Quaternionf(w, x, y, z);
}


/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
void CoordinateTransform::xyz2enu(const Eigen::Vector3d &pose, double *E)
{
    Eigen::Vector3d pos_r;
    pos_r[0] = pose[0] * M_PI / 180;
    pos_r[1] = pose[1] * M_PI / 180;
    pos_r[2] = pose[2] * M_PI / 180;

    double sinp=sin(pos_r[0]),cosp=cos(pos_r[0]),sinl=sin(pos_r[1]),cosl=cos(pos_r[1]);

    E[0]=-sinl;      E[3]=cosl;       E[6]=0.0;
    E[1]=-sinp*cosl; E[4]=-sinp*sinl; E[7]=cosp;
    E[2]=cosp*cosl;  E[5]=cosp*sinl;  E[8]=sinp;
}

/* multiply matrix (wrapper of blas dgemm) -------------------------------------
* multiply matrix by matrix (C=alpha*A*B+beta*C)
* args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
*          int    n,k,m     I  size of (transposed) matrix A,B
*          double alpha     I  alpha
*          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
*          double beta      I  beta
*          double *C        IO matrix C (n x k)
* return : none
*-----------------------------------------------------------------------------*/
void CoordinateTransform::matmul(const char *tr, int n, int k, int m, double alpha,
                   const double *A, const double *B, double beta, double *C)
{
    double d;
    int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

    for (i=0;i<n;i++) for (j=0;j<k;j++)
    {
        d=0.0;
        switch (f)
        {
            case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
            case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
            case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
            case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
        }
        if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
    }
}

}  // namespace lidar
}  // namespace perception
