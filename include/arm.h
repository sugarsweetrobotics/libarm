#pragma once

#include <math.h>
#include <memory>
#include <vector>

#include <string>
#include <sstream>



#ifdef WIN32
// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBARM_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBARM_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef LIBARM_EXPORTS
#define LIBARM_API __declspec(dllexport)
#else
#define LIBARM_API __declspec(dllimport)
#endif

#else // ifdef WIN32
#define LIBARM_API 

#endif // ifdef WIN32


//typedef double[3][4] HgMatrix;
struct Vector3 {
  double v[3];
};

struct Vector4 {
  double v[4];
};


class HgMatrix;
inline std::string str(const HgMatrix& m);




struct HgMatrix {
  double v[3][4];
  
  HgMatrix(const double value[3][4]) {
    copyFrom(value);
  }

  HgMatrix() {
    for(int i = 0;i < 3;i++) {
      for(int j = 0;j < 4;j++) {
	if (i == j) v[i][j] = 1;
	else v[i][j] = 0;
      }
    }
  }

  HgMatrix(const HgMatrix& hgMatrix) {
    copyFrom(hgMatrix);
  }
  
  void copyFrom(const double value[3][4]) {
    for(int i = 0;i < 3;i++) {
      for(int j = 0;j < 4;j++) {
	v[i][j] = value[i][j];
      }	
    }
  }

  void copyFrom(const HgMatrix& hgMatrix) {
    for(int i = 0;i < 3;i++) {
      for(int j = 0;j < 4;j++) {
	v[i][j] = hgMatrix.v[i][j];
      }
    }
  }
  
  void operator= (const HgMatrix& hgMatrix) { copyFrom(hgMatrix);  }

    std::string operator()(const HgMatrix& m) {
      return str(m);
    }
  

};

inline std::string str(const Vector3& v) {
  std::stringstream ss;
  ss << v.v[0] << ", " << v.v[1] << ", " << v.v[2];
  return ss.str();
}

inline std::string str(const HgMatrix& m) {
  std::stringstream ss;
  ss << m.v[0][0] << " " << m.v[0][1] << " " << m.v[0][2] << " " << m.v[0][3] << std::endl
     << m.v[1][0] << " " << m.v[1][1] << " " << m.v[1][2] << " " << m.v[1][3] << std::endl
     << m.v[2][0] << " " << m.v[2][1] << " " << m.v[2][2] << " " << m.v[2][3] ;
  
  return ss.str();
}


static double identityValues[3][4] = {{1.0, 0, 0, 0}, {0, 1.0, 0, 0}, {0, 0, 1.0, 0}};
static HgMatrix identity(identityValues);


class RobotArm {
 public:
  RobotArm() {}
  virtual ~RobotArm() {}
 public:
  virtual double joint(const int index) = 0;
  virtual HgMatrix getEEPose() = 0;
  virtual std::vector<double> targetJointVelocity() = 0;
};

typedef std::shared_ptr<RobotArm> SPROBOTARM;

std::shared_ptr<RobotArm> createRPPRPR(const double link[3]);
  
SPROBOTARM solveFK(SPROBOTARM arm, const double joints[6]);
SPROBOTARM solveIK(SPROBOTARM arm, const HgMatrix& target);

inline HgMatrix getEEPose(SPROBOTARM arm) {
  return arm->getEEPose();
}

inline double joint(SPROBOTARM arm, const int index) {
  return arm->joint(index);
}

inline Vector3 hgMatrixToEulerZYZ(const HgMatrix& hg) {
  Vector3 v;
  double c2 = hg.v[2][2];
  if (c2 == 1.0) {
    v.v[1] = 0.0;
    v.v[0] = atan2(hg.v[0][1], hg.v[1][1]);
    v.v[2] = 0.0;
    return v;
  }
  
  v.v[1] = acos(c2);
  v.v[0] = atan2(hg.v[1][2], hg.v[0][2]);
  double s2 = sin(v.v[1]);
  double s3 = hg.v[2][1] / s2;
  double c3 = (hg.v[2][0] - c2*s3)/(-s2);
  v.v[2] = atan2(s3, c3);
  return v;
}

inline HgMatrix positionPoseEulerZYZToHgMatrix(const double position[]) {
  double s1 = sin(position[3]); double c1 = cos(position[3]);
  double s2 = sin(position[4]); double c2 = cos(position[4]);
  double s3 = sin(position[5]); double c3 = cos(position[5]);

  HgMatrix result;
  result.v[0][0] = c1*c2*c3+c1*s2*s3-s1*s3;
  result.v[0][1] = -c1*c2*s3-s1*c3;
  result.v[0][2] = c1*s2;
  result.v[1][0] = s1*c2*c3+s1*s2*s3+c1*s3;
  result.v[1][1] = -s1*c2*s3+c1*c3;
  result.v[1][2] = s1*s2;
  result.v[2][0] = -s2*c3+c2*s3;
  result.v[2][1] = s2*s3;
  result.v[2][2] = c2;
  result.v[0][3] = position[0];
  result.v[1][3] = position[1];
  result.v[2][3] = position[2];
  return result;
}


inline HgMatrix positionPoseEulerXYZToHgMatrix(const double position[]) {
  double s1 = sin(position[3]); double c1 = cos(position[3]);
  double s2 = sin(position[4]); double c2 = cos(position[4]);
  double s3 = sin(position[5]); double c3 = cos(position[5]);

  HgMatrix result;
  result.v[0][0] = c2*c3;
  result.v[0][1] = -c2*s3;
  result.v[0][2] = s2;
  result.v[1][0] = s1*s2*c3+c1*s3;
  result.v[1][1] = -s1*s2*s3+c1*c3;
  result.v[1][2] = -s1*c2;
  result.v[2][0] = -c1*s2*c3+s1*s3;
  result.v[2][1] = c1*s2*s3+s1*c3;
  result.v[2][2] = c1*s2;
  result.v[0][3] = position[0];
  result.v[1][3] = position[1];
  result.v[2][3] = position[2];
  return result;
}

inline double sign(double v) {
  return v > 0 ? 1 : -1;
}

inline Vector3 hgMatrixToRPY(const HgMatrix& m) {
  Vector3 a;
  const double EPSILON = 1.0e-8;
  double s2 = m.v[0][2];
  if (fabs(s2-1.0) < EPSILON) {
    a.v[0] = 0;
    a.v[1] = M_PI/2;
    a.v[2] = -atan2(m.v[0][1], m.v[0][0]);
  } else if (fabs(s2+1.0) < EPSILON) {
    a.v[0] = 0;
    a.v[1] = -M_PI/2;
    a.v[2] = -atan2(m.v[0][1], m.v[0][0]);
  } else {
    a.v[1] = asin(s2);
    //double c2 = cos(a.v[1]);
    //a.v[0] = -atan2(m.v[1][2]/c2, m.v[2][2]/s2);
    a.v[0] = -atan2(m.v[1][2], m.v[2][2]);
    a.v[2] = -atan2(m.v[0][1], m.v[0][0]);
  }
  
  return a;
}


inline Vector4 MatrixToRotation(const HgMatrix& m)
{
  double radian;
  double cosTh = ( m.v[0][0] + m.v[1][1] + m.v[2][2] - 1 ) / 2;
  Vector4 axis;
  if( cosTh <= -1.0 ) {
    radian = M_PI;
    if( m.v[0][0] <= -1.0) {
      axis.v[0] = 0.0;
    } else {
      axis.v[0] = sqrt((m.v[0][0] + 1.0) / 2.0);
    }

    if( m.v[1][1] <= -1.0 ) {
      axis.v[1] = 0.0;
    } else {
      axis.v[1] = sign(m.v[0][1]) * sqrt((m.v[1][1] + 1.0) / 2.0);
    }

    if( m.v[2][2] <= -1.0) {
      axis.v[2] = 0.0;
    } else {
      axis.v[2] = sign(m.v[0][2]) * sqrt((m.v[2][2] + 1.0) / 2.0);
    }
    
  } else if (cosTh < 1.0) { // -1.0 < cosTh < 1.0
    axis.v[0] = m.v[2][1] - m.v[1][2]; // Vx * sinTh * 2
    axis.v[1] = m.v[0][2] - m.v[2][0]; // Vy * sinTh * 2
    axis.v[2] = m.v[1][0] - m.v[0][1]; // Vz * sinTh * 2
    double norm = sqrt(axis.v[0]*axis.v[0] + axis.v[1]*axis.v[1] + axis.v[2]*axis.v[2]);
    if(norm == 0.0) {
      radian = 0.0;
      axis.v[0] = 1.0;
      axis.v[1] = axis.v[2] = 0.0;
    } else {
      for(int i = 0;i < 3;i++) {
	axis.v[i] = axis.v[i] / norm;
      }
      double sinTh = norm / 2.0;
      radian = atan2(sinTh, cosTh);
    }
  } else {
    radian = 0.0;
    axis.v[0] = 1.0;
    axis.v[1] = axis.v[2] = 0.0;
  }


  axis.v[3] = radian;
  return axis;
}
