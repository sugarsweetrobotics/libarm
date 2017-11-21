#include <iostream>

#include <arm.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>

inline double normalize_angle(double angle) {
  while(angle > M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

typedef Eigen::Matrix<double, 4, 4> MATRIX44;
typedef Eigen::Matrix<double, 6, 6> MATRIX66;
typedef Eigen::Vector4d VECTOR4;
typedef Eigen::Matrix<double, 6, 1> VECTOR6;

class RobotArm6AxisRPPRPR : public RobotArm {
 public:
  double L[3];
  double jointAngles[6];
  MATRIX44 jointMat[6];
  MATRIX44 jointMatAbs[7];
  VECTOR4 jointToJointDistance[4];
  //  VECTOR6 jointVelocity;
 public: 
  RobotArm6AxisRPPRPR() {}
  virtual ~RobotArm6AxisRPPRPR() {}

  RobotArm6AxisRPPRPR(const RobotArm6AxisRPPRPR& robot) {
    copyFrom(robot);
  }

  void copyFrom(const RobotArm6AxisRPPRPR& robot) {
    for(int i = 0;i < 3;i++) {
      L[i] = robot.L[i];
    }
	       
    for(int i = 0;i < 6;i++) {
      jointAngles[i] = robot.jointAngles[i];
      jointMat[i] = robot.jointMat[i];
    }

    for(int i = 0;i < 7;i++) {
      jointMatAbs[i] = robot.jointMatAbs[i];
    }
  }

 public:

  virtual HgMatrix getEEPose()  {
    HgMatrix hg;
    for (int i = 0;i < 3;i++) {
      for (int j = 0;j < 4;j++) {
	hg.v[i][j] = jointMatAbs[6](i, j);

      }
    }
    return hg;
  }

  virtual double joint(const int index) {
    return jointAngles[index];
  }

  virtual std::vector<double> targetJointVelocity()  {

  }
 public:
  void operator=(const RobotArm6AxisRPPRPR& robot) {
    copyFrom(robot);
  }
};


SPROBOTARM createRPPRPR(const double link[3]) {
  RobotArm6AxisRPPRPR* pRobot = new RobotArm6AxisRPPRPR();;
  pRobot->L[0] = link[0];
  pRobot->L[1] = link[1];
  pRobot->L[2] = link[2];

  return std::shared_ptr<RobotArm6AxisRPPRPR>(pRobot);
}



MATRIX44 RotTransX(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  //result(0, 0) = 1; result(0, 1) = 0; result(0, 2) = 0;
  /* result(1, 0) = 0;*/ result(1, 1) = c; result(1, 2) =-s;
  /* result(2, 0) = 0;*/ result(2, 1) = s; result(2, 2) = c;
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;
  return result;
}


MATRIX44 RotTransY(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  result(0, 0) = c; /*result(0, 1) = 0;*/ result(0, 2) = s;
  //result(1, 0) = 0; result(1, 1) = 1; result(1, 2) = 0; 
  result(2, 0) =-s; /*result(2, 1) = 0;*/ result(2, 2) = c;
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;
  return result;
}

MATRIX44 RotTransZ(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  result(0, 0) = c; result(0, 1) = -s; /*result(0, 2) = 0;*/
  result(1, 0) = s; result(1, 1) = c; /*result(1, 2) = 0;*/
  //result(2, 0) = 0; result(2, 1) = 0; result(2, 2) = 1; 
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;

  return result;
}


//MATRIX33 updateJointRotation(const double jointAngles[6]) {
  //  std::vector<MATRIX33> mats{RotZ(jointAngles[0]), RotY(jointAngles[1]), RotY(jointAngles[2]), RotZ(jointAngles[3]), RotY(jointAngles[4]), RotZ(jointAngles[5])};
//  return mats
//}

SPROBOTARM solveFK(SPROBOTARM arm_, const double joints[6]) {
  //  std::cout << "solving" << std::endl;
  RobotArm6AxisRPPRPR *pRobot = new RobotArm6AxisRPPRPR();
  RobotArm6AxisRPPRPR &new_robot = *pRobot;

  std::shared_ptr<RobotArm6AxisRPPRPR> arm = std::dynamic_pointer_cast<RobotArm6AxisRPPRPR>(arm_);


  for(int i = 0;i < 3;i++) {
    new_robot.L[i] = arm->L[i];
  }

  for(int i = 0;i < 6;i++) {
    new_robot.jointAngles[i] = joints[i];
  }

  //  std::cout << "hoge" << std::endl;
  //new_robot.jointMat[0] = RotTransZ(joints[0], 0, 0, 0);
  new_robot.jointMat[0] = RotTransZ(joints[0], 0, 0, new_robot.L[0]);
  new_robot.jointMat[1] = RotTransY(joints[1], 0, 0, 0);
  new_robot.jointMat[2] = RotTransY(joints[2], 0, 0, new_robot.L[1]);
  new_robot.jointMat[3] = RotTransZ(joints[3], 0, 0, 0);
  new_robot.jointMat[4] = RotTransY(joints[4], 0, 0, 0);
  new_robot.jointMat[5] = RotTransZ(joints[5], 0, 0, new_robot.L[2]);

  /*
  for(int i = 0;i < 6;i++) {
    std::cout << "-------jointMat[" << i << "]-------" << std::endl;
    std::cout << new_robot.jointMat[i] << std::endl;
  }
  */

  new_robot.jointMatAbs[0] = MATRIX44::Identity(4,4);
  for(int i = 0;i < 6;i++) {
    new_robot.jointMatAbs[i+1] = new_robot.jointMatAbs[i] * new_robot.jointMat[i];
    //   new_robot.jointMatAbs[i+1] = new_robot.jointMat[i] * new_robot.jointMatAbs[i];
  }
  /*
  for(int i = 0;i < 7;i++) {
    std::cout << "-------Abs" << i << "-------" << std::endl;
    std::cout << new_robot.jointMatAbs[i] << std::endl;
  }
  */
  return std::shared_ptr<RobotArm6AxisRPPRPR>(pRobot);
}



MATRIX66 calcJ(SPROBOTARM arm_) {
  std::shared_ptr<RobotArm6AxisRPPRPR> arm = std::dynamic_pointer_cast<RobotArm6AxisRPPRPR>(arm_);
  MATRIX66 jacobian;

  double joints[6];
  for(int i = 0;i < 6;i++) {
    joints[i] = joint(arm_, i);
  }
  HgMatrix hg0 = getEEPose(arm);
  Vector3 rpy0 = hgMatrixToRPY(hg0);
  /*
  std::cout << "hg0" << std::endl << str(hg0) << std::endl;
    std::cout << "joints=";
    for(int j = 0;j < 6;j++) {
      std::cout << joints[j] << ", ";
    }
    std::cout << std::endl;
  */
  double epsilon = 1.0e-3;
  MATRIX66 j2;
  double joints2[6];
  for(int i = 0;i < 6;i++) {
    for(int j = 0;j < 6;j++) {
      if (i == j) {
	joints2[j] = joints[j] + epsilon;
      } else {
	joints2[j] = joints[j];
      }
    }
    SPROBOTARM arm2 = solveFK(arm, joints2);
    HgMatrix hg = getEEPose(arm2);
    /*
    std::cout << "j(" << i << ")" << std::endl;
    std::cout << str(hg) << std::endl;
    std::cout << "joints2=";
    for(int j = 0;j < 6;j++) {
      std::cout << joints2[j] << ", ";
    }
    std::cout << std::endl;
    */
    double d[6];
    d[0] = hg.v[0][3] - hg0.v[0][3];
    d[1] = hg.v[1][3] - hg0.v[1][3];
    d[2] = hg.v[2][3] - hg0.v[2][3];
    Vector3 rpy = hgMatrixToRPY(hg);
    d[3] = normalize_angle(rpy.v[0] - rpy0.v[0]);
    d[4] = normalize_angle(rpy.v[1] - rpy0.v[1]);
    d[5] = normalize_angle(rpy.v[2] - rpy0.v[2]);
    
    for(int j = 0;j < 6;j++) {
      j2(j, i) = d[j];
    }
  }
    
  VECTOR4 axis[6] = {
    arm->jointMatAbs[1].col(2),
    arm->jointMatAbs[2].col(1),
    arm->jointMatAbs[3].col(1),
    arm->jointMatAbs[4].col(2),
    arm->jointMatAbs[5].col(1),
    arm->jointMatAbs[6].col(2)
  };

  VECTOR4 offset[3] = {
    arm->L[0] * arm->jointMatAbs[1].col(2),
    arm->L[1] * arm->jointMatAbs[3].col(2),
    arm->L[2] * arm->jointMatAbs[6].col(2),
  };

  VECTOR4 vec[6];

  vec[5] = offset[2];
  vec[4] = vec[5];
  vec[3] = vec[4];
  vec[2] = vec[3] + offset[1];
  vec[1] = vec[2];
  vec[0] = vec[1] + offset[0];
  
  
  
  /*
  for(int i = 0;i < 6;i++) {
    std::cout << "axis[" << i << "]=" << std::endl << axis[i] << std::endl;
    }*/

  MATRIX44 invMat = MATRIX44::Identity(4, 4);
  //  MATRIX44 invMat2 = invMat;
  for(int i = 5;i >= 0;i--) {
    //std::cout << "foo" << i << std::endl;

    MATRIX44 mat = MATRIX44::Identity(4,4);
    for(int j = 0;j < (5-i);j++) {
    mat = mat * arm->jointMat[j];
    }
    //invMat = invMat * arm->jointMat[i];
    invMat = arm->jointMat[i] * invMat;
    //VECTOR4 diff = invMat.col(3);
    VECTOR4 diff = vec[i];
    
    //VECTOR4 diff = mat.col(3);
    //std::cout << "inv(" << i << "): " << std::endl << diff << std::endl;
    //VECTOR4 diff = arm->jointMatAbs[6].col(3) - arm->jointMatAbs[i].col(3);
    jacobian(0, i) = axis[i](1) * diff(2) - axis[i](2) * diff(1);
    jacobian(1, i) = axis[i](2) * diff(0) - axis[i](0) * diff(2);
    jacobian(2, i) = axis[i](0) * diff(1) - axis[i](1) * diff(0);
    jacobian(3, i) = axis[i](0);
    jacobian(4, i) = axis[i](1);
    jacobian(5, i) = axis[i](2);
    //    invMat2 = invMat;
    //invMat = arm->jointMat[i] * invMat;

  }

  //std::cout << "J.........." << std::endl << jacobian << std::endl;
  //std::cout << "j2........." << std::endl << j2 << std::endl;

  return jacobian;//.transpose();
  //  return j2;
}

MATRIX66 calcInvJ(const MATRIX66& jacobian) {
  Eigen::JacobiSVD< MATRIX66 > svd(jacobian, Eigen::ComputeFullU |
				       Eigen::ComputeFullV);

  Eigen::JacobiSVD<MATRIX66>::SingularValuesType sigma(svd.singularValues());
  //  std::cout << "singular values" << std::endl << svd.singularValues() << std::endl;
  //std::cout << "matrix U" << std::endl << svd.matrixU() << std::endl;
  //std::cout << "matrix V" << std::endl << svd.matrixV() << std::endl;

  Eigen::JacobiSVD<MATRIX66>::SingularValuesType sigma_inv(sigma.size());
  double tolerance = 1.e-6;
  std::cout << "sigam(";
  for(int i=0; i< sigma.size(); ++i) {
    std::cout << sigma(i) << ", ";
    
    sigma_inv(i) = sigma(i) > tolerance ? 1.0/sigma(i) : 0.0;
  }
  std::cout << std::endl;

  auto J = svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();
  //std::cout << "-Jacobian = " << std::endl << J << std::endl;
  return J;

  //std::cout << "-Jacobian = " << std::endl << jacobian.inverse() << std::endl;
  //return jacobian.inverse();
}

HgMatrix ToHgMatrix(const MATRIX44& m) {
  HgMatrix hm;
  for(int i = 0;i < 3;i++) {
    for(int j = 0;j < 4;j++) {
      hm.v[i][j] = m(i, j);
    }
  }

  return hm;
}


MATRIX44 ToMatrix44(const HgMatrix& m) {
  MATRIX44 d = MATRIX44::Identity(4,4);
  for(int i = 0;i < 3;i++) {
    for(int j = 0;j < 4;j++) {
      d(i, j) = m.v[i][j];
    }
  }
  return d;
}

void MatrixToRotation(const MATRIX44& matrix, VECTOR4& axis, double* radian)
{
  axis(3) = 0;
  double cosTh = ( matrix(0,0) + matrix(1,1) + matrix(2,2) - 1 ) / 2;
  if( cosTh <= -1.0 ) {
    *radian = M_PI;
    if( matrix(0,0) <= -1.0) {
      axis(0) = 0.0;
    } else {
      axis(0) = sqrt((matrix(0,0) + 1.0) / 2.0);
    }

    if( matrix(1,1) <= -1.0 ) {
      axis(1) = 0.0;
    } else {
      axis(1) = sign(matrix(0,1)) * sqrt((matrix(1,1) + 1.0) / 2.0);
    }

    if( matrix(2,2) <= -1.0) {
      axis(2) = 0.0;
    } else {
      axis(2) = sign(matrix(0,2)) * sqrt((matrix(2,2) + 1.0) / 2.0);
    }

  } else if (cosTh < 1.0) { // -1.0 < cosTh < 1.0
    axis(0) = matrix(2,1) - matrix(1,2); // Vx * sinTh * 2
    axis(1) = matrix(0,2) - matrix(2,0); // Vy * sinTh * 2
    axis(2) = matrix(1,0) - matrix(0,1); // Vz * sinTh * 2
    double norm = axis.norm();
    //double norm = sqrt(axis % axis);
    if(norm == 0.0) {
      *radian = 0.0;
      axis(0) = 1.0;
      axis(1) = axis(2) = 0.0;
    } else {
      for(int i = 0;i < 3;i++) {
	axis(i) = axis(i) / norm;
      }
      double sinTh = norm / 2.0;
      *radian = atan2(sinTh, cosTh);
    }
  } else {
    *radian = 0.0;
    axis(0) = 1.0;
    axis(1) = axis(2) = 0.0;
  }
}


VECTOR4 calcVelocity(MATRIX44& current, MATRIX44 &target, double maxVelocity) {
  MATRIX44 rot = current.transpose() * target; // Get Rotation from Current To Target
  VECTOR4 axis;
  double radian;
  MatrixToRotation(rot, axis, &radian);

  axis = current * axis;
  radian = fabs(radian) > maxVelocity ? sign(radian) * maxVelocity : radian;
  axis(3) = radian;
  return axis;
}


SPROBOTARM solveIK(SPROBOTARM arm_, const HgMatrix& target) {
  Vector3 pose = hgMatrixToRPY(target);
  //std::cout << "Target(" << target.v[0][3] << ", " << target.v[1][3] << ", " << target.v[2][3] << ", "  
  //	    << pose.v[0] << ", " << pose.v[1] << ", " << pose.v[2]
  //<< ")" <<  std::endl;
  std::shared_ptr<RobotArm6AxisRPPRPR> arm = std::dynamic_pointer_cast<RobotArm6AxisRPPRPR>(arm_);
  MATRIX44 targetMatrix = ToMatrix44(target);

  //std::cout << "target = " << std::endl << str(target) << std::endl;
  //std::string c;
  //std::getline(std::cin, c);

  for(int i = 0;i < 10000;i++) {

    double maxRotVelocity = 0.1;
    VECTOR4 vel = calcVelocity(arm->jointMatAbs[6], targetMatrix, maxRotVelocity);
    Vector3 current_pose = hgMatrixToRPY(ToHgMatrix(arm->jointMatAbs[6]));

    VECTOR6 d;
    d(0) = target.v[0][3] - arm->jointMatAbs[6](0,3);
    d(1) = target.v[1][3] - arm->jointMatAbs[6](1,3);
    d(2) = target.v[2][3] - arm->jointMatAbs[6](2,3);
    d(3) = vel(0) * vel(3);
    d(4) = vel(1) * vel(3);
    d(5) = vel(2) * vel(3);
    //d(3) = pose.v[0] - current_pose.v[0];
    //d(4) = pose.v[1] - current_pose.v[1];
    //d(5) = pose.v[2] - current_pose.v[2];
    double distance = sqrt(d(0)*d(0) + d(1)*d(1) + d(2)*d(2));
    //std::cout << "Pose(" 
    //<< arm->jointMatAbs[6](0,3) << ", " << arm->jointMatAbs[6](1,3) << ", " << arm->jointMatAbs[6](2,3) << ", "
    //<< current_pose.v[0] << ", " << current_pose.v[1] << ", " << current_pose.v[2] << ")" 
    std::cout << "[Distance=" << distance << ", " << d(3) << ", " << d(4) << ", " << d(5) << "]" << std::endl;
    std::string strbuf;
    //std::getline(std::cin, strbuf);

    double tolerance = 0.1;
    double angularTolerance = 0.02;
    bool angF = true;
    for(int i = 0;i < 3;i++) {
      if (fabs(d(3+i) ) > angularTolerance) {
	angF = false;
      }
    }

    if (distance < tolerance && angF) {
      std::cout << "Converge." << std::endl;
      return arm;
    }


    for(int i = 0;i < 6;i++) {
      ///      d(i) = -d(i);
    }

    double maxV = 0.0;
    for(int i = 0;i < 3;i++) {
      if (fabs(d(i)) > maxV) maxV = fabs(d(i));
    }
    VECTOR6 v;
    double res = 1.0;
    if (maxV > res) {
      v(0) = d(0) / maxV * res;
      v(1) = d(1) / maxV * res;
      v(2) = d(2) / maxV * res;
    } else {
      v(0) = d(0);
      v(1) = d(1);
      v(2) = d(2);
    }



    double maxA = 0.0;
    for(int i = 0;i < 3;i++) {
      double diffAngle = d(i+3);
      while(diffAngle >= M_PI) {
	diffAngle -= M_PI*2;
      }
      while(diffAngle < -M_PI) {
	diffAngle += M_PI*2;
      }

      d(i+3) = diffAngle;
      if (fabs(d(i+3)) > maxA) maxA = fabs(d(i+3));
    }

    double angularRes = 0.01;
    if (maxA > angularRes) {
      v(3) = d(3) / maxA * angularRes;
      v(4) = d(4) / maxA * angularRes;
      v(5) = d(5) / maxA * angularRes;
    } else {
      v(3) = d(3);
      v(4) = d(4);
      v(5) = d(5);
    }

    MATRIX66 J = calcJ(arm);
    MATRIX66 invJ = calcInvJ(J);
    
    //std::cout << "DIFF  " << std::endl << v << std::endl;
    VECTOR6 velocity = invJ * v;
    //std::cout << "RESULT " << std::endl << velocity << std::endl;
    
    double joints[6];
    for(int i = 0;i < 6;i++) {
      joints[i] = arm->jointAngles[i] + velocity(i) / 2;
    }
    
    SPROBOTARM arm__ = solveFK(arm_, joints);
    arm = std::dynamic_pointer_cast<RobotArm6AxisRPPRPR>(arm__);
    //std::cout << "T = " << std::endl << str(getEEPose(arm__)) << std::endl;
    //  std::cout << "POS " << std::endl << arm->jointMatAbs[6] << std::endl;
    //std::cout << "targret" << std::endl << str(target) << std::endl;
  }


  return arm;
}

