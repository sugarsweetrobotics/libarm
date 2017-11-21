#include <arm.h>
#include <iostream>

#include <math.h>


double links[] = {111, 222, 333};

int assertTrue(bool flag) {
  if (flag) {
    std::cout << "..........OK." << std::endl;
    return 0;
  }

  std::cout << "..........FAILED." << std::endl;
  return 1;
}


bool test_fk(double joints[], double position[]) {
  std::cout << "Starting Test FK" << std::endl;
  double EPSILON = 0.01;
  SPROBOTARM solved1 = solveFK(createRPPRPR(links), joints);
  HgMatrix hg = getEEPose(solved1);
  Vector3 p = hgMatrixToEulerZYZ(hg);
  Vector3 a = hgMatrixToRPY(hg);

  std::cout << " - arg - Angle=(" << joints[0] << ", " << joints[1] << ", " << joints[2] << ", " << joints[3] << ", " << joints[4] << ", " << joints[5] << ")" << std::endl;
  std::cout << " - arg - Target (" << position[0] << ", " << position[1] << ", " << position[2] << ", " 
	    << position[3] << ", " << position[4] << ", " << position[5] << ")" << std::endl;

  std::cout << " - Result - Current Mat  = " << std::endl << str(hg) << std::endl;
  std::cout << " - Result - Current PoseXYZRPY =(" << hg.v[0][3] << ", " << hg.v[1][3] << ", " << hg.v[2][3] 
	    << ", " << a.v[0] << ", " << a.v[1] << ", " << a.v[2] << ")" << std::endl;
  for(int i = 0;i < 3;i++) { 
    if (fabs(hg.v[i][3] - position[i]) > EPSILON) {
      std::cout << "position[" << i << "] is invalid." << std::endl;
      return false;
    }
    double diffAngle = (a.v[i] - position[i+3]);
    while(diffAngle >= M_PI) {
      diffAngle -= M_PI*2;
    }
    while(diffAngle < -M_PI) {
      diffAngle += M_PI+2;
    }
      
    if (diffAngle > EPSILON) {
      std::cout << "position[" << i+3 << "] is invalid." << std::endl;
      return false;
    }
  }
  return true;
}

bool test_ik(double joints[], double position[], double res_joints[]); 

bool test_ik(double joints[], double res_joints[]) {
  SPROBOTARM solved1 = solveFK(createRPPRPR(links), res_joints);
  HgMatrix hg = getEEPose(solved1);
  Vector3 a = hgMatrixToRPY(hg);
  double position[] = {hg.v[0][3], hg.v[1][3], hg.v[2][3], a.v[0], a.v[1], a.v[2]};
  return test_ik(joints, position, res_joints);
}

bool test_ik(double joints[], double position[], double res_joints[]) {
  std::cout << "Starting Test IK" << std::endl;
  std::cout << " arg - Current Joints(" << joints[0] << ", " << joints[1] << ", " << joints[2] 
	    << ", " << joints[3] << ", " << joints[4] << ", " << joints[5]<< ")" << std::endl;
  //double EPSILN = 0.000001;
  SPROBOTARM solved1 = solveFK(createRPPRPR(links), joints);
  HgMatrix hg = getEEPose(solved1);

  Vector3 p = hgMatrixToRPY(hg);
  std::cout << " arg - CurrentPoseXYZRPY(" << hg.v[0][3] << ", " << hg.v[1][3] << ", " << hg.v[2][3] 
	    << ", " << p.v[0] << ", " << p.v[1] << ", " << p.v[2] << ")" << std::endl;

  HgMatrix h = positionPoseEulerXYZToHgMatrix(position);
  std::cout << " arg -Target(" << position[0] << ", " << position[1] << ", " << position[2] << ", " 
	    << position[3] << ", " << position[4] << ", " << position[5] << ")" << std::endl; 
  std::cout << " arg - Target --> positionPoseEulerXYZToHgMatrix = " << std::endl << str(h) << std::endl;


  SPROBOTARM solved2 = solveIK(solved1, h);
  double EPSILON = 0.3;
  std::cout << "target_joint (";
  for(int i = 0;i < 6;i++) {
    std::cout << res_joints[i] << ", ";
  }
  std::cout << ")" << std::endl;

  std::cout << "result_joint (" ;
  double joints2[6];
  for(int i = 0;i < 6;i++) {
    std::cout << joint(solved2,i) << ", ";
    joints2[i] = joint(solved2,i);
  }
  std::cout << ")" << std::endl;

  
  SPROBOTARM solved3 = solveFK(createRPPRPR(links), res_joints);
  HgMatrix hg3 = getEEPose(solved3);
  std::cout << "target: " << std::endl << str(h) << std::endl;
  std::cout << "p: " << str(hgMatrixToRPY(h)) << std::endl;
  
  HgMatrix hg2 = getEEPose(solved2);
  std::cout << "result: " << std::endl << str(hg2) << std::endl;
  std::cout << "p: " << str(hgMatrixToRPY(hg2)) << std::endl;

  double dx = hg2.v[0][3] - hg3.v[0][3];
  double dy = hg2.v[1][3] - hg3.v[1][3];
  double dz = hg2.v[2][3] - hg3.v[2][3];
  std::cout << "Distance=" << sqrt(dx*dx + dy*dy + dz*dz) << std::endl;
  bool flag = true;
  for(int i = 0;i < 6;i++) {
    if (fabs(joint(solved2, i) - res_joints[i]) > EPSILON) flag = false;
  }
  std::cout << std::endl;
  return flag;
  
}

int test_count = 0;



int fk_test(void) {
  std::cout << "Helllo" << std::endl;
  int count = 0;
  test_count = 0;

  double test1[][6] = {
    {0, 0, 0, 0, 0, 0},
    {0, 0, 666, 0, 0, 0},
  };
  count += assertTrue(test_fk(test1[0], test1[1]));
  test_count++;

  double test2[][6] = {
    {M_PI/2, 0, 0, 0, 0, 0},
    {0, 0, 666,  0, 0, M_PI/2},
  };
  count += assertTrue(test_fk(test2[0], test2[1]));
  test_count++;

  double test3[][6] = {
    {0, M_PI/2, 0, 0, 0, 0},
    {555, 0, 111, 0, M_PI/2, 0},
  };
  count += assertTrue(test_fk(test3[0], test3[1]));
  test_count++;

  double test3_[][6] = {
    {0, 0, 0, 0, M_PI/2, 0},
    {333, 0, 333, 0, M_PI/2, 0},
  };
  count += assertTrue(test_fk(test3_[0], test3_[1]));
  test_count++;


  double test4[][6] = {
    {0, M_PI/2, 0, 0, 0, 1.0},
    {555, 0, 111, 0, M_PI/2, 1.0},
  };
  count += assertTrue(test_fk(test4[0], test4[1]));
  test_count++;

  double test5[][6] = {
    {0.0001, M_PI/3, M_PI/4, 0.00001, M_PI/10, 0},
    {471.535, 0.0481825, 40.6352, -3.14135, 0.994838, 3.14159}

  };
  count += assertTrue(test_fk(test5[0], test5[1]));
  test_count++;

  return count;
}


int ik_test(void) {
  std::cout << "Starting ik_test" << std::endl;
  int count = 0;
  test_count = 0;


  double test1[][6] = {
    {0.001, M_PI/4, M_PI/5, 0.00001, M_PI/7, 0.001},
    //{471.535, 0.0481825, 40.6352, -3.14135, 0.994838, 3.14159},
    {0.001, M_PI/4, M_PI/4, 0.00001, M_PI/10, 0},
  };

  count += assertTrue(test_ik(test1[0], test1[1]));//, test1[2]));
  test_count++;

  double test2[][6] = {
    {M_PI/4, M_PI/4, M_PI/5, M_PI/5, M_PI/10, 0.001},
    //{471.535, 0.0481825, 40.6352, -3.14135, 0.994838, 3.14159},
    {M_PI/5, M_PI/4, M_PI/4, M_PI/4, M_PI/10, 0},
  };

  count += assertTrue(test_ik(test2[0], test2[1]));//, test1[2]));
  test_count++;


  double test3[][6] = {
    {M_PI/8, M_PI/4, M_PI/5, -M_PI/8, 0.001, -0.1},
    //{471.535, 0.0481825, 40.6352, -3.14135, 0.994838, 3.14159},
    {M_PI/5, M_PI/4, M_PI/4, M_PI/8, M_PI/10, 0},
  };

  count += assertTrue(test_ik(test3[0], test3[1]));//, test1[2]));
  test_count++;

  return count;
}


int main(void) {
  int fk_err_count = fk_test();
  int fk_test_count = test_count;

  int ik_err_count = ik_test();
  int ik_test_count = test_count;

  std::cout << "Test FK (" << fk_err_count << "/" << fk_test_count << " Failed.)" << std::endl;
  std::cout << "Test IK (" << ik_err_count << "/" << ik_test_count << " Failed.)" << std::endl;
  return 0;
}

