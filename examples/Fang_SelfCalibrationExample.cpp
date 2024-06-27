

#include <gtsam/slam/Fang_SelfCalibrationWrapper.h> 


using namespace std;
using namespace gtsam;



/* ************************************************************************* */
/**
 * Test GTSAM
 */
std::vector<gtsam::Point3> createPoints() {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,10.0));
  points.push_back(gtsam::Point3(10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
  points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
  points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

  return points;
}

std::vector<gtsam::Pose3> createPoses(
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,-M_PI/2), gtsam::Point3(30, 0, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,-M_PI/4,0), gtsam::Point3(sin(M_PI/4)*30, 0, 30*(1-sin(M_PI/4)))),
            size_t steps = 2) {

  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
  std::vector<gtsam::Pose3> poses;
  size_t i = 1;
  poses.push_back(init);
  for(; i < steps; ++i) {
    poses.push_back(poses[i-1].compose(delta));
  }

  return poses;
}
/* ************************************************************************* */




typedef gtsam::SelfCalibrationForward FGO;
// typedef gtsam::SelfCalibrationReverse FGO;


int main(int argc, char* argv[]) {


  // Create the set of ground-truth
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // Simulated measurements from each camera pose, adding them to the factor graph
  Cal3_f K(50.0, 50.0, 0.0, 50.0, 50.0);


  FGO calGraph;


  for (size_t i = 0; i < poses.size(); ++i) {
    for (size_t j = 0; j < points.size(); ++j) {
      PinholeCamera<FGO::CALIBRATION_MODEL> camera(poses[i], K);
      Point2 measurement = camera.project(points[j]);

      calGraph.add_keypoints_2d (i, j, measurement, 1.0);
    }
  }


  calGraph.addPosePrior(0, poses[0], 0.1, 0.3);
  calGraph.addCalibrationPrior(K);
  calGraph.addLandmarkPrior (0, points[0], 0.1);


  /** initial estimates */
  auto var_K = FGO::CALIBRATION_MODEL(500.0, 500.0, 0.0, 50.0, 50.0);


  auto var_poses = poses;
  for (size_t i = 0; i < var_poses.size(); ++i) {
    var_poses[i] = var_poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                               Point3(0.05, -0.10, 0.20)));
  }
   
  auto var_points = points;
  for (size_t j = 0; j < var_points.size(); ++j) {
    var_points[j] = var_points[j] + Point3(-0.25, 0.20, 0.15);
  }



  // calGraph.optimize_from(var_K, var_poses, var_points);

  auto var_D = FGO::LENSDISTORT_MODEL(0.01);
  calGraph.optimize_from(var_K, var_D, var_poses, var_points);

  return 0;
}



