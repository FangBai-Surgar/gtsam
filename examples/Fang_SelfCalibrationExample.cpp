

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






int main(int argc, char* argv[]) {


  // Create the set of ground-truth
  vector<Point3> points = createPoints();
  vector<Pose3> poses = createPoses();

  // Simulated measurements from each camera pose, adding them to the factor graph
  Cal3_f K(50.0, 50.0, 0.0, 50.0, 50.0);


  typedef gtsam::SelfCalibrationForward FGO;
  FGO calGraph;
  calGraph.verbose(true);
   
  /** add observations to 2d image key points */
  for (size_t i = 0; i < poses.size(); ++i) {
    for (size_t j = 0; j < points.size(); ++j) {
      PinholeCamera<FGO::CALIBRATION_MODEL> camera(poses[i], K);
      Point2 measurement = camera.project(points[j]);

      calGraph.add_keypoints_2d (i, j, measurement, 1.0);
    }
  }


  /** initial estimates */
  std::vector<Eigen::Matrix4d> var_poses;
  for (size_t i = 0; i < poses.size(); ++i) {
  var_poses.push_back( poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                            Point3(0.05, -0.10, 0.20))).matrix() );
  }
  var_poses[0] = poses[0].matrix();

  std::vector<Eigen::Vector3d> var_points;
  for (size_t j = 0; j < points.size(); ++j) {
  var_points.push_back( points[j] + Point3(-0.25, 0.20, 0.15) );
  }
  var_points[0] = points[0];



  Eigen::Matrix3d var_K = K.K();
  double new_focal = 567;
  var_K(0, 0) = new_focal;
  var_K(1, 1) = new_focal;

  Eigen::VectorXd var_D = FGO::LENSDISTORT_MODEL(1.234).vector();


  std::cout << " -------------- data generated ------------------ " << "\n";


  /** For priors, make sure the values assigned to the first pose, and the first landmark are consistent with the measurement */
  calGraph.optimize_from(var_K, var_D, var_poses, var_points);


  std::cout << " -------------- variables optimized ------------------ " << "\n";

  return 0;
}



