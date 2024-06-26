

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

  GTSAM_SelfCalibration calGraph;


  for (size_t i = 0; i < poses.size(); ++i) {
    for (size_t j = 0; j < points.size(); ++j) {
      PinholeCamera<Cal3_f> camera(poses[i], K);
      Point2 measurement = camera.project(points[j]);

      calGraph.add_keypoints_2d (i, j, measurement, 1.0);
    }
  }


  calGraph.addPosePrior(0, poses[0], 0.1, 0.3);
  calGraph.addCalibrationPrior(K);
  calGraph.addLandmarkPrior (0, points[0], 0.1);


  /** initial estimates */
  auto var_K = GTSAM_SelfCalibration::camera_model(500.0, 500.0, 0.0, 50.0, 50.0);


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

  auto var_D = GTSAM_SelfCalibration::lens_dist_model(0.1);
  calGraph.optimize_from(var_K, var_D, var_poses, var_points);

  return 0;
}








// int main(int argc, char* argv[]) {
//   // Create the set of ground-truth
//   vector<Point3> points = createPoints();
//   vector<Pose3> poses = createPoses();

//   size_t used_poses = 8;


//   // Create the factor graph
//   NonlinearFactorGraph graph;

//   // Add a prior on pose x1.
//   auto poseNoise = noiseModel::Diagonal::Sigmas(
//       (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
//           .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
//   graph.addPrior(Symbol('x', 0), poses[0], poseNoise); // or to gtsam::Pose3::identity()

//   // Simulated measurements from each camera pose, adding them to the factor
//   // graph
//   Cal3_f K(50.0, 50.0, 0.0, 50.0, 50.0);
//   auto measurementNoise =
//       noiseModel::Isotropic::Sigma(2, 1.0);
//   for (size_t i = 0; i < used_poses; ++i) {
//     for (size_t j = 0; j < points.size(); ++j) {
//       PinholeCamera<Cal3_f> camera(poses[i], K);
//       Point2 measurement = camera.project(points[j]);
//       // The only real difference with the Visual SLAM example is that here we
//       // use a different factor type, that also calculates the Jacobian with
//       // respect to calibration
//       graph.emplace_shared<UncalibratedProjectionFactor<Cal3_f> >(
//           measurement, measurementNoise, Symbol('x', i), Symbol('l', j),
//           Symbol('K', 0));
//     }
//   }

//   // Add a prior on the position of the first landmark.
//   auto pointNoise =
//       noiseModel::Isotropic::Sigma(3, 0.1);
//   graph.addPrior(Symbol('l', 0), points[0],
//                  pointNoise);  // add directly to graph

//   // Add a prior on the calibration.
// //   auto calNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 0.1, 100, 100).finished());
//   auto calNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 500).finished());
//   graph.addPrior(Symbol('K', 0), K, calNoise);

//   // Create the initial estimate to the solution
//   // now including an estimate on the camera calibration parameters
//   Values initialEstimate;
//   initialEstimate.insert(Symbol('K', 0), Cal3_f(500.0, 500.0, 0.0, 50.0, 50.0));
//   for (size_t i = 0; i < used_poses; ++i)
//     initialEstimate.insert(
//         Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25),
//                                                Point3(0.05, -0.10, 0.20))));
//   for (size_t j = 0; j < points.size(); ++j)
//     initialEstimate.insert<Point3>(Symbol('l', j),
//                                    points[j] + Point3(-0.25, 0.20, 0.15));


//   /* print initial values */
//   initialEstimate.print("Initial values\n");

//   /* Optimize the graph and print results */
//   Values result = DoglegOptimizer(graph, initialEstimate).optimize();
//   result.print("Final results:\n"); 


//   std::cout << "initial error=" <<graph.error(initialEstimate)<< std::endl;
//   std::cout << "final error=" <<graph.error(result)<< std::endl;



//   /** obtain the optimised value for each variable */
//   Matrix3 opt_K = result.at(Symbol('K', 0)).cast<Cal3_f>().K();
//   Matrix4 opt_pose1 = result.at(Symbol('x', 0)).cast<Pose3>().matrix();
//   Matrix4 opt_pose2 = result.at(Symbol('x', 1)).cast<Pose3>().matrix();

//   cout << "opt_K = \n" << opt_K << "\n";
//   cout << "opt_pose1 = \n" << opt_pose1 << "\n";
//   cout << "opt_pose2 = \n" << opt_pose2 << "\n";


//   std::cout << "BOOST_LIB_VERSION = " << BOOST_LIB_VERSION << '\n';


//   return 0;
// }


