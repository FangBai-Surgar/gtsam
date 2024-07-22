
/**
 * @file gtsam/slam/Fang_SelfCalibrationWrapper.h.
 * @brief A wrapper template class to use self-calibration module in GTSAM
 * @date June 20, 2024
 * @author Fang Bai
 */


#pragma once


// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/slam/Fang_SelfCalibrationTypes.h>  
#include <gtsam/slam/Fang_SelfCalibrationFactors.h>  

// Standard headers
#include <vector>











namespace gtsam {



template < typename CALIBRATION,  template<typename> class DIST_CALIB_PROJ_FACTOR >
class SelfCalibrationWrapper {

  public:

    typedef CALIBRATION CALIBRATION_MODEL;

    typedef DIST_CALIB_PROJ_FACTOR<CALIBRATION> SelfCalibrationFactor;

    typedef gtsam::NonlinearFactorGraph FactorGraph;
    typedef gtsam::DoglegOptimizer optimizer;


  protected:

    FactorGraph graph;

    bool verbose_;

  public:

    SelfCalibrationWrapper() : graph(), verbose_(false) {}

    void add_keypoints_2d (int ith_pose, int jth_landmark, const gtsam::Point2 & img_kpts, double sigma = 1.0) {
      /** image noise */
      auto img_noise = gtsam::noiseModel::Isotropic::Sigma(2, sigma);
      /** add factor and its measurement */
      graph.emplace_shared<SelfCalibrationFactor>(
          img_kpts,
          img_noise,
          gtsam::Symbol('x', ith_pose),
          gtsam::Symbol('l', jth_landmark),
          gtsam::Symbol('K', 0)
          );
    }
  
    /** @brief A unified interface function for SURGAR 
     * @brief After optimization, this function updates the values passed through the function arguments.
    */


    template <typename Landmark3DArrayType>
    void optimize_from_3view (double & focal, double & kappa, double & u0, double & v0,
                              Eigen::Matrix4d & pose1, Eigen::Matrix4d & pose2, Eigen::Matrix4d & pose3, 
                              Landmark3DArrayType & landmarks_3d) {

      std::vector<Eigen::Matrix4d> poses;

      poses.push_back(pose1); poses.push_back(pose2); poses.push_back(pose3);

      this->optimize_from (focal, kappa, u0, v0, poses, landmarks_3d);

      pose1 = poses[0]; pose2 = poses[1]; pose3 = poses[2];

    }



    template <typename Landmark3DArrayType>
    void optimize_from_2view (double & focal, double & kappa, double & u0, double & v0,
                              Eigen::Matrix4d & pose1, Eigen::Matrix4d & pose2, 
                              Landmark3DArrayType & landmarks_3d) {

      std::vector<Eigen::Matrix4d> poses;

      poses.push_back(pose1); poses.push_back(pose2);

      this->optimize_from (focal, kappa, u0, v0, poses, landmarks_3d);

      pose1 = poses[0]; pose2 = poses[1];

    }



    void optimize_from (double & focal, double & kappa, double & u0, double & v0,
                        std::vector<Eigen::Matrix4d> & poses, std::vector<Eigen::Vector3d> & landmarks_3d) {

      Eigen::Matrix<double, Eigen::Dynamic, 3> mat_landmarks_3d;
      mat_landmarks_3d.resize(landmarks_3d.size(), 3);

      for (size_t j = 0; j < landmarks_3d.size(); ++j) {
          auto & lmk = landmarks_3d[j];
          mat_landmarks_3d.row(j) <<  lmk(0), lmk(1), lmk(2);
      }

      this->optimize_from (focal, kappa, u0, v0, poses, mat_landmarks_3d);

      for (size_t j = 0; j < landmarks_3d.size(); ++j) {
          const auto & lmk = mat_landmarks_3d.row(j);
          landmarks_3d[j] = Eigen::Vector3d(lmk(0), lmk(1), lmk(2));
      }
    }


    void optimize_from (double & focal, double & kappa, double & u0, double & v0,
                        std::vector<Eigen::Matrix4d> & poses,
                        Eigen::Matrix<double, Eigen::Dynamic, 3> & landmarks_3d) {

      this->addPosePrior(0, gtsam::Pose3(poses[0]), 0.001, 0.003);
      const auto & lmk0 = landmarks_3d.row(0);
      this->addLandmarkPrior (0, gtsam::Point3(lmk0(0), lmk0(1), lmk0(2)), 0.1);

      gtsam::Values initialEstimate;        

      initialEstimate.insert(gtsam::Symbol('K', 0), CALIBRATION(focal, kappa, u0, v0));

      for (size_t i = 0; i < poses.size(); ++i) {
        initialEstimate.insert(gtsam::Symbol('x', i), gtsam::Pose3(poses[i]));
      }
      for (int j = 0; j < landmarks_3d.rows(); ++j) {
        const auto & lmk = landmarks_3d.row(j);
        initialEstimate.insert(gtsam::Symbol('l', j), gtsam::Point3(lmk(0), lmk(1), lmk(2)));
      }

      if (0&&verbose_) {
        graph.print("\nFactor Graph:\n");
        initialEstimate.print("\nInitial Values:\n");
      }

      /* Optimize the graph and print results */
      gtsam::Values result = optimizer(graph, initialEstimate).optimize();

      if (0&&verbose_) {
        result.print("\nFinal Result:\n");
      }

      /** obtain the optimised value for each variable */
      auto calib = result.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>();
      focal = calib.focal(); kappa = calib.kappa(); u0 = calib.u0(); v0 = calib.v0();

      auto index = [](gtsam::Key key) { return Symbol(key).index(); }; 
      for (const auto &pair : result.extract<gtsam::Pose3>()) {
        poses[index(pair.first)] = pair.second.matrix();
      }
      for (const auto &pair : result.extract<gtsam::Point3>()) {
        const gtsam::Point3 & pt = pair.second;
        landmarks_3d.row(index(pair.first)) << pt(0), pt(1), pt(2);
      }



      if (verbose_) {
        std::cout << " -------------- Calibration K ------------------ " << "\n";
        std::cout << "Init: \t" << initialEstimate.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>() <<"\n";
        std::cout << "Opt:  \t" << result.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>() << "\n";
        std::cout << " -------------- Cost Function ------------------ " << "\n";
        std::cout << "Init: \t" <<graph.error(initialEstimate)<< "\n";
        std::cout << "Opt:  \t" <<graph.error(result)<< "\n"; 
      }

    }





    void verbose (bool val = true) { verbose_ = val; }


protected:

    /** pose prior: to remove the global gauge/transformation ambiguitiey */
    void addPosePrior (size_t ith_pose = 0, gtsam::Pose3 pose = gtsam::traits<Pose3>::Identity(), double sigma_rot = 0.001 /*rad on roll, pitch, yaw*/, double sigma_tran = 0.001 /*m on x, y, z*/) {
      auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas( (Vector(6) << gtsam::Vector3::Constant(sigma_rot), gtsam::Vector3::Constant(sigma_tran)).finished());
      graph.addPrior(gtsam::Symbol('x', ith_pose), pose, poseNoise);
    }

    /** landmark prior: to remove the global scale ambiguitiey */
    void addLandmarkPrior (size_t jth_landmark, gtsam::Point3 landmark_3d, double sigma = 0.1) {
      auto pointNoise = noiseModel::Isotropic::Sigma(3, sigma);
      graph.addPrior(Symbol('l', jth_landmark), landmark_3d, pointNoise);
    }

    /** calibration prior. Don't see a reason to use this */
    void addCalibrationPrior (CALIBRATION K){
        auto calNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 500).finished());
        graph.addPrior(Symbol('K', 0), K, calNoise);
    }


};



} // namespace




namespace gtsam {

  typedef gtsam::SelfCalibrationWrapper < gtsam::Cal_fd, gtsam::UncalibratedProjectionFactor> SelfCalibrationForward;

} // namespace

















// namespace gtsam {



// template < typename CALIBRATION, typename LENSDISTORT,  template<typename, typename> class DIST_CALIB_PROJ_FACTOR >
// class SelfCalibrationWrapperBackup {

//   public:

//     typedef CALIBRATION CALIBRATION_MODEL;
//     typedef LENSDISTORT LENSDISTORT_MODEL;

//     typedef DIST_CALIB_PROJ_FACTOR<CALIBRATION, LENSDISTORT> SelfCalibrationFactor;

//     typedef gtsam::NonlinearFactorGraph FactorGraph;
//     typedef gtsam::DoglegOptimizer optimizer;


//   protected:

//     FactorGraph graph;

//     bool verbose_;

//   public:

//     SelfCalibrationWrapperBackup() : graph(), verbose_(false) {}

//     void add_keypoints_2d (size_t ith_pose, size_t jth_landmark, gtsam::Point2 & img_kpts, double sigma = 1.0) {
//       /** image noise */
//       auto img_noise = gtsam::noiseModel::Isotropic::Sigma(2, sigma);
//       /** add factor and its measurement */
//       graph.emplace_shared<SelfCalibrationFactor>(
//           img_kpts,
//           img_noise,
//           gtsam::Symbol('x', ith_pose),
//           gtsam::Symbol('l', jth_landmark),
//           gtsam::Symbol('K', 0),
//           gtsam::Symbol('D', 0)
//           );
//     }
  
//     /** @brief A unified interface function for SURGAR 
//      * @brief After optimization, this function updates the values passed through the function arguments.
//     */
//     void optimize_from (Eigen::Matrix3d & K, Eigen::VectorXd & dist, std::vector<Eigen::Matrix4d> & poses, std::vector<Eigen::Vector3d> & landmarks_3d) {

//       this->addPosePrior(0, gtsam::Pose3(poses[0]), 0.001, 0.003);
//       this->addLandmarkPrior (0, landmarks_3d[0], 0.1);

//       gtsam::Values initialEstimate;

//       initialEstimate.insert(gtsam::Symbol('K', 0), CALIBRATION(K));
//       initialEstimate.insert(gtsam::Symbol('D', 0), LENSDISTORT(dist));
//       for (size_t i = 0; i < poses.size(); ++i) {
//         initialEstimate.insert(gtsam::Symbol('x', i), gtsam::Pose3(poses[i]));
//       }
//       for (size_t j = 0; j < landmarks_3d.size(); ++j) {
//         initialEstimate.insert(gtsam::Symbol('l', j), landmarks_3d[j]);
//       }

//       /* Optimize the graph and print results */
//       gtsam::Values result = optimizer(graph, initialEstimate).optimize();
      
//       /** obtain the optimised value for each variable */
//       K = result.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>().K();

//       dist = result.at(gtsam::Symbol('D', 0)).cast<LENSDISTORT>().vector();

//       auto index = [](gtsam::Key key) { return Symbol(key).index(); }; 
//       for (const auto &pair : result.extract<gtsam::Pose3>()) {
//         poses[index(pair.first)] = pair.second.matrix();
//       }
//       for (const auto &pair : result.extract<gtsam::Point3>()) {
//         landmarks_3d[index(pair.first)] = pair.second;
//       }

//       if (verbose_) {
//         std::cout << " -------------- Calibration K ------------------ " << "\n";
//         std::cout << "Init: \t" << initialEstimate.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>() <<"\n";
//         std::cout << "Opt:  \t" << result.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>() << "\n";
//         std::cout << " -------------- Lens Distortion ------------------ " << "\n";
//         std::cout << "Init: \t" << initialEstimate.at(gtsam::Symbol('D', 0)).cast<LENSDISTORT>() <<"\n";
//         std::cout << "Opt:  \t" << result.at(gtsam::Symbol('D', 0)).cast<LENSDISTORT>() << "\n";
//         std::cout << " -------------- Cost Function ------------------ " << "\n";
//         std::cout << "Init: \t" <<graph.error(initialEstimate)<< "\n";
//         std::cout << "Opt:  \t" <<graph.error(result)<< "\n"; 
//       }

//     }

//     void verbose (bool val = true) { verbose_ = val; }


// protected:

//     /** pose prior: to remove the global gauge/transformation ambiguitiey */
//     void addPosePrior (size_t ith_pose = 0, gtsam::Pose3 pose = gtsam::traits<Pose3>::Identity(), double sigma_rot = 0.001 /*rad on roll, pitch, yaw*/, double sigma_tran = 0.001 /*m on x, y, z*/) {
//       auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas( (Vector(6) << gtsam::Vector3::Constant(sigma_rot), gtsam::Vector3::Constant(sigma_tran)).finished());
//       graph.addPrior(gtsam::Symbol('x', ith_pose), pose, poseNoise);
//     }

//     /** landmark prior: to remove the global scale ambiguitiey */
//     void addLandmarkPrior (size_t jth_landmark, gtsam::Point3 & landmark_3d, double sigma = 0.1) {
//       auto pointNoise = noiseModel::Isotropic::Sigma(3, sigma);
//       graph.addPrior(Symbol('l', jth_landmark), landmark_3d, pointNoise);
//     }

//     /** calibration prior. Don't see a reason to use this */
//     void addCalibrationPrior (CALIBRATION & K){
//         auto calNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 500).finished());
//         graph.addPrior(Symbol('K', 0), K, calNoise);
//     }


// };



// } // namespace




// namespace gtsam {

//   typedef gtsam::SelfCalibrationWrapperBackup < gtsam::Cal3_f, gtsam::LensDistortFieldOfViewModel, gtsam::UncalibratedProjectionDistortedImageFactor> SelfCalibrationReverse;

//   typedef gtsam::SelfCalibrationWrapperBackup < gtsam::Cal3_f, gtsam::LensDistortRadialFirstOrder, gtsam::DistortedUncalibratedProjectionFactor> SelfCalibrationForward;

// } // namespace



