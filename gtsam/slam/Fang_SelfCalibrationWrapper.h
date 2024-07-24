
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





/** GTSAM pose.transformTo
 * 
      const Matrix3 Rt = R_.transpose();
      const Point3 q(Rt*(point - t_));
 * 
 */






namespace gtsam {



template < typename CALIBRATION >
class SelfCalibrationWrapper {

  public:

    typedef CALIBRATION CALIBRATION_MODEL;

    typedef UncalibratedProjectionFactor<CALIBRATION> SelfCalibrationFactor;
    
    typedef FixedPoseUncalibratedProjectionFactor<CALIBRATION> FixedPoseSelfCalibrationFactor;

    typedef gtsam::NonlinearFactorGraph FactorGraph;

    // typedef gtsam::DoglegOptimizer optimizer;

    typedef gtsam::GaussNewtonOptimizer optimizer;

    // typedef gtsam::LevenbergMarquardtOptimizer optimizer; 

  protected:

    FactorGraph graph;

    bool verbose_;

  public:

    SelfCalibrationWrapper() : graph(), verbose_(false) {}


    void verbose (bool val = true) { verbose_ = val; }


    /** pose prior: to remove the global gauge/transformation ambiguitiey */
    void add_pose_prior (size_t ith_pose = 0, const Eigen::Matrix4d& pose = Eigen::Matrix4d::Identity(), 
                         double sigma_rot = 0.001 /*rad on roll, pitch, yaw*/, double sigma_tran = 0.003 /*m on x, y, z*/) {
      auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas( (Vector(6) << gtsam::Vector3::Constant(sigma_rot), gtsam::Vector3::Constant(sigma_tran)).finished());
      graph.addPrior(gtsam::Symbol('x', ith_pose), gtsam::Pose3(pose), poseNoise);
    }


    void add_keypoints_2d (int ith_pose, int jth_landmark, const Eigen::Vector2d& img_kpts, double sigma = 1.0) {
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

    void add_keypoints_2d_from_fixed_pose (const Eigen::Matrix4d& pose, int jth_landmark, const Eigen::Vector2d & img_kpts, double sigma = 1.0) {
      auto img_noise = gtsam::noiseModel::Isotropic::Sigma(2, sigma);
      /** add factor and its measurement */
      graph.emplace_shared<FixedPoseSelfCalibrationFactor>(
          img_kpts,
          gtsam::Pose3(pose),
          img_noise,
          gtsam::Symbol('l', jth_landmark),
          gtsam::Symbol('K', 0)
          );
    }

  
    /** @brief A unified interface function for SURGAR 
     * @brief After optimization, this function updates the values passed through the function arguments.
    */


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


      // fix the scale. There're several tricks for this.
      // This is difficult because scale is sensitive to initialization.
  
    if(0)
    {
      size_t ith_pose = 0, jth_landmark = 0;
      double sigma = 0.1;
      Pose3 pose(poses[ith_pose]);
      Point3 pt(landmarks_3d.row(jth_landmark)[0], landmarks_3d.row(jth_landmark)[1], landmarks_3d.row(jth_landmark)[2]);
      double depthij = pose.transformTo(pt)[2];
      this->addLandmarkDepthPrior (ith_pose, jth_landmark, depthij, sigma*depthij);
      printf("Fix Depth of Landmark %d from Pose %d to %f \n", static_cast<int>(ith_pose), static_cast<int>(jth_landmark), depthij);
    }
    if(0)
    {
      size_t ith_pose = 0, jth_landmark = landmarks_3d.rows()-1;
      double sigma = 0.1;
      Pose3 pose(poses[ith_pose]);
      Point3 pt(landmarks_3d.row(jth_landmark)[0], landmarks_3d.row(jth_landmark)[1], landmarks_3d.row(jth_landmark)[2]);
      double depthij = pose.transformTo(pt)[2];
      this->addLandmarkDepthPrior (ith_pose, jth_landmark, depthij, sigma*depthij);
      printf("Fix Depth of Landmark %d from Pose %d to %f \n", static_cast<int>(ith_pose), static_cast<int>(jth_landmark), depthij);
    }
    if(0)
    {
      size_t ith_pose = 0;
      double sigma = 0.5;
      for (size_t jth_landmark = 0; jth_landmark < static_cast<size_t>(landmarks_3d.rows()); jth_landmark++)
      {
          Pose3 pose(poses[ith_pose]);
          Point3 pt(landmarks_3d.row(jth_landmark)[0], landmarks_3d.row(jth_landmark)[1], landmarks_3d.row(jth_landmark)[2]);
          double depthij = pose.transformTo(pt)[2];
          this->addLandmarkDepthPrior (ith_pose, jth_landmark, depthij, sigma*depthij);
          printf("Fix Depth of Landmark %d from Pose %d to %f \n", static_cast<int>(ith_pose), static_cast<int>(jth_landmark), depthij);
      }
    }
    

      if (1)
      {
        const auto & lmk0 = landmarks_3d.row(0);
        this->addLandmarkPrior (0, gtsam::Point3(lmk0(0), lmk0(1), lmk0(2)), 0.1 );
        printf("Fix Landmark %d \n", 0);
      }
      if (0)
      {
        size_t lmk_1st = 0, lmk_2nd = landmarks_3d.rows() - 1;

        double d = std::sqrt( (landmarks_3d.row(lmk_1st) - landmarks_3d.row(lmk_2nd)).squaredNorm() );
        this->addRelativePointDistancePrior (lmk_1st, lmk_2nd, d, 0.1*d);
        printf("Fix reltive distance between Landmarks (%d, %d): %f \n", static_cast<int>(lmk_1st), static_cast<int>(lmk_2nd), d);

        // const auto & lmk1 = landmarks_3d.row(lmk_1st);
        // double dn1 = gtsam::Point3(lmk1(0), lmk1(1), lmk1(2)).norm();
        // this->addLandmarkNormPrior (lmk_1st, dn1, 0.3*dn1);  
        // printf("Fix Landmark (%d) norm: %f \n", static_cast<int>(lmk_1st), dn1);

        // const auto & lmk2 = landmarks_3d.row(lmk_2nd);
        // double dn2 = gtsam::Point3(lmk2(0), lmk2(1), lmk2(2)).norm();
        // this->addLandmarkNormPrior (lmk_2nd, dn2, 1.0*dn2);
        // printf("Fix Landmark (%d) norm: %f \n", static_cast<int>(lmk_2nd), dn2);
      }


      // This DOES NOT work yet.
      // fix the relative translation distance between 1st and 2nd pose

      // double dt = ( poses[0].col(3) - poses[1].col(3) ).norm();
      // this->addRRelativePoseTranslationDistancePrior(0, 1, dt, 0.5*dt);
      // printf("Fix relative translation between poses %d and %d : to %f \n", 0, 1, dt);




      gtsam::Values initialEstimate;

      if (0) {
        graph.print("\nFactor Graph:\n");
        initialEstimate.print("\nInitial Values:\n");
      }


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







protected:


    /** landmark prior: to remove the global scale ambiguitiey */
    void addLandmarkPrior (size_t jth_landmark, gtsam::Point3 landmark_3d, double sigma = 0.001) {
      auto pointNoise = noiseModel::Isotropic::Sigma(3, sigma);
      graph.addPrior(Symbol('l', jth_landmark), landmark_3d, pointNoise);
    }

    /** calibration prior. Don't see a reason to use this */
    void addCalibrationPrior (CALIBRATION K){
        auto calNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 500).finished());
        graph.addPrior(Symbol('K', 0), K, calNoise);
    }


    /** landmark prior: to remove the global scale ambiguitiey */
    void addLandmarkDepthPrior (size_t ith_pose, size_t jth_landmark, double depth, double sigma = 1.0) {
      if (depth <=0) {
        std::cerr << "Depth of landmark " << jth_landmark << "is not positive! Not added as a prior" << std::endl;
        return;
      }
      auto depth_noise = noiseModel::Isotropic::Sigma(1, sigma);
      graph.emplace_shared<Landmark3DDepthFactor>(
          depth,
          depth_noise,
          gtsam::Symbol('x', ith_pose),
          gtsam::Symbol('l', jth_landmark)
          );
    }



    /** pose relative translation norm prior */
    void addRRelativePoseTranslationDistancePrior (size_t pth_pose, size_t qth_pose, double distance, double sigma = 0.001) {
      auto dist_noise = noiseModel::Isotropic::Sigma(1, sigma);
      graph.emplace_shared<RelativePoseTranslationDistanceFactor>(
          distance,
          dist_noise,
          gtsam::Symbol('x', pth_pose),
          gtsam::Symbol('x', qth_pose)
          );
    }



    /** landmark norm prior: to remove the global scale ambiguitiey */
    void addLandmarkNormPrior (size_t jth_landmark, double distance, double sigma = 0.001) {
      auto dist_noise = noiseModel::Isotropic::Sigma(1, sigma);
      graph.emplace_shared<Point3NormFactor>(
          distance,
          dist_noise,
          gtsam::Symbol('l', jth_landmark)
          );
      
    }

    void addRelativePointDistancePrior (size_t pth_landmark, size_t qth_landmark, double distance, double sigma = 0.001) {
      auto dist_noise = noiseModel::Isotropic::Sigma(1, sigma);
      graph.emplace_shared<RelativePointDistanceFactor>(
          distance,
          dist_noise,
          gtsam::Symbol('l', pth_landmark),
          gtsam::Symbol('l', qth_landmark)
          );
    }


};



} // namespace




namespace gtsam {

  typedef gtsam::SelfCalibrationWrapper < gtsam::Cal_fd > SelfCalibrationForward;

} // namespace












#ifdef SURGAR_SELFCALIBRATION_TEST_FANG




namespace gtsam {



template < typename CALIBRATION, typename LENSDISTORT,  template<typename, typename> class DIST_CALIB_PROJ_FACTOR >
class SelfCalibrationWrapperBackup {

  public:

    typedef CALIBRATION CALIBRATION_MODEL;
    typedef LENSDISTORT LENSDISTORT_MODEL;

    typedef DIST_CALIB_PROJ_FACTOR<CALIBRATION, LENSDISTORT> SelfCalibrationFactor;

    typedef gtsam::NonlinearFactorGraph FactorGraph;
    typedef gtsam::DoglegOptimizer optimizer;


  protected:

    FactorGraph graph;

    bool verbose_;

  public:

    SelfCalibrationWrapperBackup() : graph(), verbose_(false) {}

    void add_keypoints_2d (size_t ith_pose, size_t jth_landmark, gtsam::Point2 & img_kpts, double sigma = 1.0) {
      /** image noise */
      auto img_noise = gtsam::noiseModel::Isotropic::Sigma(2, sigma);
      /** add factor and its measurement */
      graph.emplace_shared<SelfCalibrationFactor>(
          img_kpts,
          img_noise,
          gtsam::Symbol('x', ith_pose),
          gtsam::Symbol('l', jth_landmark),
          gtsam::Symbol('K', 0),
          gtsam::Symbol('D', 0)
          );
    }
  
    /** @brief A unified interface function for SURGAR 
     * @brief After optimization, this function updates the values passed through the function arguments.
    */
    void optimize_from (Eigen::Matrix3d & K, Eigen::VectorXd & dist, std::vector<Eigen::Matrix4d> & poses, std::vector<Eigen::Vector3d> & landmarks_3d) {

      this->addPosePrior(0, gtsam::Pose3(poses[0]), 0.001, 0.003);
      this->addLandmarkPrior (0, landmarks_3d[0], 0.1);

      gtsam::Values initialEstimate;

      initialEstimate.insert(gtsam::Symbol('K', 0), CALIBRATION(K));
      initialEstimate.insert(gtsam::Symbol('D', 0), LENSDISTORT(dist));
      for (size_t i = 0; i < poses.size(); ++i) {
        initialEstimate.insert(gtsam::Symbol('x', i), gtsam::Pose3(poses[i]));
      }
      for (size_t j = 0; j < landmarks_3d.size(); ++j) {
        initialEstimate.insert(gtsam::Symbol('l', j), landmarks_3d[j]);
      }

      /* Optimize the graph and print results */
      gtsam::Values result = optimizer(graph, initialEstimate).optimize();
      
      /** obtain the optimised value for each variable */
      K = result.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>().K();

      dist = result.at(gtsam::Symbol('D', 0)).cast<LENSDISTORT>().vector();

      auto index = [](gtsam::Key key) { return Symbol(key).index(); }; 
      for (const auto &pair : result.extract<gtsam::Pose3>()) {
        poses[index(pair.first)] = pair.second.matrix();
      }
      for (const auto &pair : result.extract<gtsam::Point3>()) {
        landmarks_3d[index(pair.first)] = pair.second;
      }

      if (verbose_) {
        std::cout << " -------------- Calibration K ------------------ " << "\n";
        std::cout << "Init: \t" << initialEstimate.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>() <<"\n";
        std::cout << "Opt:  \t" << result.at(gtsam::Symbol('K', 0)).cast<CALIBRATION>() << "\n";
        std::cout << " -------------- Lens Distortion ------------------ " << "\n";
        std::cout << "Init: \t" << initialEstimate.at(gtsam::Symbol('D', 0)).cast<LENSDISTORT>() <<"\n";
        std::cout << "Opt:  \t" << result.at(gtsam::Symbol('D', 0)).cast<LENSDISTORT>() << "\n";
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
    void addLandmarkPrior (size_t jth_landmark, gtsam::Point3 & landmark_3d, double sigma = 0.1) {
      auto pointNoise = noiseModel::Isotropic::Sigma(3, sigma);
      graph.addPrior(Symbol('l', jth_landmark), landmark_3d, pointNoise);
    }

    /** calibration prior. Don't see a reason to use this */
    void addCalibrationPrior (CALIBRATION & K){
        auto calNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 500).finished());
        graph.addPrior(Symbol('K', 0), K, calNoise);
    }


};



} // namespace




namespace gtsam {

  typedef gtsam::SelfCalibrationWrapperBackup < gtsam::Cal3_f, gtsam::LensDistortFieldOfViewModel, gtsam::UncalibratedProjectionDistortedImageFactor> SelfCalibrationReverse;

  typedef gtsam::SelfCalibrationWrapperBackup < gtsam::Cal3_f, gtsam::LensDistortRadialFirstOrder, gtsam::DistortedUncalibratedProjectionFactor> SelfCalibrationForward;

} // namespace



#endif