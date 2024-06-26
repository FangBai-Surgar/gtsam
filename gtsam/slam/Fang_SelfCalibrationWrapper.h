

#pragma once



/**
 * @file gtsam/slam/Fang_SelfCalibrationWrapper.h.
 * @brief A wrapper class to use self-calibration module in GTSAM
 * @date June 20, 2024
 * @author Fang Bai
 */


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


class GTSAM_SelfCalibration {

  public:

    /** GTSAM */
    typedef gtsam::Cal3_f camera_model;
    typedef gtsam::LensDistortDivisionModel lens_dist_model;
    
    typedef gtsam::UncalibratedProjectionFactor<camera_model> factor_cal_proj;
    typedef gtsam::UncalibratedProjectionDistortedImageFactor<camera_model, lens_dist_model> factor_cal_proj_dist;


    typedef gtsam::NonlinearFactorGraph FactorGraph;
    typedef gtsam::LevenbergMarquardtOptimizer optimizer;

    /**
     * If landmarks are not constrained with a prior, GTSAM throw an error when using GaussNewton and Dogleg.
     */
    /**
     * Thrown when a linear system is ill-posed.  The most common cause for this
        error is having underconstrained variables.  Mathematically, the system is
        underdetermined.  See the GTSAM Doxygen documentation at
        http://borg.cc.gatech.edu/ on gtsam::IndeterminantLinearSystemException for
        more information.
        Aborted (core dumped)
     */
    // typedef gtsam::DoglegOptimizer optimizer;
    // typedef gtsam::GaussNewtonOptimizer optimizer;

  protected:

    FactorGraph graph;

  public:

    GTSAM_SelfCalibration() : graph() {}

    void add_keypoints_2d (size_t ith_pose, size_t jth_landmark, gtsam::Point2 & img_kpts, double sigma = 1.0) {
      /** image noise */
      auto img_noise = gtsam::noiseModel::Isotropic::Sigma(2, sigma);
      /** add factor and its measurement */
      graph.emplace_shared<factor_cal_proj_dist>(
          img_kpts,
          img_noise,
          gtsam::Symbol('x', ith_pose),
          gtsam::Symbol('l', jth_landmark),
          gtsam::Symbol('K', 0),
          gtsam::Symbol('D', 0)
          );
      // graph.emplace_shared<factor_cal_proj>(
      //     img_kpts,
      //     img_noise,
      //     gtsam::Symbol('x', ith_pose),
      //     gtsam::Symbol('l', jth_landmark),
      //     gtsam::Symbol('K', 0)
      //     );
    }
    
  
    /** pose prior */
    void addPosePrior (size_t ith_pose = 0, gtsam::Pose3 pose = gtsam::traits<Pose3>::Identity(), double sigma_rot = 0.001 /*rad on roll, pitch, yaw*/, double sigma_tran = 0.001 /*m on x, y, z*/) {
      auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas( (Vector(6) << gtsam::Vector3::Constant(sigma_rot), gtsam::Vector3::Constant(sigma_tran)).finished());
      graph.addPrior(gtsam::Symbol('x', ith_pose), pose, poseNoise);
    }

    /** calibration prior. Don't see a reason to use this */
    void addCalibrationPrior (camera_model & K){
        auto calNoise = noiseModel::Diagonal::Sigmas((Vector(1) << 500).finished());
        graph.addPrior(Symbol('K', 0), K, calNoise);
    }

    void addLandmarkPrior (size_t jth_landmark, gtsam::Point3 & landmark_3d, double sigma = 0.1) {
      auto pointNoise = noiseModel::Isotropic::Sigma(3, sigma);
      graph.addPrior(Symbol('l', jth_landmark), landmark_3d, pointNoise);
    }

    /** the parameters of the input will be optimiszed */
    void optimize_from (camera_model & K, lens_dist_model & dist, std::vector<gtsam::Pose3> & poses, std::vector<gtsam::Point3> & landmarks_3d) {
    // void optimize_from (camera_model & K, std::vector<gtsam::Pose3> & poses, std::vector<gtsam::Point3> & landmarks_3d) {

      gtsam::Values initialEstimate;

      initialEstimate.insert(gtsam::Symbol('K', 0), K);

      initialEstimate.insert(gtsam::Symbol('D', 0), dist);
      
      for (size_t i = 0; i < poses.size(); ++i) {
        initialEstimate.insert(gtsam::Symbol('x', i), poses[i]);
      }
      for (size_t j = 0; j < landmarks_3d.size(); ++j) {
        initialEstimate.insert(gtsam::Symbol('l', j), landmarks_3d[j]);
      }


      /* print initial values */
      initialEstimate.print("Initial values\n");
      

      /* Optimize the graph and print results */
      gtsam::Values result = optimizer(graph, initialEstimate).optimize();


      result.print("Final results:\n");
      
      std::cout << "initial error=" <<graph.error(initialEstimate)<< std::endl;
      std::cout << "final error=" <<graph.error(result)<< std::endl;



      /** obtain the optimised value for each variable */
      gtsam::Matrix3 opt_K = result.at(gtsam::Symbol('K', 0)).cast<camera_model>().K();
      gtsam::Vector1 opt_dist = result.at(gtsam::Symbol('D', 0)).cast<lens_dist_model>().vector();
      gtsam::Matrix4 opt_pose1 = result.at(gtsam::Symbol('x', 0)).cast<gtsam::Pose3>().matrix();
      gtsam::Matrix4 opt_pose2 = result.at(gtsam::Symbol('x', 1)).cast<gtsam::Pose3>().matrix();


      std::cout << "opt_K = \n" << opt_K << "\n";
      std::cout << "opt_dist = \n" << opt_dist << "\n";
      std::cout << "opt_pose1 = \n" << opt_pose1 << "\n";
      std::cout << "opt_pose2 = \n" << opt_pose2 << "\n";


    }

};

}
