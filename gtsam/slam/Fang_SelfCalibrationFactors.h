
/**
 * @file gtsam/slam/Fang_SelfCalibrationFactors.h.h
 * @brief Implementation of self-calibration factors. 
 * At its core, these factors formulate a reprojection cost function by allowing adjustable calibration and distortion parameters.
 * @date June 10, 2024
 * @author Fang Bai
 */

#pragma once



#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/concepts.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/timing.h>

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif
#include <iostream>
#include <string>





/**
 * 
 * @brief a general SFM factor with an unknown calibration. 
 * code modified from factor GeneralSFMFactor2 in "gtsam/slam/GeneralSFMFactor.h", originally implemented by Kai Ni
 * 
 * minimise \sum_j || proj (K[R_j X + t_j]) - img_pts || over (K, R_j, t_j, X)
 * 
 * The projective factor where the calibration matrix is optimisable.
 * There is no need to modify anything of this class when implementing your customized calibration method.
 * 
 */

namespace gtsam {
/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * Compared to GeneralSFMFactor, it is a ternary-factor because the calibration is isolated from camera..
 */
template<class CALIBRATION>
class UncalibratedProjectionFactor: public NoiseModelFactorN<Pose3, Point3, CALIBRATION> {

  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)
  static const int DimK = FixedDimension<CALIBRATION>::value;

protected:

  Point2 measured_; ///< the 2D measurement

public:

  typedef UncalibratedProjectionFactor<CALIBRATION> This;
  typedef PinholeCamera<CALIBRATION> Camera;///< typedef for camera type
  typedef NoiseModelFactorN<Pose3, Point3, CALIBRATION> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation of the measurements
   * @param poseKey is the index of the camera
   * @param landmarkKey is the index of the landmark
   * @param calibKey is the index of the calibration
   */
  UncalibratedProjectionFactor(const Point2& measured, const SharedNoiseModel& model, Key poseKey, Key landmarkKey, Key calibKey) :
  Base(model, poseKey, landmarkKey, calibKey), measured_(measured) {}
  UncalibratedProjectionFactor():measured_(0.0,0.0) {} ///< default constructor

  ~UncalibratedProjectionFactor() override {} ///< destructor

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "UncalibratedProjectionFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }

  /** h(x)-z */
  Vector evaluateError(const Pose3& pose3, const Point3& point, const CALIBRATION &calib,
      OptionalMatrixType H1,
      OptionalMatrixType H2,
      OptionalMatrixType H3) const override
  {
    try {
      Camera camera(pose3,calib);
      return camera.project(point, H1, H2, H3) - measured_;
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(2, 6);
      if (H2) *H2 = Matrix::Zero(2, 3);
      if (H3) *H3 = Matrix::Zero(2, DimK);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2())
      << " behind Camera " << DefaultKeyFormatter(this->key1()) << std::endl;
    }
    return Z_2x1;
  }

  /** return the measured */
  inline const Point2 measured() const {
    return measured_;
  }

private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
#endif
};

template<class CALIBRATION>
struct traits<UncalibratedProjectionFactor<CALIBRATION> > : Testable<
    UncalibratedProjectionFactor<CALIBRATION> > {
};


} //namespace







/**
 * 
 * @brief a general SFM factor with an unknown calibration and distortion coefficients
 * 
 * minimise \sum_j || proj (K[R_j X + t_j]) - undistort_kappa (img_pts) || over (K, R_j, t_j, X, kappa)
 * 
 */


namespace gtsam {


template<class CALIBRATION, class LENSDISTORT>
class UncalibratedProjectionDistortedImageFactor : public NoiseModelFactorN<Pose3, Point3, CALIBRATION, LENSDISTORT> {

  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)
  GTSAM_CONCEPT_MANIFOLD_TYPE(LENSDISTORT)
  static const int DimK = FixedDimension<CALIBRATION>::value;
  static const int DimD = FixedDimension<LENSDISTORT>::value;


protected:

  Point2 measured_; ///< the 2D measurement


public:

  typedef UncalibratedProjectionDistortedImageFactor<CALIBRATION, LENSDISTORT> This;
  typedef PinholeCamera<CALIBRATION> Camera;///< typedef for camera type
  typedef NoiseModelFactorN<Pose3, Point3, CALIBRATION, LENSDISTORT> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation of the measurements
   * @param poseKey is the index of the camera
   * @param landmarkKey is the index of the landmark
   * @param calibKey is the index of the calibration
   * @param distKey is the index of the distortion
   */
  UncalibratedProjectionDistortedImageFactor(const Point2& measured, const SharedNoiseModel& model, Key poseKey, Key landmarkKey, Key calibKey, Key distKey) :
  Base(model, poseKey, landmarkKey, calibKey, distKey), measured_(measured) {}
  UncalibratedProjectionDistortedImageFactor():measured_(0.0,0.0) {} ///< default constructor

  ~UncalibratedProjectionDistortedImageFactor() override {} ///< destructor


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}



  void print(const std::string& s = "UncalibratedProjectionDistortedImageFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }
  

  /** proj (K[R_j X + t_j]) - distort (img_pts) - img_pts  */
  Vector evaluateError(const Pose3& pose3, const Point3& point, const CALIBRATION &calib, const LENSDISTORT & lens_dist,
      OptionalMatrixType H1,
      OptionalMatrixType H2,
      OptionalMatrixType H3,
      OptionalMatrixType H4) const override
  {
    try {
      Camera camera(pose3,calib);
      Point2 pxy (calib.px(), calib.px());
      return camera.project(point, H1, H2, H3) - (lens_dist.undistort(measured_ - pxy, H4) + pxy);
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(2, 6);
      if (H2) *H2 = Matrix::Zero(2, 3);
      if (H3) *H3 = Matrix::Zero(2, DimK);
      if (H4) *H4 = Matrix::Zero(2, DimD);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2())
      << " behind Camera " << DefaultKeyFormatter(this->key1()) << std::endl;
    }
    return Z_2x1;
  }
  

  inline const Point2 measured() const {
    return measured_;
  }


private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    // NoiseModelFactor4 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor4",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
#endif



};


template<class CALIBRATION, class LENSDISTORT>
struct traits< UncalibratedProjectionDistortedImageFactor<CALIBRATION, LENSDISTORT> > : Testable <
    UncalibratedProjectionDistortedImageFactor<CALIBRATION, LENSDISTORT> > {

};


} //namespace










/**
 * 
 * @brief a general SFM factor with an unknown calibration and distortion coefficients
 * 
 * minimise \sum_j || dist_kappa (proj (K[R_j X + t_j])) - img_pts || over (K, R_j, t_j, X, kappa)
 * 
 */


namespace gtsam {


template<class CALIBRATION, class LENSDISTORT>
class DistortedUncalibratedProjectionFactor : public NoiseModelFactorN<Pose3, Point3, CALIBRATION, LENSDISTORT> {

  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)
  GTSAM_CONCEPT_MANIFOLD_TYPE(LENSDISTORT)
  static const int DimK = FixedDimension<CALIBRATION>::value;
  static const int DimD = FixedDimension<LENSDISTORT>::value;


protected:

  Point2 measured_; ///< the 2D measurement


public:

  typedef DistortedUncalibratedProjectionFactor<CALIBRATION, LENSDISTORT> This;
  typedef PinholeCamera<CALIBRATION> Camera;///< typedef for camera type
  typedef NoiseModelFactorN<Pose3, Point3, CALIBRATION, LENSDISTORT> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation of the measurements
   * @param poseKey is the index of the camera
   * @param landmarkKey is the index of the landmark
   * @param calibKey is the index of the calibration
   * @param distKey is the index of the distortion
   */
  DistortedUncalibratedProjectionFactor(const Point2& measured, const SharedNoiseModel& model, Key poseKey, Key landmarkKey, Key calibKey, Key distKey) :
  Base(model, poseKey, landmarkKey, calibKey, distKey), measured_(measured) {}
  DistortedUncalibratedProjectionFactor():measured_(0.0,0.0) {} ///< default constructor

  ~DistortedUncalibratedProjectionFactor() override {} ///< destructor


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}



  void print(const std::string& s = "DistortedUncalibratedProjectionFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }
  

  /** proj (K[R_j X + t_j]) - distort (img_pts) - img_pts  */
  Vector evaluateError(const Pose3& pose3, const Point3& point, const CALIBRATION &calib, const LENSDISTORT & lens_dist,
      OptionalMatrixType H1,
      OptionalMatrixType H2,
      OptionalMatrixType H3,
      OptionalMatrixType H4) const override
  {
    try {
      Camera camera(pose3,calib);
      Point2 pxy (calib.px(), calib.px());
      Point2 proj = camera.project(point, H1, H2, H3);
      proj = lens_dist.distort(proj - pxy, H1, H2, H3, H4) + pxy;
      return proj - measured_;
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(2, 6);
      if (H2) *H2 = Matrix::Zero(2, 3);
      if (H3) *H3 = Matrix::Zero(2, DimK);
      if (H4) *H4 = Matrix::Zero(2, DimD);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2())
      << " behind Camera " << DefaultKeyFormatter(this->key1()) << std::endl;
    }
    return Z_2x1;
  }
  

  inline const Point2 measured() const {
    return measured_;
  }


private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    // NoiseModelFactor4 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor4",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
#endif


};


template<class CALIBRATION, class LENSDISTORT>
struct traits< DistortedUncalibratedProjectionFactor<CALIBRATION, LENSDISTORT> > : Testable <
    DistortedUncalibratedProjectionFactor<CALIBRATION, LENSDISTORT> > {

};


} //namespace



