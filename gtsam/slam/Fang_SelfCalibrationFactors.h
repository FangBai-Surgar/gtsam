
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









namespace gtsam {



/**
 * @brief A factor to constrain the depth of a 3D landmark
 */
class Landmark3DDepthFactor: public NoiseModelFactorN<Pose3, Point3> {

protected:

  double measured_;


public:

  typedef Landmark3DDepthFactor This;
  typedef NoiseModelFactorN<Pose3, Point3> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;


  Landmark3DDepthFactor(double measured, const SharedNoiseModel& model, Key posekKey, Key landmarkKey) :
    Base(model, posekKey, landmarkKey), measured_(measured) {}
  Landmark3DDepthFactor():measured_(0.0) {} ///< default constructor

  ~Landmark3DDepthFactor() override {} ///< destructor


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "Landmark3DDepthFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << (s.empty() ? s : s + " ") << measured_ << std::endl;
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && std::abs(this->measured_ -  e->measured_) <  tol;
  }

  double costFuncHelper (const Pose3& pose, const Point3& landmark, OptionalJacobian<1, 6> H1, OptionalJacobian<1, 3> H2) const {

    Eigen::Matrix<double, 3, 6> Hself;
    Eigen::Matrix<double, 3, 3> Hpoint;

    Point3 lmk = pose.transformTo(landmark, Hself, Hpoint);
    if (H1) *H1 = Hself.row(2);
    if (H2) *H2 = Hpoint.row(2);
    
    return lmk[2]; /// lmk.z()
  }

  /** h(x)-z */
  Vector evaluateError(const Pose3& pose, const Point3& landmark, OptionalMatrixType H1, OptionalMatrixType H2) const override
  {
    try {
      return gtsam::Vector1 ( costFuncHelper(pose, landmark, H1, H2) - measured_ );
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(1, 6);
      if (H2) *H2 = Matrix::Zero(1, 3);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2())
          << " behind Camera " << DefaultKeyFormatter(this->key1()) << std::endl;
    }
    return Vector1(0);
  }

  /** return the measured */
  inline double measured() const {
    return measured_;
  }


};

template<>
struct traits<Landmark3DDepthFactor> : Testable< Landmark3DDepthFactor > {};















/**
 * @brief A factor to constrain the relative distance of two points in 3D
 */
class RelativePoseTranslationDistanceFactor: public NoiseModelFactorN<Pose3, Pose3> {

protected:

  double measured_;


public:

  typedef RelativePoseTranslationDistanceFactor This;
  typedef NoiseModelFactorN<Pose3, Pose3> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;


  RelativePoseTranslationDistanceFactor(double measured, const SharedNoiseModel& model, Key poseKey1, Key poseKey2) :
    Base(model, poseKey1, poseKey2), measured_(measured) {}
  RelativePoseTranslationDistanceFactor():measured_(0.0) {} ///< default constructor

  ~RelativePoseTranslationDistanceFactor() override {} ///< destructor


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "RelativePoseTranslationDistanceFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << (s.empty() ? s : s + " ") << measured_ << std::endl;
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && std::abs(this->measured_ -  e->measured_) <  tol;
  }

  /** h(x)-z */
  double costFuncHelper (const Pose3& pose1, const Pose3& pose2,
                       OptionalJacobian<1, 6> H1, OptionalJacobian<1, 6> H2) const {

      const Point3&  t1 = pose1.translation();
      const Point3&  t2 = pose2.translation();

      Point3 dt = t1 - t2;
      double d = dt.norm();

      dt = dt * (1.0 / d);

      if (H1) {
          *H1 << 0.0, 0.0, 0.0,   dt(0),  dt(1),  dt(2);
      }       
      if (H2) {
          *H2 << 0.0, 0.0, 0.0,  -dt(0), -dt(1), -dt(2);
      }

      return d;
  }

  Vector evaluateError(const Pose3& pose1, const Pose3& pose2,
                        OptionalMatrixType H1, OptionalMatrixType H2) const override
  {
    try {
      return Vector1 ( costFuncHelper(pose1, pose2, H1, H2) - measured_);
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(1, 6);
      if (H2) *H2 = Matrix::Zero(1, 6);
      std::cout << e.what() << ": Relative distance between Pose "<< DefaultKeyFormatter(this->key1())
                            << " and Pose " << DefaultKeyFormatter(this->key2()) << " cannot be evaluated." << std::endl;
    }
    return Vector1(0);
  }

  /** return the measured */
  inline double measured() const {
    return measured_;
  }


};

template<>
struct traits<RelativePoseTranslationDistanceFactor> : Testable< RelativePoseTranslationDistanceFactor > {};









/**
 * @brief A factor to constrain the relative distance of two points in 3D
 */
class Point3NormFactor: public NoiseModelFactorN<Point3> {

protected:

  double measured_;


public:

  typedef Point3NormFactor This;
  typedef NoiseModelFactorN<Point3> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;


  Point3NormFactor(double measured, const SharedNoiseModel& model, Key landmarkKey) :
    Base(model, landmarkKey), measured_(measured) {}
  Point3NormFactor():measured_(0.0) {} ///< default constructor

  ~Point3NormFactor() override {} ///< destructor


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "Point3NormFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << (s.empty() ? s : s + " ") << measured_ << std::endl;
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && std::abs(this->measured_ -  e->measured_) <  tol;
  }

  /** h(x)-z */
  Vector evaluateError(const Point3& lmk, OptionalMatrixType H1) const override
  {
    try {
      return gtsam::Vector1 ( gtsam::norm3(lmk, H1) - measured_ );
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(1, 3);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key1())
                            <<" norm cannot be evaluated." << std::endl;
    }
    return Vector1(0);
  }

  /** return the measured */
  inline double measured() const {
    return measured_;
  }


};

template<>
struct traits<Point3NormFactor> : Testable< Point3NormFactor > {};







/**
 * @brief A factor to constrain the relative distance of two points in 3D
 */
class RelativePointDistanceFactor: public NoiseModelFactorN<Point3, Point3> {

protected:

  double measured_;


public:

  typedef RelativePointDistanceFactor This;
  typedef NoiseModelFactorN<Point3, Point3> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;


  RelativePointDistanceFactor(double measured, const SharedNoiseModel& model, Key landmarkKey1, Key landmarkKey2) :
    Base(model, landmarkKey1, landmarkKey2), measured_(measured) {}
  RelativePointDistanceFactor():measured_(0.0) {} ///< default constructor

  ~RelativePointDistanceFactor() override {} ///< destructor


  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "RelativePointDistanceFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    std::cout << (s.empty() ? s : s + " ") << measured_ << std::endl;
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && std::abs(this->measured_ -  e->measured_) <  tol;
  }

  /** h(x)-z */
  Vector evaluateError(const Point3& lmk1, const Point3& lmk2,
                       OptionalMatrixType H1, OptionalMatrixType H2) const override
  {
    try {
      // std::cout<<"H1 in Factor = " << H1 << std::endl;      
      return gtsam::Vector1 ( gtsam::distance3(lmk1, lmk2, H1, H2) - measured_ );
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(1, 3);
      if (H2) *H2 = Matrix::Zero(1, 3);
      std::cout << e.what() << ": Relative distance between Landmark "<< DefaultKeyFormatter(this->key1())
                            << " and Landmark " << DefaultKeyFormatter(this->key2()) << " cannot be evaluated." << std::endl;
    }
    return Vector1(0);
  }

  /** return the measured */
  inline double measured() const {
    return measured_;
  }


};

template<>
struct traits<RelativePointDistanceFactor> : Testable< RelativePointDistanceFactor > {};







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








#ifdef SURGAR_SELFCALIBRATION_TEST_FANG


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


#endif