/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


/**
 * @file gtsam/slam/UncalibratedProjectionFactor_Fang.h.h
 *
 * @brief Bundle adjustment factor (or SfM factor) with an unknown calibration.  
 * The camera type (which specifies the calibratable paramters) is templated in the Factor class.

 * The following factor implements the case where only the folcal lenth is varying duing calibration: 
 * UncalibratedProjectionFactor<Cal3_f>
 * 
 * 
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
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/timing.h>

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif
#include <iostream>
#include <string>



namespace boost {
namespace serialization {
class access;
} /* namespace serialization */
} /* namespace boost */



#include <gtsam/geometry/Cal3.h>


/**
 * Camera calibration value class
 * 
 * Terms in gtsam
 * 
 * K = [fx,  s,  u0
 *       0, fy,  v0
 *       0,  0,  1 ]
 * 
 * Intrinsic coordintes:  [x; y] = [X/Z; Y/Z] for a 3D poitn [X, Y, Z]
 * 
 * Image coordiantes: [u; v] =  [ fx*x + s*y + u0
 *                                      fy*y + v0  ]
 * 
 * uncalibrate:  (x, y) --- > (u, v)
 * calibrate:    (u, v) ----> (x, y), i.e., recompute (x, y) from (u, v) which is also in closed-form.
 * 
 */


/**
 * To implement a new calibration method, what we need is to:
 * defined a new camera class heritied from Cal3, with the dimension (i.e., dof) to update.
 * Update the methods:
 * - uncalibrate()
 * - retract()
 * - localCoordinates()
 * The methods calibrate() and between() are not required and thus removed. See "gtsam/geometry/Cal3_S2.h"
 */


namespace gtsam {

/**
 * @brief Focal length only calibration. fx = fy
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Cal3_f : public Cal3 {
 public:
  enum { dimension = 1 };

  ///< shared pointer to calibration object
  typedef std::shared_ptr<Cal3_f> shared_ptr;

  /// @name Standard Constructors
  /// @{

  /// Create a default calibration that leaves coordinates unchanged
  Cal3_f() = default;

  /// constructor from doubles
  Cal3_f(double fx, double fy, double s, double u0, double v0)
      : Cal3(fx, fy, s, u0, v0) {}

  /// constructor from vector
  Cal3_f(const Vector5& d) : Cal3(d) {}

  /**
   * Easy constructor, takes fov in degrees, asssumes zero skew, unit aspect
   * @param fov field of view in degrees
   * @param w image width
   * @param h image height
   */
  Cal3_f(double fov, int w, int h) : Cal3(fov, w, h) {}

  /**
   * Convert intrinsic coordinates xy to image coordinates uv, fixed derivaitves
   * @param p point in intrinsic coordinates (x, y)
   * @param Dcal optional 2*5 Jacobian wrpt Cal3 parameters (fx, fy, s, u0, v0)
   * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates (x, y)
   * @return point in image coordinates (u, v)
   */
  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 1> Dcal = {}, OptionalJacobian<2, 2> Dp = {}) const {                        
    const double x = p.x(), y = p.y();
    if (Dcal) *Dcal << x,
                      y;
    if (Dp) *Dp << fx_, s_,
                  0.0, fy_;
    return Point2(fx_ * x + s_ * y + u0_, fy_ * y + v0_);
  }

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal3_f& cal) {
    // Use the base class version since it is identical.
    os << (Cal3&)cal;
    return os;
  }

  /// print with optional string
  void print(const std::string& s = "Cal3_f") const override {
    gtsam::print((Matrix)K(), s);
  }

  /// Check if equal up to specified tolerance
  bool equals(const Cal3_f& K, double tol = 10e-9) const {
    return Cal3::equals(K, tol);
  }

  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline static size_t Dim() { return dimension; }

  /// Given 5-dim tangent vector, create new calibration
  inline Cal3_f retract(const Vector & d) const {
    return Cal3_f(fx_ + d(0), fy_ + d(0), s_, u0_, v0_);
  }

  /// Unretraction for the calibration
  Vector1 localCoordinates(const Cal3_f& T2) const {
    const Vector5 tmp = T2.vector() - vector();
    return Vector1(tmp(0));
  }

  /// @}
  /// @name Advanced Interface
  /// @{

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3_f", boost::serialization::base_object<Cal3>(*this));
  }

  /// @}
};

template <>
struct traits<Cal3_f> : public internal::Manifold<Cal3_f> {};

template <>
struct traits<const Cal3_f> : public internal::Manifold<Cal3_f> {};

}  // \ namespace gtsam






#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorSpace.h>


namespace gtsam {


/**
 * @remark every types in the Factor template must be defined on manifold
 * @note a vector space belongs to Lie group, thus is well defined on manifold
 */

class LensDistortDivisionModel : public gtsam::Vector1 {

public:

  enum { dimension = 1 };

  LensDistortDivisionModel () = default;
  LensDistortDivisionModel (double kappa) : gtsam::Vector1() { 
    (*this)[0] = kappa;
  }
  LensDistortDivisionModel (const gtsam::Vector1 & cp) : gtsam::Vector1(cp) {}

  double distDarams () const {double tmp = (*this)[0]; return tmp*tmp; }
  void distParams (double val) { (*this)[0] = std::sqrt(val); }

  friend std::ostream& operator<<(std::ostream& os, const LensDistortDivisionModel& dist) {
    // Use the base class version since it is identical.
    os << (Vector1&)dist;
    return os;
  }

  /// Required by Testable traits
  void print(const std::string& s = "LensDistortDivisionModel") const {
    gtsam::print((Vector)*this, s);
  }

  bool equals(const LensDistortDivisionModel& dist, double tol = 10e-9) const {
    return gtsam::traits<gtsam::Vector1>::Equals(*this, dist, tol);
  }

  /** negDk: return the negative form of original Jacobian, as there is a minus before this function */
  Point2 undistort (const Point2& centered_imgpts, OptionalJacobian<2, 1> minusDk = {}) const {
    double kappa = (*this)[0];
    double rr = centered_imgpts.squaredNorm();
    double s = (1.0/(1.0 + kappa * kappa * rr));
    if (minusDk)
      *minusDk =  s*s*(2*kappa*rr)*centered_imgpts;
    return s*centered_imgpts;
  }


  /** vector space specification */
  const gtsam::Vector1& vector() const { 
    return *this; 
  }
  inline static LensDistortDivisionModel Identity() {
      return LensDistortDivisionModel(0.0);
  }
  template <typename vector_like_t>
  LensDistortDivisionModel operator+(const vector_like_t& b) const {
      LensDistortDivisionModel a = *this;
      a += b;
      return a;
  }
  LensDistortDivisionModel operator-(const LensDistortDivisionModel& b) const
  {
      LensDistortDivisionModel a = *this;
      a -= b;
      return a;
  }

};

template <>
struct traits<LensDistortDivisionModel>  : public internal::VectorSpace<LensDistortDivisionModel> {};


} //namespace



