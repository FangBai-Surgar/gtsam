
/**
 * @file gtsam/slam/Fang_SelfCalibrationTypes.h
 * @brief Implementation of camera calibration and lens distortion types
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



#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
namespace boost {
namespace serialization {
class access;
} /* namespace serialization */
} /* namespace boost */
#endif









namespace gtsam {

class GTSAM_EXPORT Cal_fd {

private:

  double f_;
  double kappa_;
  double u0_;
  double v0_;


public:

  enum { dimension = 2 };

  typedef std::shared_ptr<Cal_fd> shared_ptr;

public:

  Cal_fd (double f, double kappa, double u0, double v0) :
          f_(f), kappa_(kappa), u0_(u0), v0_(v0) {}


  Point2 uncalibrate(const Point2& p, OptionalJacobian<2, 2> Dcal = {}, OptionalJacobian<2, 2> Dp = {}) const {                       

    const double x = p.x(), y = p.y();
    const double rr = x*x + y*y;
    const double c = 1.0 + kappa_*rr;

    /** d_focal,  d_kappa */
    if (Dcal) *Dcal << c*x, f_*rr*x,
                       c*y, f_*rr*y;

    double fk = f_*kappa_;

    if (Dp) *Dp << 2*fk*x*x + f_*c,   2*fk*x*y,
                   2*fk*y*x,         2*fk*y*y + f_*c;

    return Point2(f_*c*x + u0_, f_*c*y + v0_);

  }


  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline static size_t Dim() { return dimension; }

  /// Given 2-dim tangent vector, create new calibration
  inline Cal_fd retract(const Vector2 & d) const {
    return Cal_fd(f_ + d(0), kappa_ + d(1), u0_, v0_);
  }

  Vector4 vector() const {
    Vector4 v;
    v << f_, kappa_, u0_, v0_;
    return v;
  }

  /// Unretraction for the calibration
  Vector2 localCoordinates(const Cal_fd& T2) const {
    const Vector4 tmp = T2.vector() - this->vector();
    return Vector2(tmp(0), tmp(1));
  }


  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                               const Cal_fd& cal) {
    os << "focal: " << cal.f_ << ", kappa: " << cal.kappa_ 
       << ", u0: " << cal.u0_ << ", v0: " << cal.v0_;
    return os;
  }

  /// print with optional string
  void print(const std::string& s = "Cal_fd") const {
    gtsam::print((Vector)vector(), s);
  }

  /// Check if equal up to specified tolerance
  bool equals(const Cal_fd& K, double tol = 10e-9) const {
    return ( std::fabs(f_ - K.f_) < tol && 
             std::fabs(kappa_ - K.kappa_) < tol && 
             std::fabs(u0_ - K.u0_) < tol &&
             std::fabs(v0_ - K.v0_) < tol );
  }

};


template <>
struct traits<Cal_fd> : public internal::Manifold<Cal_fd> {};

template <>
struct traits<const Cal_fd> : public internal::Manifold<Cal_fd> {};

}  // namespace gtsam



















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

  /// constructor from matrix
  Cal3_f(const Matrix33& m) : Cal3(m(0,0), m(1,1), m(0,1), m(0,2), m(1,2)) {}
  

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
  #ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "Cal3_f", boost::serialization::base_object<Cal3>(*this));
  }
  #endif
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

  typedef std::shared_ptr<LensDistortDivisionModel> shared_ptr;


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
    double s = (1.0/(1.0 + kappa * rr));
    if (minusDk)
      *minusDk =  s*s*rr*centered_imgpts;
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





namespace gtsam {


/**
 * @remark every types in the Factor template must be defined on manifold
 * @note a vector space belongs to Lie group, thus is well defined on manifold
 */

class LensDistortFieldOfViewModel : public gtsam::Vector1 {

public:

  enum { dimension = 1 };

  typedef std::shared_ptr<LensDistortFieldOfViewModel> shared_ptr;


  LensDistortFieldOfViewModel () = default;
  LensDistortFieldOfViewModel (double kappa) : gtsam::Vector1() { 
    (*this)[0] = kappa;
  }
  LensDistortFieldOfViewModel (const gtsam::Vector1 & cp) : gtsam::Vector1(cp) {}

  double distDarams () const {double tmp = (*this)[0]; return tmp*tmp; }
  void distParams (double val) { (*this)[0] = std::sqrt(val); }

  friend std::ostream& operator<<(std::ostream& os, const LensDistortFieldOfViewModel& dist) {
    // Use the base class version since it is identical.
    os << (Vector1&)dist;
    return os;
  }

  /// Required by Testable traits
  void print(const std::string& s = "LensDistortFieldOfViewModel") const {
    gtsam::print((Vector)*this, s);
  }

  bool equals(const LensDistortFieldOfViewModel& dist, double tol = 10e-9) const {
    return gtsam::traits<gtsam::Vector1>::Equals(*this, dist, tol);
  }

  /** negDk: return the negative form of original Jacobian, as there is a minus before this function */
  Point2 undistort (const Point2& centered_imgpts, OptionalJacobian<2, 1> minusDk = {}) const {
    double kappa = (*this)[0];
    double r = centered_imgpts.norm();
    double tmp1 = std::tan(r*kappa);
    double tmp2 = std::tan(kappa/2.0);
    double c1 = std::cos(r*kappa);
    double c2 = std::cos(kappa/2.0);
    double s = 1.0/(2.0*tmp2*c1*c1) - tmp1/(4.0*r*tmp2*tmp2*c2*c2);
    if (minusDk)
      *minusDk =  -s*centered_imgpts;
    return ( tmp1/(2.0*r*tmp2) ) * centered_imgpts;
  }


  /** vector space specification */
  const gtsam::Vector1& vector() const { 
    return *this; 
  }
  inline static LensDistortFieldOfViewModel Identity() {
      return LensDistortFieldOfViewModel(0.0);
  }
  template <typename vector_like_t>
  LensDistortFieldOfViewModel operator+(const vector_like_t& b) const {
      LensDistortFieldOfViewModel a = *this;
      a += b;
      return a;
  }
  LensDistortFieldOfViewModel operator-(const LensDistortFieldOfViewModel& b) const
  {
      LensDistortFieldOfViewModel a = *this;
      a -= b;
      return a;
  }

};

template <>
struct traits<LensDistortFieldOfViewModel>  : public internal::VectorSpace<LensDistortFieldOfViewModel> {};


} //namespace









namespace gtsam {


/**
 * @remark every types in the Factor template must be defined on manifold
 * @note a vector space belongs to Lie group, thus is well defined on manifold
 */

class LensDistortRadialFirstOrder : public gtsam::Vector1 {

public:

  enum { dimension = 1 };

  typedef std::shared_ptr<LensDistortRadialFirstOrder> shared_ptr;


  LensDistortRadialFirstOrder () = default;
  LensDistortRadialFirstOrder (double kappa) : gtsam::Vector1() { 
    (*this)[0] = kappa;
  }
  LensDistortRadialFirstOrder (const gtsam::Vector1 & cp) : gtsam::Vector1(cp) {}

  double distDarams () const {double tmp = (*this)[0]; return tmp*tmp; }
  void distParams (double val) { (*this)[0] = std::sqrt(val); }

  friend std::ostream& operator<<(std::ostream& os, const LensDistortRadialFirstOrder& dist) {
    // Use the base class version since it is identical.
    os << (Vector1&)dist;
    return os;
  }

  /// Required by Testable traits
  void print(const std::string& s = "LensDistortRadialFirstOrder") const {
    gtsam::print((Vector)*this, s);
  }

  bool equals(const LensDistortRadialFirstOrder& dist, double tol = 10e-9) const {
    return gtsam::traits<gtsam::Vector1>::Equals(*this, dist, tol);
  }


  /** @brief Dp: Jacobian wrt centered projective point 
   *         Dk: Jacobian wrt radial distortion parameter
  */
  Point2 distort (const Point2& centered_proj,
                        OptionalJacobian<2, 6> Dpose = {},
                        OptionalJacobian<2, 3> Dpoint = {},
                        OptionalJacobian<2, 1> Dcal = {},
                        OptionalJacobian<2, 1> Ddist = {}) const {
    double kappa = (*this)[0];
    double rr = centered_proj.squaredNorm();
    double s = 1.0 + kappa*rr;
    if (Dpose || Dpoint || Dcal || Ddist) {
      gtsam::Matrix22 Dp = s*I_2x2 + 2.0*kappa*centered_proj*centered_proj.transpose();
      if (Dpose)
        *Dpose = Dp * (*Dpose);
      if (Dpoint)
        *Dpoint = Dp * (*Dpoint);
      if (Dcal)
        *Dcal = Dp * (*Dcal);
      if (Ddist)
        *Ddist = rr*centered_proj;
    }
    return s*centered_proj;
  }


  /** vector space specification */
  const gtsam::Vector1& vector() const { 
    return *this; 
  }
  inline static LensDistortRadialFirstOrder Identity() {
      return LensDistortRadialFirstOrder(0.0);
  }
  template <typename vector_like_t>
  LensDistortRadialFirstOrder operator+(const vector_like_t& b) const {
      LensDistortRadialFirstOrder a = *this;
      a += b;
      return a;
  }
  LensDistortRadialFirstOrder operator-(const LensDistortRadialFirstOrder& b) const
  {
      LensDistortRadialFirstOrder a = *this;
      a -= b;
      return a;
  }

};

template <>
struct traits<LensDistortRadialFirstOrder>  : public internal::VectorSpace<LensDistortRadialFirstOrder> {};


} //namespace



