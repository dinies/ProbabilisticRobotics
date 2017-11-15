#pragma once
#include "ad.h"
#include <Eigen/Geometry>

namespace AD {
  
  template <typename Scalar_> 
  using Vector2ad = Eigen::Matrix<DualValue_<Scalar_>, 2, 1>;

  template <typename Scalar_> 
  using Vector3ad =  Eigen::Matrix<DualValue_<Scalar_>, 3, 1> ;

  template <typename Scalar_> 
  using Vector4ad =  Eigen::Matrix<DualValue_<Scalar_>, 4, 1> ;

  template <typename Scalar_> 
  using Vector5ad =  Eigen::Matrix<DualValue_<Scalar_>, 5, 1> ;

  template <typename Scalar_> 
  using Vector6ad =  Eigen::Matrix<DualValue_<Scalar_>, 6, 1> ;


  template <typename Scalar_> 
  using Matrix4ad =  Eigen::Matrix<DualValue_<Scalar_>, 4, 4> ;

  template <typename Scalar_> 
  using Matrix3ad =  Eigen::Matrix<DualValue_<Scalar_>, 3, 3> ;
  
  template <typename Scalar_> 
  using Isometry3ad =  Eigen::Transform<DualValue_<Scalar_>, 3, Eigen::Affine> ;

  template <typename Scalar_>
  using Quaternionad  =  Eigen::Quaternion<DualValue_<Scalar_> > ;

  template <typename Scalar_> 
  using Vector2 = Eigen::Matrix<Scalar_, 2, 1>;

  template <typename Scalar_> 
  using Vector3 =  Eigen::Matrix<Scalar_, 3, 1> ;

  template <typename Scalar_> 
  using Vector4 =  Eigen::Matrix<Scalar_, 4, 1> ;

  template <typename Scalar_> 
  using Vector5 =  Eigen::Matrix<Scalar_, 5, 1> ;

  template <typename Scalar_> 
  using Vector6 =  Eigen::Matrix<Scalar_, 6, 1> ;


  template <typename Scalar_> 
  using Matrix4 =  Eigen::Matrix<Scalar_, 4, 4> ;

  template <typename Scalar_> 
  using Matrix3 =  Eigen::Matrix<Scalar_, 3, 3> ;
  
  template <typename Scalar_> 
  using Isometry3 =  Eigen::Transform<Scalar_, 3, Eigen::Affine> ;

  template <typename Scalar_>
  using Quaternion  =  Eigen::Quaternion<Scalar_ > ;


  template <typename Scalar_>
  Isometry3<Scalar_> v2t(const Vector6<Scalar_>& v){
    Isometry3<Scalar_> iso;
    iso.translation()=v.head(3);
    Scalar_ n=v.tail(3).squaredNorm();
    if (n>Scalar_(1.0))
      n=Scalar_(1.0);
    Scalar_ w=sqrt(Scalar_(1.0)-n);
    iso.linear()=Quaternion<Scalar_>(w,v(3),v(4),v(5)).toRotationMatrix();
    return iso;
  }

  template <typename Scalar_>
  Vector6<Scalar_> t2v(const Isometry3<Scalar_>& iso){
    Vector6<Scalar_> v;
    Quaternion<Scalar_> q(iso.linear());
    if (q.w()<Scalar_(0)){
      q.coeffs()=-q.coeffs();
    }
    v.block<3,0>(0,0)=iso.translation();
    v.block<3,0>(3,0)=q.vec();
    return v;
  }
   
  
  template <typename Scalar_>
  Matrix3<Scalar_> RotationX(const Scalar_& angle) {
    Matrix3<Scalar_> R;
    const Scalar_ s=sin(angle);
    const Scalar_ c=cos(angle);
    const Scalar_ one(1.);
    const Scalar_ zero(0.);
    R << 
      one, zero, zero,
      zero, c, -s,
      zero, s, c;
    return R;
  }

  template <typename Scalar_>
  Matrix3<Scalar_> RotationY(const Scalar_& angle) {
    Matrix3<Scalar_> R;
    const Scalar_ s=sin(angle);
    const Scalar_ c=cos(angle);
    const Scalar_ one(1.);
    const Scalar_ zero(0.);
    R << 
      c, zero, s,
      zero, one, zero,
      -s, zero, c;
    return R;
  }

  template <typename Scalar_>
  Matrix3<Scalar_> RotationZ(const Scalar_& angle) {
    Matrix3<Scalar_> R;
    const Scalar_ s=sin(angle);
    const Scalar_ c=cos(angle);
    const Scalar_ one(1.);
    const Scalar_ zero(0.);
    R << 
      c, -s, zero,
      s, c, zero,
      zero, zero, one;
    return R;
  }

  template <typename Scalar_>
  Matrix3<Scalar_> Rotation(const Vector3<Scalar_>& angles) {
    return RotationX(angles.x())*RotationY(angles.y())*RotationZ(angles.z());
  }


  template <typename Scalar_>
  Matrix3<Scalar_> skew(const Vector3<Scalar_> v){
    Matrix3<Scalar_> S;
    const Scalar_ zero(0.);
    S << 
      zero, -v[2], v[1], 
      v[2], zero, -v[0], 
      -v[1], v[0], zero;
    return S;
  }
  
}
