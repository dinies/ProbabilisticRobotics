#pragma once
#include <cmath>

namespace AD {

  /**basic functions for float, redefined to please the compiler*/
  float sin(float f){ return sinf(f);}
  float cos(float f){ return cosf(f);}
  float exp(float f){ return expf(f);}
  float log(float f){ return logf(f);}
  float sqrt(float f){return sqrtf(f);}
  float atan2(float y, float x) {return atan2f(y,x);}

  /**basic functions for double, redefined to please the compiler*/
  double sin(double f){ return sinl(f);}
  double cos(double f){ return cosl(f);}
  double exp(double f){ return expl(f);}
  double log(double f){ return logl(f);}
  double sqrt(double f){return sqrtl(f);}
  double atan2(double y, double x) {return atan2l(y,x);}

  /**dual value, stores the elements of autodiff
     it reperesent a pair
     u, u'
     and defines all common operators	      
     +,-,*,/, and so on

     DualValue is parametiric w.r.t the base type (float or double)
  */

  template <typename T>
  class DualValue_{
  public:
    typedef T BaseType;
    T value;
    T derivative;      

    DualValue_(){value=0, derivative=0;}
    DualValue_(const T& v) {
      value=v;
      derivative=0;
    }

    DualValue_(const T& v, const T& d) {
      value=v;
      derivative=d;
    }

    DualValue_& operator=(const T& v) {
      value=v;
      derivative=0;
      return *this;
    }

    DualValue_& operator +=(const DualValue_& op){
      value+=op.value;
      derivative+=op.derivative;
      return *this;
    }

    DualValue_& operator -=(const DualValue_& op){
      value-=op.value;
      derivative-=op.derivative;
      return *this;
    }

    DualValue_& operator *=(const DualValue_& op){
      value*=op.value;
      derivative=derivative*op.value+value*op.derivative;
      return *this;
    }

    DualValue_& operator /=(const DualValue_& op){
      value/=op.value;
      derivative=(derivative*op.value-value*op.derivative)/(op.value*op.value);
      return *this;
    }

    bool operator>(const DualValue_& v) const {
      return value>v.value;
    }

    bool operator==(const DualValue_& v) const {
      return value==v.value;
    }

    bool operator<(const DualValue_& v) const {
      return value<v.value;
    }

  };


  
  typedef DualValue_<float> DualValuef;
  typedef DualValue_<float> DualValued;

  template <typename T>
  DualValue_<T> operator+(const DualValue_<T>& op){
    return op;
  }

  template <typename T>
  DualValue_<T> operator-(const DualValue_<T>& op2){
    return DualValue_<T>(-op2.value, -op2.derivative);
  }

  template <typename T>
  DualValue_<T> operator+(const DualValue_<T>& op1, const DualValue_<T>& op2){
    return DualValue_<T>(op1.value+op2.value, op1.derivative+op2.derivative);
  }

  template <typename T>
  DualValue_<T> operator-(const DualValue_<T>& op1, const DualValue_<T>& op2){
    return DualValue_<T>(op1.value-op2.value, op1.derivative-op2.derivative);
  }

  template <typename T>
  DualValue_<T> operator*(const DualValue_<T>& op1, const DualValue_<T>& op2){
    return DualValue_<T>(op1.value*op2.value,op1.derivative*op2.value+op1.value*op2.derivative);
  }

  template <typename T>
  DualValue_<T> operator/(const DualValue_<T>& op1, const DualValue_<T>& op2){
    return DualValue_<T>(op1.value/op2.value, (op1.derivative*op2.value-op1.value*op2.derivative)/(op2.value*op2.value));
  }

  template <typename T>
  DualValue_<T> sin(const DualValue_<T>& op){
    return DualValue_<T>(sin(op.value), cos(op.value)*op.derivative);
  }

  template <typename T>
  DualValue_<T> cos(const DualValue_<T>& op){
    return DualValue_<T>(cos(op.value), -sin(op.value)*op.derivative);
  }

  template <typename T>
  DualValue_<T> log(const DualValue_<T>& op){
    return DualValue_<T>(log(op.value), 1./fabs(op.value)*op.derivative);
  }

  template <typename T>
  DualValue_<T> exp(const DualValue_<T>& op){
    return DualValue_<T>(exp(op.value), exp(op.value)*op.derivative);
  }

  template <typename T>
  DualValue_<T> sqrt(const DualValue_<T>& op){
    return DualValue_<T>(sqrt(op.value), 0.5/sqrt(op.value)*op.derivative);
  }

  template <typename T>
  DualValue_<T> atan2(const DualValue_<T>& op1, const DualValue_<T>& op2){
    return DualValue_<T>(atan2(op1.value, op2.value), 
			 1./(1+pow(op1.value/op2.value,2))
			 *(op1.derivative*op2.value-op1.value*op2.derivative)
			 /(op2.value*op2.value));
  }



}

