#include "ad.h"
#include <Eigen/Core>

namespace AD {
  
  template <typename Scalar_ , int InputSize_, int OutputSize_>
  class MultivariateFunction{
  public:
    typedef Scalar_ Scalar; 
    static const int InputSize=InputSize_;
    static const int OutputSize=OutputSize_;
 
    //virtual void operator()(Scalar_* output, const Scalar_* input)=0;
  };


  template <typename BaseScalar_, template <typename _T> class FunctorType_>
  class ADMultivariateFunction: public  FunctorType_< DualValue_<BaseScalar_> >{
  public:
    static const int InputSize=FunctorType_< DualValue_<BaseScalar_> >::InputSize;
    static const int OutputSize=FunctorType_< DualValue_<BaseScalar_> >::OutputSize;

    void operator()(BaseScalar_* output, const BaseScalar_* input){
      DualValue_<BaseScalar_> ad_input[InputSize];
      DualValue_<BaseScalar_> ad_output[OutputSize];
      for (size_t c=0; c<InputSize; c++){
	ad_input[c].value=input[c];
	ad_input[c].derivative=0;
      }
      FunctorType_<DualValue_<BaseScalar_> >::operator()(ad_output,ad_input);
      for (size_t r=0; r<OutputSize; r++){
	output[r]=ad_output[r].value;
      }
    }

    typedef 
    typename Eigen::Matrix<BaseScalar_, OutputSize, InputSize>
    JacobianType;
    
    JacobianType jacobian(const BaseScalar_* input){
      JacobianType jacobian=JacobianType::Zero();
      const int rows=FunctorType_<BaseScalar_>::OutputSize;
      const int cols=FunctorType_<BaseScalar_>::InputSize;
      DualValue_<BaseScalar_> ad_input[cols];
      DualValue_<BaseScalar_> ad_output[rows];
      for (size_t c=0; c<cols; c++){
	ad_input[c].value=input[c];
	ad_input[c].derivative=0;
      }
      for (size_t c=0; c<cols; c++){
	ad_input[c].derivative=1.0;
	FunctorType_<DualValue_<BaseScalar_> >::operator()(ad_output, ad_input);
	for (size_t r=0; r<rows; r++){
	  jacobian(r,c)=ad_output[r].derivative;
	}
	ad_input[c].derivative=0.0;
      }
      return jacobian;
    }
  };

}
