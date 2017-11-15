#pragma once
#include "defs.h"

namespace thin_localizer {

  /**Cell of a distance map.
     Each cell has a row, a column, a parent (which is the closest occupied cell), and a distance (which is a distance to the cell).
   */
  struct DistanceMapCell{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //! ctor
    DistanceMapCell(){
      parent = 0;
      distance = std::numeric_limits<int>::max();
      r = 0;
      c = 0;
    }

    //! the closest occupied cell
    DistanceMapCell* parent;
    //! the distance to the closest
    float distance;
    //! row and column
    int r, c;
    //! weight factor for the distance (smaller for nearer objects);
    float weight;
  };

  //! matrix of cells
  typedef Eigen::Matrix<DistanceMapCell, Eigen::Dynamic, Eigen::Dynamic> DistanceMap;

  //! computes a distance map from an int image. The cells in the image having a value >-1 are considered as occupied.
  //! @param dmap: the output map
  //! @param imap: an image whose cell [i,j] contains the value of the closest occupied cell, in indexImage. this is overwritten.
  //! @param indexImage: this is the input. Each occupied cell should have a unique value >-1. This value is used to refer to the closest point in the imap
  //! @param maxDistance: stops the expansion at this value.
void makeDistanceMap(DistanceMap& dmap, IntImage& imap, FloatImage& distances, 
		     const IntImage& indexImage, float maxDistance = 100, const FloatImage& weights=FloatImage(0,0));

  //! makes an inage out of a dmap
  void dmap2img(UnsignedCharImage& img, const DistanceMap& dmap);

}
