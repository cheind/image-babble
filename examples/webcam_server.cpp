/*! \file webcam_server.cpp
    \example webcam_server.cpp
    \brief Streams images from a video device to clients.

    \copyright Copyright (c) 2013, PROFACTOR GmbH, Christoph Heindl
    \license This project is released under the New BSD License.
*/

/** [Include Statement] */
#include <imagebabble/imagebabble.hpp>
#include <imagebabble/conversion/opencv.hpp>
/** [Include Statement] */

#include <opencv2/opencv.hpp>
#include <iostream>

/** [Namespace Abbrevation] */
namespace ib = ::imagebabble;
/** [Namespace Abbrevation] */

/** [Example] */
int main(int argc, char *argv[]) 
{  
  // Start server
  ib::fast_server< ib::image > is;
  is.set_max_pending_outbound(100);
  is.startup("tcp://*:6000");
   
  // Open video device
  cv::VideoCapture vc(0);
  cv::Mat cv_img;
   
  // While more images are available ...
  int count = 0;
  while (vc.grab() && vc.retrieve(cv_img)) 
  {
    // Convert image
    ib::image ib_img;
    ib::cvt_image(cv_img, ib_img, ib::copy_mem());

    // Publish to all connected clients
    is.publish(ib_img);
  }
  
  return 0;
}
/** [Example] */