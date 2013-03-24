/*! \file webcam_client.cpp
    \example webcam_client.cpp
    \brief Receives images from a streaming server.

    \copyright Copyright (c) 2013, PROFACTOR GmbH, Christoph Heindl
    \license This project is released under the New BSD License.
*/

#include <imagebabble/imagebabble.hpp>
#include <imagebabble/conversion/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

namespace ib = imagebabble;

/** [Example] */
int main(int argc, char *argv[]) 
{
  // Start client
  ib::fast_client< ib::image > ic;
  ic.startup("tcp://127.0.0.1:6001");

  // Try to receive image with timeout
  ib::image ib_image;
  while (ic.receive(ib_image, 5000)) {

    // Convert image
    cv::Mat cv_img;
    ib::cvt_image(ib_image, cv_img, ib::copy_mem());

    // Show image
    cv::imshow("image", cv_img);
    cv::waitKey(1);
  }
  
  return 0;
}
/** [Example] */