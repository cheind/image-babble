/*! \file imagegroup_server.cpp
    \example imagegroup_server.cpp
    \brief Sends a collection of images at once.

    \copyright Copyright (c) 2013, PROFACTOR GmbH, Christoph Heindl
    \license This project is released under the New BSD License.
  */

/** [Include Statement] */
#include <imagebabble/imagebabble.hpp>
#include <imagebabble/conversion/opencv.hpp>
/** [Include Statement] */

#include <opencv2/opencv.hpp>
#include <iostream>

namespace ib = imagebabble;

/** [Example] */
int main(int argc, char *argv[]) 
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <address>" << std::endl;
    return -1;
  }

  // Load a bunch of images
  cv::Mat cv_img[2];
  cv_img[0] = cv::imread("image_0.png");
  cv_img[1] = cv::imread("image_1.png");

  if (cv_img[0].empty() || cv_img[1].empty()) {
    std::cerr << "Could not find images" << std::endl;
    return -1;
  }

  // Start server
  ib::reliable_server is;
  is.startup(argv[1]);

  // Form group
  ib::image_group g;
  g.add_image(ib::cvt_image< ib::image >(cv_img[0], ib::share_mem()), "first");
  g.add_image(ib::cvt_image< ib::image >(cv_img[1], ib::share_mem()), "second");
  
  // Publish
  is.publish(g);

  // Shutdown
  is.shutdown();
  
  return 0;
}
/** [Example] */