/*! \file imagegroup_client.cpp
    \example imagegroup_client.cpp
    \brief Receives a collection of images at once.

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
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " address" << std::endl;
    return -1;
  }

  ib::reliable_client< ib::image_group > ic;
  ic.startup(argv[1]);

  ib::image_group g;
  if (ic.receive(g, 5000)) {

    std::vector<ib::image> &images = g.get_images();
    std::vector<std::string> &names = g.get_names();

    for (size_t i = 0; i < images.size(); ++i) {

      cv::Mat cv_img = ib::cvt_image< cv::Mat >(images[i], ib::share_mem());      
      //cv::imshow(names[i], cv_img);
    }

  }

  ic.shutdown();

  cv::waitKey();
  
  return 0;
}
/** [Example] */