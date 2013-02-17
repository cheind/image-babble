/*! \file imagegroup_server.cpp
    \example imagegroup_server.cpp

    Copyright (c) 2013, PROFACTOR GmbH, Christoph Heindl
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * Neither the name of PROFACTOR GmbH nor the
          names of its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL PROFACTOR GmbH BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
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