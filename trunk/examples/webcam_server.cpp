/*! \file webcam_server.cpp

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


#include <imagebabble.hpp>
#include <opencv2\opencv.hpp>
#include <iostream>

int main(int argc, char *argv[]) 
{
  namespace ib = imagebabble;

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " address" << std::endl;
    return -1;
  }

  // Open video device

  cv::VideoCapture vc;
  vc.open(0);

  if (!vc.isOpened()) {
    std::cerr << "Failed to open video device" << std::endl;
    return -1;
  }

  // Start server
  ib::fast_image_server is;
  is.startup(argv[1]);

  cv::Mat img;
  
  while (vc.grab()) {
    vc.retrieve(img);

    ib::image_header ih(img.cols, img.rows, 3, 1, "");
    ib::image_data id(img.ptr(), ih.get_total_bytes(), ib::copy_mem());

    ib::frame f;
    f.append_image_header(std::move(ih));
    f.append_image_data(std::move(id));

    is.publish(f);
  }

  is.shutdown();

	return -1;
}