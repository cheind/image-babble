/*! \file webcam_server.cpp

    Copyright (c) 2013 PROFACTOR GmbH, Christoph Heindl
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
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