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

  // Start server
  ib::fast_image_client ic;
  ic.startup(argv[1]);
  
  ib::frame f;
  while (ic.receive(f, 5000)) {

    cv::Mat img;
    
    ib::image_header &ih = f.get_image_headers().front();
    ib::image_data &id = f.get_image_data().front();

    img.create(ih.get_height(), ih.get_width(), CV_8UC3);
    id.copy_data(img.ptr(), img.step * img.rows);
    
    cv::imshow("cam", img);
    cv::waitKey(5);    
  }

  ic.shutdown();

	return -1;
}