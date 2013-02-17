/*! \file webcam_server.cpp
    \example webcam_server.cpp

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

/** [Namespace Abbrevation] */
namespace ib = ::imagebabble;
/** [Namespace Abbrevation] */

/** [Example] */
int main(int argc, char *argv[]) 
{  
  // Start server
  ib::fast_server is;
  is.startup("tcp://*:6000");
   
  // Open video device
  cv::VideoCapture vc(0);
  cv::Mat cv_img;
   
  // While more images are available ...
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