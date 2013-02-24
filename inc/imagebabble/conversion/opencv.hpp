/*! \file opencv.hpp

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

#ifndef __IMAGE_BABBLE_CVT_OPENCV_HPP_INCLUDED__
#define __IMAGE_BABBLE_CVT_OPENCV_HPP_INCLUDED__

#include "../core.hpp"
#include "../image_support.hpp"

#include <opencv2/core/core.hpp>

namespace imagebabble {

  /** Convert from OpenCV matrix to image. */
  template<class MemOp>
  inline void cvt_image(const cv::Mat &src, image &to, const MemOp &m) 
  {
    to = image(src.cols, src.rows, src.step, src.data, m);
    to.set_external_type(src.type());
  }

  /** Convert from image to OpenCV matrix. */
  inline void cvt_image(const image &src, cv::Mat &to, const copy_mem &m) 
  {
    switch (src.get_format()) {
    case image::FORMAT_BGR_888:
    case image::FORMAT_RGB_888:
      to.create(src.get_height(), src.get_width(), CV_8UC3);
      break;
    case image::FORMAT_GRAY_8:
      to.create(src.get_height(), src.get_width(), CV_8UC1);
      break;
    case image::FORMAT_UNKNOWN:
      to.create(src.get_height(), src.get_width(), src.get_external_type());
      break;
    }

    if ((to.rows * to.step) != src.size()) {
      throw ib_error(ib_error::ECONVERSION);
    }

    memcpy(to.data, src.ptr<void>(), src.size());    
  }

  /** Convert from image to OpenCV matrix. */
  inline void cvt_image(const image &src, cv::Mat &to, const share_mem &m) 
  {
    int type = -1;

    switch (src.get_format()) {
    case image::FORMAT_BGR_888:
    case image::FORMAT_RGB_888:
      type = CV_8UC3;
      break;
    case image::FORMAT_GRAY_8:
      type = CV_8UC1;
      break;
    case image::FORMAT_UNKNOWN:
      // Use external type
      type = src.get_external_type();
      break;
    }

    to = cv::Mat( 
      src.get_height(), src.get_width(), type, 
      const_cast<void*>(src.ptr<void>()), 
      src.get_step()
    );
  }
  
}




#endif