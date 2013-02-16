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
  inline bool cvt_image(const cv::Mat &src, image &to, const MemOp &m) 
  {
    to = image(src.cols, src.rows, src.type(), src.step, src.data, m);
    return true;
  }

  /** Convert from image to OpenCV matrix. */
  inline bool cvt_image(const image &src, cv::Mat &to, const copy_mem &m) 
  {
    to.create(src.get_height(), src.get_width(), src.get_type());
    
    if (to.elemSize1() != src.size()) {
      return false;
    }

    memcpy(to.data, src.ptr<void>(), src.size());    
    return true;
  }

  /** Convert from image to OpenCV matrix. */
  inline bool cvt_image(const image &src, cv::Mat &to, const share_mem &m) 
  {
    to = cv::Mat(
      src.get_height(), src.get_width(), src.get_type(), 
      const_cast<void*>(src.ptr<void>()), src.get_step()
    );

    return true;
  }
  
}




#endif