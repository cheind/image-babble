/*! \file test_image_opencv.cpp

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

#include <boost/test/unit_test.hpp>

#include <imagebabble/imagebabble.hpp>
#include <imagebabble/conversion/opencv.hpp>

BOOST_AUTO_TEST_SUITE(test_image_opencv)

namespace ib = imagebabble;

BOOST_AUTO_TEST_CASE(convert)
{
  cv::Mat cv_img(cv::Size(640,480), CV_8UC3);
  ib::image ib_img = ib::cvt_image< ib::image >(cv_img, ib::share_mem());  
  cv::Mat cv_img2 = ib::cvt_image< cv::Mat > (ib_img, ib::share_mem());

  BOOST_REQUIRE_EQUAL(ib::image::FORMAT_UNKNOWN, ib_img.get_format());

  BOOST_REQUIRE_EQUAL(cv_img.rows, cv_img2.rows);
  BOOST_REQUIRE_EQUAL(cv_img.cols, cv_img2.cols);
  BOOST_REQUIRE_EQUAL(cv_img.step, cv_img2.step);
  BOOST_REQUIRE_EQUAL(cv_img.type(), cv_img2.type());
  BOOST_REQUIRE_EQUAL(cv_img.data, cv_img2.data);
  BOOST_REQUIRE_EQUAL(cv_img.data, ib_img.ptr<void>());

  ib::image ib_img2 = ib::cvt_image< ib::image >(cv_img, ib::copy_mem());
  cv::Mat cv_img3 = ib::cvt_image< cv::Mat > (ib_img2, ib::copy_mem());

  BOOST_REQUIRE_EQUAL(cv_img.rows, cv_img3.rows);
  BOOST_REQUIRE_EQUAL(cv_img.cols, cv_img3.cols);
  BOOST_REQUIRE_EQUAL(cv_img.step, cv_img3.step);
  BOOST_REQUIRE_EQUAL(cv_img.type(), cv_img3.type());
  BOOST_REQUIRE_NE(cv_img.data, cv_img3.data);
  BOOST_REQUIRE_NE(cv_img.data, ib_img2.ptr<void>());
  BOOST_REQUIRE_NE(cv_img3.data, ib_img2.ptr<void>());
}

BOOST_AUTO_TEST_CASE(convert_format)
{
  cv::Mat cv_img(cv::Size(640,480), CV_8UC3);
  ib::image ib_img = ib::cvt_image< ib::image >(cv_img, ib::share_mem());  
  ib_img.set_format(ib::image::FORMAT_RGB_888);
  ib_img.set_external_type(-1);
  cv::Mat cv_img2 = ib::cvt_image< cv::Mat > (ib_img, ib::share_mem());

  BOOST_REQUIRE_EQUAL(cv_img.rows, cv_img2.rows);
  BOOST_REQUIRE_EQUAL(cv_img.cols, cv_img2.cols);
  BOOST_REQUIRE_EQUAL(cv_img.step, cv_img2.step);
  BOOST_REQUIRE_EQUAL(cv_img.type(), cv_img2.type());
  BOOST_REQUIRE_EQUAL(cv_img.data, cv_img2.data);
  BOOST_REQUIRE_EQUAL(cv_img.data, ib_img.ptr<void>());

  ib::image ib_img2 = ib::cvt_image< ib::image >(cv_img, ib::copy_mem());
  cv::Mat cv_img3 = ib::cvt_image< cv::Mat > (ib_img2, ib::copy_mem());

  BOOST_REQUIRE_EQUAL(cv_img.rows, cv_img3.rows);
  BOOST_REQUIRE_EQUAL(cv_img.cols, cv_img3.cols);
  BOOST_REQUIRE_EQUAL(cv_img.step, cv_img3.step);
  BOOST_REQUIRE_EQUAL(cv_img.type(), cv_img3.type());
  BOOST_REQUIRE_NE(cv_img.data, cv_img3.data);
  BOOST_REQUIRE_NE(cv_img.data, ib_img2.ptr<void>());
  BOOST_REQUIRE_NE(cv_img3.data, ib_img2.ptr<void>());
}

BOOST_AUTO_TEST_SUITE_END()
