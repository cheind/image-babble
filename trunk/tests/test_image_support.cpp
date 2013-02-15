/*! \file test_image_support.cpp

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
#include "test_utilities.hpp"

BOOST_AUTO_TEST_SUITE(test_image_support)

namespace ib = imagebabble;

BOOST_AUTO_TEST_CASE(image)
{
  {
    ib::image i;
    BOOST_REQUIRE_EQUAL(0, i.get_width());
    BOOST_REQUIRE_EQUAL(0, i.get_height());
    BOOST_REQUIRE_EQUAL(0, i.get_bytes_per_pixel());
    BOOST_REQUIRE_EQUAL(0, i.size());    
  }

  {
    ib::image i(640,480,3);
    BOOST_REQUIRE_EQUAL(640, i.get_width());
    BOOST_REQUIRE_EQUAL(480, i.get_height());
    BOOST_REQUIRE_EQUAL(3, i.get_bytes_per_pixel());
    BOOST_REQUIRE_EQUAL(640*480*3, i.size());    
  }

  {
    int x = 10;
    ib::image i(1,1,4,&x, ib::share_mem());

    BOOST_REQUIRE_EQUAL(1, i.get_width());
    BOOST_REQUIRE_EQUAL(1, i.get_height());
    BOOST_REQUIRE_EQUAL(4, i.get_bytes_per_pixel());
    BOOST_REQUIRE_EQUAL(4, i.size());    
    BOOST_REQUIRE_EQUAL(&x, i.ptr<int>());    
  }

  {
    int x = 10;
    ib::image i(1,1,4,&x, ib::copy_mem());

    BOOST_REQUIRE_EQUAL(1, i.get_width());
    BOOST_REQUIRE_EQUAL(1, i.get_height());
    BOOST_REQUIRE_EQUAL(4, i.get_bytes_per_pixel());
    BOOST_REQUIRE_EQUAL(4, i.size());    
    BOOST_REQUIRE_NE(&x, i.ptr<int>());    
  }

#ifdef IMAGEBABBLE_HAS_RVALUE_REFS
   {

      int x = 10;
      ib::image i0(1,1,4,&x, ib::share_mem());
      ib::image i1(std::move(i0));

      BOOST_REQUIRE_EQUAL(1, i1.get_width());
      BOOST_REQUIRE_EQUAL(1, i1.get_height());
      BOOST_REQUIRE_EQUAL(4, i1.get_bytes_per_pixel());
      BOOST_REQUIRE_EQUAL(4, i1.size());    
      BOOST_REQUIRE_EQUAL(&x, i1.ptr<int>());    

      ib::image i2 = std::move(i1);

      BOOST_REQUIRE_EQUAL(1, i2.get_width());
      BOOST_REQUIRE_EQUAL(1, i2.get_height());
      BOOST_REQUIRE_EQUAL(4, i2.get_bytes_per_pixel());
      BOOST_REQUIRE_EQUAL(4, i2.size());    
      BOOST_REQUIRE_EQUAL(&x, i2.ptr<int>());   

      i2 = std::move(i2);
    
      BOOST_REQUIRE_EQUAL(1, i2.get_width());
      BOOST_REQUIRE_EQUAL(1, i2.get_height());
      BOOST_REQUIRE_EQUAL(4, i2.get_bytes_per_pixel());
      BOOST_REQUIRE_EQUAL(4, i2.size());    
      BOOST_REQUIRE_EQUAL(&x, i2.ptr<int>());

      /* I believe zeromq does this is by design. It copies the data if the message size is 
         small and reference counts it if its above a certain size. It looks like the size 
         threshold is specified by the max_vsm_size enum in msg.hpp, currently set to 29, so if 
         you make your message size 30 it should switch to reference counting. */

      const int zmq_min_size_ref_count = 30;
      const int elems = (zmq_min_size_ref_count / sizeof(int)) + 1;

      // should ref-count

      i0 = ib::image(1, elems, sizeof(int));
      int *adr = i0.ptr<int>();
      i1 = std::move(i0);

      BOOST_REQUIRE_EQUAL(1, i1.get_width());
      BOOST_REQUIRE_EQUAL(elems, i1.get_height());
      BOOST_REQUIRE_EQUAL(sizeof(int), i1.get_bytes_per_pixel());
      BOOST_REQUIRE_EQUAL(elems * sizeof(int), i1.size());    
      BOOST_REQUIRE_EQUAL(adr, i1.ptr<int>());

      // should not ref count

      i0 = ib::image(1, elems - 1, sizeof(int));
      adr = i0.ptr<int>();
      i1 = std::move(i0);
  
      BOOST_REQUIRE_NE(adr, i1.ptr<int>());

    }
#endif
}


BOOST_AUTO_TEST_CASE(send_receive_image)
{
  send_receive sr;

  sr.set_send([](ib::reliable_server &s){
    
    ib::image img(1, 5, sizeof(int));
    for (int i = 0; i < img.get_height(); ++i) {
      img.ptr<int>()[i] = i;
    } 

    BOOST_REQUIRE(s.publish(img));
  });

  sr.set_recv([](ib::reliable_client &c) {
    
    ib::image img;
    BOOST_REQUIRE(c.receive(img));
    BOOST_REQUIRE_EQUAL(1, img.get_width());
    BOOST_REQUIRE_EQUAL(5, img.get_height());
    BOOST_REQUIRE_EQUAL(sizeof(int), img.get_bytes_per_pixel());
    BOOST_REQUIRE_EQUAL(0, img.ptr<int>()[0]);
    BOOST_REQUIRE_EQUAL(1, img.ptr<int>()[1]);
    BOOST_REQUIRE_EQUAL(2, img.ptr<int>()[2]);
    BOOST_REQUIRE_EQUAL(3, img.ptr<int>()[3]);
    BOOST_REQUIRE_EQUAL(4, img.ptr<int>()[4]);    

  });

  BOOST_REQUIRE(sr.run());
}

BOOST_AUTO_TEST_CASE(send_receive_image_group)
{
  send_receive sr;

  sr.set_send([](ib::reliable_server &s){
    
    ib::image_group g;
    g.set_id("my frame");
    g.add_image(ib::image(1, 5, sizeof(int)), "image one");
    g.add_image(ib::image(1, 5, sizeof(int)), "image two");
    g.add_image(ib::image(1, 5, sizeof(int)), "image three");

    for (int i = 0; i < g.get_images()[0].get_height(); ++i) { g.get_images()[0].ptr<int>()[i] = i; }
    for (int i = 0; i < g.get_images()[1].get_height(); ++i) { g.get_images()[1].ptr<int>()[i] = i; }
    for (int i = 0; i < g.get_images()[2].get_height(); ++i) { g.get_images()[2].ptr<int>()[i] = i; }

    BOOST_REQUIRE(s.publish(g));
  });

  sr.set_recv([](ib::reliable_client &c) {
    
    ib::image_group g;

    BOOST_REQUIRE(c.receive(g));
    BOOST_REQUIRE_EQUAL(std::string("my frame"), g.get_id());
    BOOST_REQUIRE_EQUAL(3, g.get_images().size());
    BOOST_REQUIRE_EQUAL(3, g.get_names().size());
    BOOST_REQUIRE_EQUAL(3, g.size());

    BOOST_REQUIRE_EQUAL(std::string("image one"), g.get_names()[0]);
    BOOST_REQUIRE_EQUAL(std::string("image two"), g.get_names()[1]);
    BOOST_REQUIRE_EQUAL(std::string("image three"), g.get_names()[2]);

    BOOST_REQUIRE_EQUAL(5, g.get_images()[0].get_height());
    BOOST_REQUIRE_EQUAL(5, g.get_images()[1].get_height());
    BOOST_REQUIRE_EQUAL(5, g.get_images()[2].get_height());

    for (int i = 0; i < g.get_images()[0].get_height(); ++i) { BOOST_REQUIRE_EQUAL(i, g.get_images()[0].ptr<int>()[i]); }
    for (int i = 0; i < g.get_images()[1].get_height(); ++i) { BOOST_REQUIRE_EQUAL(i, g.get_images()[1].ptr<int>()[i]); }
    for (int i = 0; i < g.get_images()[2].get_height(); ++i) { BOOST_REQUIRE_EQUAL(i, g.get_images()[2].ptr<int>()[i]); }

  });

  BOOST_REQUIRE(sr.run());
}

BOOST_AUTO_TEST_CASE(send_receive_image_group_many_times)
{
  send_receive sr;

  sr.set_send([](ib::reliable_server &s){
    
    ib::image_group g;
    g.set_id("content");
    g.add_image(ib::image(1, 5, sizeof(int)), "image one");    
    for (int i = 0; i < g.get_images()[0].get_height(); ++i) { g.get_images()[0].ptr<int>()[i] = i; }
    
    BOOST_REQUIRE(s.publish(g));
    BOOST_REQUIRE(s.publish(g));
    BOOST_REQUIRE(s.publish(g));
    BOOST_REQUIRE(s.publish(g));
    BOOST_REQUIRE(s.publish(g));
    g.set_id("end");
    BOOST_REQUIRE(s.publish(g));
  });

  sr.set_recv([](ib::reliable_client &c) {
    
    ib::image_group g;

    bool cont = true;
    do {
      BOOST_REQUIRE(c.receive(g));
      cont = (g.get_id() == "content");

      BOOST_REQUIRE_EQUAL(1, g.get_images().size());
      BOOST_REQUIRE_EQUAL(std::string("image one"), g.get_names()[0]);
      BOOST_REQUIRE_EQUAL(5, g.get_images()[0].get_height());
      for (int i = 0; i < g.get_images()[0].get_height(); ++i) { BOOST_REQUIRE_EQUAL(i, g.get_images()[0].ptr<int>()[i]); }

    } while (cont);
  });

  BOOST_REQUIRE(sr.run());
}

BOOST_AUTO_TEST_CASE(receive_into_existing_memory)
{
  send_receive sr;

  sr.set_send([](ib::reliable_server &s){
    
    ib::image_group g;
    g.add_image(ib::image(1, 5, sizeof(int)), "image one");
    
    for (int i = 0; i < g.get_images()[0].get_height(); ++i) { g.get_images()[0].ptr<int>()[i] = i; }
    
    BOOST_REQUIRE(s.publish(g));
  });

  sr.set_recv([](ib::reliable_client &c) {
    
    int arr[5];

    ib::image_group g;
    g.add_image(ib::image(1, 5, sizeof(int), arr, ib::share_mem()));

    BOOST_REQUIRE(c.receive(g));
    BOOST_REQUIRE_EQUAL(1, g.get_images().size());
    
    BOOST_REQUIRE_EQUAL(std::string("image one"), g.get_names()[0]);
    BOOST_REQUIRE_EQUAL(5, g.get_images()[0].get_height());
    for (int i = 0; i < g.get_images()[0].get_height(); ++i) { BOOST_REQUIRE_EQUAL(i, g.get_images()[0].ptr<int>()[i]); }

    BOOST_REQUIRE_EQUAL(arr, g.get_images()[0].ptr<int>());

  });

  BOOST_REQUIRE(sr.run());
}

BOOST_AUTO_TEST_SUITE_END()
