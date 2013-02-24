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
#include <boost/thread/thread.hpp>

BOOST_AUTO_TEST_SUITE(test_image_support)

namespace ib = imagebabble;

const int any_type = 0;

inline void free_fn_count(void *data, void *hint)
{
  int *i = static_cast<int*>(hint);
  (*i) += 1;
}

BOOST_AUTO_TEST_CASE(image)
{
  {
    ib::image i;
    BOOST_REQUIRE_EQUAL(0, i.get_width());
    BOOST_REQUIRE_EQUAL(0, i.get_height());
    BOOST_REQUIRE_EQUAL(0, i.get_step());
    BOOST_REQUIRE_EQUAL(-1, i.get_external_type());
    BOOST_REQUIRE_EQUAL(ib::image::FORMAT_UNKNOWN, i.get_format());
    BOOST_REQUIRE_EQUAL(0, i.size());    
  }

  {
    ib::image i(640, 480, 640*3);
    i.set_external_type(any_type);
    i.set_format(ib::image::FORMAT_RGB_888);

    BOOST_REQUIRE_EQUAL(640, i.get_width());
    BOOST_REQUIRE_EQUAL(480, i.get_height());
    BOOST_REQUIRE_EQUAL(640*3, i.get_step());
    BOOST_REQUIRE_EQUAL(any_type, i.get_external_type());
    BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, i.get_format());
    BOOST_REQUIRE_EQUAL(640*480*3, i.size());    
  }

  {
    int x = 10;
    ib::image i(1, 1, sizeof(int)*1, &x, ib::share_mem());
    i.set_external_type(any_type);
    i.set_format(ib::image::FORMAT_RGB_888);

    BOOST_REQUIRE_EQUAL(1, i.get_width());
    BOOST_REQUIRE_EQUAL(1, i.get_height());
    BOOST_REQUIRE_EQUAL(sizeof(int), i.get_step());
    BOOST_REQUIRE_EQUAL(sizeof(int), i.size());    
    BOOST_REQUIRE_EQUAL(any_type, i.get_external_type());
    BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, i.get_format());
    BOOST_REQUIRE_EQUAL(&x, i.ptr<int>());    
  }

  {
    int x = 10;
    ib::image i(1, 1, sizeof(int)*1,&x, ib::copy_mem());
    i.set_external_type(any_type);
    i.set_format(ib::image::FORMAT_RGB_888);

    BOOST_REQUIRE_EQUAL(1, i.get_width());
    BOOST_REQUIRE_EQUAL(1, i.get_height());
    BOOST_REQUIRE_EQUAL(sizeof(int), i.get_step());
    BOOST_REQUIRE_EQUAL(sizeof(int), i.size());    
    BOOST_REQUIRE_EQUAL(any_type, i.get_external_type());
    BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, i.get_format());
    BOOST_REQUIRE_NE(&x, i.ptr<int>());    
  }

  {
    int x = 10;
    int count_released = 0;
    
    {
      ib::share_mem s(free_fn_count, &count_released);
      ib::image i(1, 1, sizeof(int)*1, &x, s);
      i.set_external_type(any_type);
      i.set_format(ib::image::FORMAT_RGB_888);
      ib::image i2 = i;
    }

    BOOST_REQUIRE_EQUAL(1, count_released);


  }

#ifdef IB_HAS_RVALUE_REFS
   {

      int x = 10;
      ib::image i0(1, 1, sizeof(int),&x, ib::share_mem());
      i0.set_external_type(any_type);
      i0.set_format(ib::image::FORMAT_RGB_888);
      ib::image i1(std::move(i0));

      BOOST_REQUIRE_EQUAL(1, i1.get_width());
      BOOST_REQUIRE_EQUAL(1, i1.get_height());
      BOOST_REQUIRE_EQUAL(sizeof(int), i1.get_step());
      BOOST_REQUIRE_EQUAL(any_type, i1.get_external_type());
      BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, i1.get_format());
      BOOST_REQUIRE_EQUAL(sizeof(int), i1.size());    
      BOOST_REQUIRE_EQUAL(&x, i1.ptr<int>());    

      ib::image i2 = std::move(i1);

      BOOST_REQUIRE_EQUAL(1, i2.get_width());
      BOOST_REQUIRE_EQUAL(1, i2.get_height());
      BOOST_REQUIRE_EQUAL(sizeof(int), i2.get_step());
      BOOST_REQUIRE_EQUAL(any_type, i2.get_external_type());
      BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, i2.get_format());
      BOOST_REQUIRE_EQUAL(sizeof(int), i2.size());    
      BOOST_REQUIRE_EQUAL(&x, i2.ptr<int>());   

      i2 = std::move(i2);
    
      BOOST_REQUIRE_EQUAL(1, i2.get_width());
      BOOST_REQUIRE_EQUAL(1, i2.get_height());
      BOOST_REQUIRE_EQUAL(sizeof(int), i2.get_step());
      BOOST_REQUIRE_EQUAL(any_type, i2.get_external_type());
      BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, i2.get_format());
      BOOST_REQUIRE_EQUAL(sizeof(int), i2.size());    
      BOOST_REQUIRE_EQUAL(&x, i2.ptr<int>());

      /* I believe zeromq does this is by design. It copies the data if the message size is 
         small and reference counts it if its above a certain size. It looks like the size 
         threshold is specified by the max_vsm_size enum in msg.hpp, currently set to 29, so if 
         you make your message size 30 it should switch to reference counting. */

      const int zmq_min_size_ref_count = 30;
      const int elems = (zmq_min_size_ref_count / sizeof(int)) + 1;

      // should ref-count

      i0 = ib::image(1, elems, sizeof(int));
      i0.set_external_type(any_type);
      i0.set_format(ib::image::FORMAT_RGB_888);
      int *adr = i0.ptr<int>();
      i1 = std::move(i0);

      BOOST_REQUIRE_EQUAL(1, i1.get_width());
      BOOST_REQUIRE_EQUAL(elems, i1.get_height());
      BOOST_REQUIRE_EQUAL(sizeof(int), i1.get_step());
      BOOST_REQUIRE_EQUAL(any_type, i1.get_external_type());
      BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, i1.get_format());
      BOOST_REQUIRE_EQUAL(elems * sizeof(int), i1.size());    
      BOOST_REQUIRE_EQUAL(adr, i1.ptr<int>());

      // should not ref count

      i0 = ib::image(1, elems - 1, sizeof(int));
      i0.set_external_type(any_type);
      i0.set_format(ib::image::FORMAT_RGB_888);
      adr = i0.ptr<int>();
      i1 = std::move(i0);
      BOOST_REQUIRE_NE(adr, i1.ptr<int>());

    }
#endif
}

void server_image_fnc() 
{
  ib::reliable_server< ib::image > s;
  s.startup();

  ib::image img(1, 5, sizeof(int));
  img.set_external_type(any_type);
  img.set_format(ib::image::FORMAT_RGB_888);
  for (int i = 0; i < img.get_height(); ++i) {
    img.ptr<int>()[i] = i;
  } 

  BOOST_REQUIRE(s.publish(img));

  s.shutdown();
}

void client_image_fnc()
{
  ib::reliable_client< ib::image > c;
  c.startup();

  ib::image img;
  BOOST_REQUIRE(c.receive(img));
  BOOST_REQUIRE_EQUAL(1, img.get_width());
  BOOST_REQUIRE_EQUAL(5, img.get_height());
  BOOST_REQUIRE_EQUAL(sizeof(int), img.get_step());
  BOOST_REQUIRE_EQUAL(any_type, img.get_external_type());
  BOOST_REQUIRE_EQUAL(ib::image::FORMAT_RGB_888, img.get_format());
  BOOST_REQUIRE_EQUAL(0, img.ptr<int>()[0]);
  BOOST_REQUIRE_EQUAL(1, img.ptr<int>()[1]);
  BOOST_REQUIRE_EQUAL(2, img.ptr<int>()[2]);
  BOOST_REQUIRE_EQUAL(3, img.ptr<int>()[3]);
  BOOST_REQUIRE_EQUAL(4, img.ptr<int>()[4]);    

  c.shutdown();
}

BOOST_AUTO_TEST_CASE(send_receive_image)
{
  boost::thread_group g;

  g.create_thread(server_image_fnc);
  g.create_thread(client_image_fnc);
  g.join_all();
}

void server_image_group_fnc() 
{
  ib::reliable_server< ib::image_group > s;
  s.startup();

  ib::image_group g;
  g.set_id("my frame");
  g.add_image(ib::image(1, 5, sizeof(int)), "image one");
  g.add_image(ib::image(1, 5, sizeof(int)), "image two");
  g.add_image(ib::image(1, 5, sizeof(int)), "image three");

  for (int i = 0; i < g.get_images()[0].get_height(); ++i) { g.get_images()[0].ptr<int>()[i] = i; }
  for (int i = 0; i < g.get_images()[1].get_height(); ++i) { g.get_images()[1].ptr<int>()[i] = i; }
  for (int i = 0; i < g.get_images()[2].get_height(); ++i) { g.get_images()[2].ptr<int>()[i] = i; }

  BOOST_REQUIRE(s.publish(g));

  s.shutdown();
}

void client_image_group_fnc()
{
  ib::reliable_client< ib::image_group > c;
  c.startup();

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

  c.shutdown();
}

BOOST_AUTO_TEST_CASE(send_receive_image_group)
{
  boost::thread_group g;

  g.create_thread(server_image_group_fnc);
  g.create_thread(client_image_group_fnc);
  g.join_all();
}

void server_image_group_many_fnc() 
{
  ib::reliable_server< ib::image_group > s;
  s.startup();

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

  s.shutdown();
}

void client_image_group_many_fnc()
{
  ib::reliable_client< ib::image_group > c;
  c.startup();

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

  c.shutdown();
}


BOOST_AUTO_TEST_CASE(send_receive_image_group_many_times)
{
  boost::thread_group g;

  g.create_thread(server_image_group_many_fnc);
  g.create_thread(client_image_group_many_fnc);
  g.join_all();
}

void server_image_group_preexisting_fnc() 
{
  ib::reliable_server< ib::image_group > s;
  s.startup();

  ib::image_group g;
  g.add_image(ib::image(1, 5, sizeof(int)), "image one");
    
  for (int i = 0; i < g.get_images()[0].get_height(); ++i) { g.get_images()[0].ptr<int>()[i] = i; }
    
  BOOST_REQUIRE(s.publish(g));

  s.shutdown();
}

void client_image_group_preexisting_fnc()
{
  ib::reliable_client< ib::image_group > c;
  c.startup();

  int arr[5];

  ib::image_group g;
  g.add_image(ib::image(1, 5, sizeof(int), arr, ib::share_mem()));

  BOOST_REQUIRE(c.receive(g));
  BOOST_REQUIRE_EQUAL(1, g.get_images().size());
    
  BOOST_REQUIRE_EQUAL(std::string("image one"), g.get_names()[0]);
  BOOST_REQUIRE_EQUAL(5, g.get_images()[0].get_height());
  for (int i = 0; i < g.get_images()[0].get_height(); ++i) { BOOST_REQUIRE_EQUAL(i, g.get_images()[0].ptr<int>()[i]); }

  BOOST_REQUIRE_EQUAL(arr, g.get_images()[0].ptr<int>());

  c.shutdown();
}

BOOST_AUTO_TEST_CASE(receive_into_existing_memory)
{
  boost::thread_group g;

  g.create_thread(server_image_group_preexisting_fnc);
  g.create_thread(client_image_group_preexisting_fnc);
  g.join_all();
}

BOOST_AUTO_TEST_SUITE_END()
