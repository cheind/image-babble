/*! \file test_fast.cpp

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
#include <boost/thread.hpp>

BOOST_AUTO_TEST_SUITE(test_fast)

namespace ib = imagebabble;

BOOST_AUTO_TEST_CASE(no_client)
{
  // Test non-blocking behaviour

  boost::thread_group g;

  // Server part
  g.create_thread([]() {

    ib::fast_server s;
    s.startup();

    int unused = 0;
    BOOST_REQUIRE(s.publish(unused));
  });

  g.join_all();
}

BOOST_AUTO_TEST_CASE(single_client)
{
  int sum_sent = 0;
  int sum_received = 0;

  bool client_ready = false;
  boost::condition_variable cv;
  boost::mutex m;

  boost::thread_group g;

  // Server part
  g.create_thread([&]() {

    ib::fast_server s;
    s.startup();

    {
      boost::mutex::scoped_lock l(m);
      while (!client_ready) {
        cv.wait(l);
      }
    }

    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

    for (int i = 0; i < 1000; ++i) {
      if (s.publish(1))
        sum_sent += 1;
    }
    
    s.publish(-1);

    s.shutdown();

  });

  // Client part
  g.create_thread([&]() {

    ib::fast_client c;
    c.startup();

    {
      boost::mutex::scoped_lock l(m);
      client_ready = true;
      cv.notify_one();
    }
    
    int j = -1;

    while (c.receive(j, 1000) && j >= 0) {
      sum_received += j;
    }

  });

  g.join_all();

  BOOST_REQUIRE(sum_sent == 1000 && sum_received == 1000);  
}

BOOST_AUTO_TEST_SUITE_END()
