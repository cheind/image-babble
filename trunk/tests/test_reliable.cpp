/*! \file test_reliable.cpp

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

BOOST_AUTO_TEST_SUITE(test_core)

namespace ib = imagebabble;

BOOST_AUTO_TEST_CASE(one_client)
{
  int sum_sent = 0;
  int sum_received = 0;

  boost::thread_group g;

  // Server part
  g.create_thread([&sum_sent]() {

    ib::reliable_server s;
    s.startup();

    for (int i = 0; i < 1000; ++i) {
      if (s.publish(i))
        sum_sent += i;
    }

    s.publish(-1);

    s.shutdown();

  });

  // Client part
  g.create_thread([&sum_received]() {

    ib::reliable_client c;
    c.startup();

    int j = -1;

    while (c.receive(j) && j >= 0) {
      sum_received += j;
    }

    c.shutdown();

  });

  g.join_all();

  BOOST_REQUIRE(sum_sent > 0 && sum_sent == sum_received);
}

BOOST_AUTO_TEST_CASE(multiple_clients)
{
  const size_t nclients = 5;

  int sum_sent = 0;
  int sum_received[nclients];

  boost::thread_group g;

  // Server part
  g.create_thread([&]() {

    ib::reliable_server s;
    s.startup();

    for (int i = 0; i < 1000; ++i) {
      if (s.publish(i, -1, nclients))
        sum_sent += i;
    }

    s.publish(-1, -1, nclients);

    s.shutdown();

  });

  // Client part
  for (size_t i = 0; i < nclients; ++i) {
    g.create_thread([i, &sum_received]() {

      ib::reliable_client c;
      c.startup();

      int j = -1;

      sum_received[i] = 0;
      while (c.receive(j) && j >= 0) {
        sum_received[i] += j;
      }

      c.shutdown();

    });
  }

  g.join_all();

  BOOST_REQUIRE(sum_sent > 0);
  for (size_t i = 0; i < nclients; ++i) {
    BOOST_REQUIRE_EQUAL(sum_sent, sum_received[i]);
  }
}

BOOST_AUTO_TEST_CASE(no_clients)
{
  int sum_sent = 0;

  boost::thread_group g;

  // Server part
  g.create_thread([&sum_sent]() {

    ib::reliable_server s;
    s.startup();

    ib::stopwatch sw;
    if (s.publish(1, 2000, 1))
      sum_sent += 1;

    BOOST_CHECK_GE(sw.elapsed_msecs(), 2000u);

    s.shutdown();

  });

  g.join_all();

  BOOST_REQUIRE(sum_sent == 0);
}

BOOST_AUTO_TEST_CASE(disconnected_client)
{
  size_t nclients = 1;

  int sum_sent = 0;
  int sum_received = 0;

  boost::thread_group g;

  // Server part
  g.create_thread([&sum_sent]() {

    ib::reliable_server s;
    s.startup();

    if (s.publish(1)) sum_sent += 1;
    if (s.publish(2)) sum_sent += 2;

    s.shutdown();

  });

  g.create_thread([&sum_received]() {

    int j = -1;

    ib::reliable_client c;    

    c.startup();
    c.receive(j); sum_received += j;
    c.shutdown();

    c.startup();
    c.receive(j); sum_received += j;
    c.shutdown();

  });

  g.join_all();

  BOOST_REQUIRE(sum_sent > 0 && sum_sent == sum_received);  
}

BOOST_AUTO_TEST_CASE(missing_client)
{
  size_t nclients = 2;

  int sum_sent = 0;
  int sum_received[2] = {0, 0};

  boost::thread_group g;

  // Server part
  g.create_thread([&sum_sent, nclients]() {

    ib::reliable_server s;
    s.startup();

    if (s.publish(1, -1, nclients)) sum_sent += 1;
    if (s.publish(2, 2000, nclients)) sum_sent += 2;
    BOOST_REQUIRE(s.has_error());
    BOOST_REQUIRE_EQUAL(s.get_last_error().get_reason(), ib::ib_error::ETIMEOUT); 

    s.shutdown();

  });

  g.create_thread([&sum_received]() {

    int j = -1;

    ib::reliable_client c;    

    c.startup();
    if (c.receive(j, -1)) sum_received[0] += j;
    if (c.receive(j, 2000)) sum_received[0] += j;
    c.shutdown();    

    BOOST_REQUIRE(c.has_error());
    BOOST_REQUIRE_EQUAL(ib::ib_error::ETIMEOUT, c.get_last_error().get_reason());
  });

  g.create_thread([&sum_received]() {

    int j = -1;

    ib::reliable_client c;    

    c.startup();
    // Just receive once and disconnect
    if (c.receive(j, -1)) sum_received[1] += j;
    c.shutdown();    

    BOOST_REQUIRE(!c.has_error());
  });

  g.join_all();

  BOOST_REQUIRE(sum_sent == 1 && sum_received[0] == 1 && sum_received[1] == 1);  
}

BOOST_AUTO_TEST_CASE(missing_server)
{
  
  boost::thread_group g;
  
  g.create_thread([]() {

    int j;
    ib::reliable_client c;    
    BOOST_REQUIRE(c.startup());
    BOOST_REQUIRE(!c.receive(j, 2000));
    BOOST_REQUIRE(c.has_error());
    BOOST_REQUIRE_EQUAL(ib::ib_error::ETIMEOUT, c.get_last_error().get_reason());
    BOOST_REQUIRE(c.shutdown());    
  });

  g.join_all();

}

BOOST_AUTO_TEST_SUITE_END()
