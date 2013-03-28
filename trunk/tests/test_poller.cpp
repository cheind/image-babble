/*! \file test_poller.cpp

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

BOOST_AUTO_TEST_SUITE(test_poller)

namespace ib = imagebabble;

void server_fnc(int times, size_t nclients, int timeout, int &count) 
{
  ib::reliable_server<int> s;
  s.startup();

  count = 0;
  for (int i = 0; i < times; ++i) {
    if (s.publish(1, timeout, nclients)) {
      count += 1;
    }
  }
  s.shutdown();
}

void client_fnc_any(int timeout, bool &success)
{
  success = false;

  ib::reliable_client<int> c[2];
  c[0].startup();
  c[1].startup();

  ib::poller poller;
  poller.add(c[0]);
  poller.add(c[1]); 

  c[0].send_request();
  c[1].send_request();

  if (!poller.poll_any(timeout, ZMQ_POLLIN))
    return;
  
  int j;
  if (poller.is_readable(0)) {
    c[0].receive(j, 0);
    success = true;
  }

  if (poller.is_readable(1)) {
    c[1].receive(j, 0);
    success = true;
  }

  // Server only sends 1 packet.
  success &= !poller.poll_any(timeout, ZMQ_POLLIN);
}

void client_fnc_all(int timeout, bool &success)
{
  success = false;

  ib::reliable_client<int> c[2];
  c[0].startup();
  c[1].startup();

  ib::poller poller;
  poller.add(c[0]);
  poller.add(c[1]); 

  c[0].send_request();
  c[1].send_request();

  if (!poller.poll_all(timeout, ZMQ_POLLIN))
    return;

  success = poller.is_readable(0) && poller.is_readable(1);
}

BOOST_AUTO_TEST_CASE(poll_any)
{
  
  int sum_sent;
  bool success;

  boost::thread_group g;

  g.create_thread(boost::bind(server_fnc, 1, 1, -1, boost::ref(sum_sent)));
  g.create_thread(boost::bind(client_fnc_any, 1000, boost::ref(success)));
  g.join_all();

  BOOST_REQUIRE(sum_sent > 0 && success);
}

BOOST_AUTO_TEST_CASE(poll_all)
{
  
  int sum_sent;
  bool success;

  boost::thread_group g;

  g.create_thread(boost::bind(server_fnc, 1, 2, -1, boost::ref(sum_sent)));
  g.create_thread(boost::bind(client_fnc_all, 1000, boost::ref(success)));
  g.join_all();

  BOOST_REQUIRE(sum_sent > 0 && success);
}

BOOST_AUTO_TEST_SUITE_END()
