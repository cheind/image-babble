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

BOOST_AUTO_TEST_SUITE(test_reliable)

namespace ib = imagebabble;


void server_fnc(int times, size_t nclients, int timeout, int &count) 
{
  ib::reliable_server<int> s;
  s.startup();

  count = 0;
  for (int i = 0; i < times; ++i) {
    if (s.publish(1, timeout, nclients))
      count += 1;
  }

  s.publish(-1, 1000, nclients);
  s.shutdown();
}

void client_fnc(int timeout, int &count)
{
  ib::reliable_client<int> c;
  c.startup();

  int j = -1;

  count = 0;
  while (c.receive(j, timeout) && j >= 0) {
    count += j;
  }

  c.shutdown();
}

void client_disconnect_fnc(int &count)
{
  ib::reliable_client<int> c;    

  count = 0;
  int j;
  c.startup();
  c.receive(j); count += j;
  c.shutdown();

  c.startup();
  c.receive(j); count += j;
  c.shutdown();
}

void client_once_fnc(int &count)
{
  ib::reliable_client<int> c;    

  count = 0;
  int j;
  c.startup();
  c.receive(j); count += j;
  c.shutdown();
}

BOOST_AUTO_TEST_CASE(one_client)
{
  
  int sum_sent;
  int sum_received;

  boost::thread_group g;

  g.create_thread(boost::bind(server_fnc, 1000, 1, -1, boost::ref(sum_sent)));
  g.create_thread(boost::bind(client_fnc, -1, boost::ref(sum_received)));
  g.join_all();

  BOOST_REQUIRE(sum_sent > 0 && sum_sent == sum_received);
}

BOOST_AUTO_TEST_CASE(multiple_clients)
{
  const size_t nclients = 5;

  int sum_sent = 0;
  int sum_received[nclients];

  boost::thread_group g;
  g.create_thread(boost::bind(server_fnc, 1000, nclients, -1, boost::ref(sum_sent)));
  for (size_t i = 0; i < nclients; ++i) {
    g.create_thread(boost::bind(client_fnc, -1, boost::ref(sum_received[i])));
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
  g.create_thread(boost::bind(server_fnc, 1, 1, 2000, boost::ref(sum_sent)));
  g.join_all();

  BOOST_REQUIRE(sum_sent == 0);
}

BOOST_AUTO_TEST_CASE(disconnected_client)
{
    
  int sum_sent;
  int sum_received;

  boost::thread_group g;

  g.create_thread(boost::bind(server_fnc, 2, 1, -1, boost::ref(sum_sent)));
  g.create_thread(boost::bind(client_disconnect_fnc, boost::ref(sum_received)));
  g.join_all();

  BOOST_REQUIRE(sum_sent > 0 && sum_sent == sum_received);
}

BOOST_AUTO_TEST_CASE(missing_client)
{
  size_t nclients = 2;

  int sum_sent = 0;
  int sum_received[2] = {0, 0};

  boost::thread_group g;
  g.create_thread(boost::bind(server_fnc, 2, 2, 2000, boost::ref(sum_sent)));
  g.create_thread(boost::bind(client_fnc, 2000, boost::ref(sum_received[0])));
  g.create_thread(boost::bind(client_once_fnc, boost::ref(sum_received[1])));
  g.join_all();

  BOOST_REQUIRE(sum_sent == 1 && sum_received[0] == 1 && sum_received[1] == 1);  
}

BOOST_AUTO_TEST_CASE(missing_server)
{
  int sum_received = 0;
  
  boost::thread_group g;
  g.create_thread(boost::bind(client_fnc, 500, boost::ref(sum_received)));
  g.join_all();
  
  BOOST_REQUIRE(0 == sum_received);

}

BOOST_AUTO_TEST_SUITE_END()
