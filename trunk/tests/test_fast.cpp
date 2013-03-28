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

void server_fnc(int &count) 
{
  ib::fast_server<int> s;
  s.startup();

  zmq::socket_t ctrl(*s.get_context(), ZMQ_REP);
  ctrl.connect("tcp://127.0.0.1:6001");
  
  boost::this_thread::sleep(boost::posix_time::seconds(1));

  count = 0;
  while (!ib::io::is_data_pending(ctrl, 0)) {
    if (s.publish(1)) {
      count += 1;
    }
  }
  ib::io::recv(ctrl, ib::io::drop(), 0);

  s.shutdown();
}

void client_fnc(int &count)
{
  ib::fast_client<int> c;  
  c.startup();

  zmq::socket_t ctrl(*c.get_context(), ZMQ_REQ);
  int linger = 0;
  ctrl.setsockopt(ZMQ_LINGER, &linger, sizeof(int));
  ctrl.bind("tcp://127.0.0.1:6001");

  boost::this_thread::sleep(boost::posix_time::seconds(1));

  int j;
  count = 0;
  while (count < 10) {
    c.send_request();
    if (c.receive(j, -1)) {
      count += 1;
    }
  }

  ib::io::send(ctrl, ib::io::empty(), 0);

  c.shutdown();
}


BOOST_AUTO_TEST_CASE(single_client)
{
  int sum_sent = 0;
  int sum_received = 0;

  boost::thread_group g;

  g.create_thread(boost::bind(server_fnc, boost::ref(sum_sent)));
  g.create_thread(boost::bind(client_fnc, boost::ref(sum_received)));
  g.join_all();

  BOOST_REQUIRE_LE(10, sum_sent);
  BOOST_REQUIRE_EQUAL(10, sum_received);  
}

BOOST_AUTO_TEST_SUITE_END()
