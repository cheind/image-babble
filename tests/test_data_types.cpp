/*! \file test_data_types.cpp

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

BOOST_AUTO_TEST_CASE(data_types)
{
  boost::thread_group g;

  // Server part
  g.create_thread([]() {

    ib::reliable_server s;
    s.startup();

    BOOST_REQUIRE(s.publish(int(-1)));
    BOOST_REQUIRE(s.publish(float(1.2)));
    BOOST_REQUIRE(s.publish(double(1.3)));
    BOOST_REQUIRE(s.publish(std::string("hello world")));
    BOOST_REQUIRE(s.publish(ib::io::empty()));

    std::vector<int> e;
    e.push_back(0); e.push_back(1); e.push_back(2);
    BOOST_REQUIRE(s.publish(e));

    s.shutdown();

  });

  // Client part
  g.create_thread([]() {

    ib::reliable_client c;
    c.startup();

    int v0;
    float v1;
    double v2;
    std::string v3;
    ib::io::empty v4;
    std::vector<int> v5;

    BOOST_REQUIRE(c.receive(v0)); BOOST_REQUIRE_EQUAL(-1, v0);
    BOOST_REQUIRE(c.receive(v1)); BOOST_REQUIRE_EQUAL(float(1.2), v1);
    BOOST_REQUIRE(c.receive(v2)); BOOST_REQUIRE_EQUAL(double(1.3), v2);
    BOOST_REQUIRE(c.receive(v3)); BOOST_REQUIRE_EQUAL(std::string("hello world"), v3);
    BOOST_REQUIRE(c.receive(v4));
    BOOST_REQUIRE(c.receive(v5)); BOOST_REQUIRE(v5.size() == 3 && v5[0] == 0 && v5[1] == 1 && v5[2] == 2);
    
    c.shutdown();

  });

  g.join_all();
}
BOOST_AUTO_TEST_SUITE_END()
