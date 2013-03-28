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

BOOST_AUTO_TEST_SUITE(test_datatypes)

namespace ib = imagebabble;


template<class A>
void require_equal(const A &lhs, const A &rhs) {
  BOOST_REQUIRE(lhs == rhs);
}

void require_equal(const ib::io::empty &lhs, const ib::io::empty &rhs) 
{}


template<class T>
inline void publish_t(const T &t) {
  ib::reliable_server<T> s; 
  s.startup(); 
  BOOST_REQUIRE(s.publish(t));
  s.shutdown();
}

template<class T>
inline void receive_t(const T &should_be, bool should_parse = true) {
  ib::reliable_client<T> c; 
  c.startup(); 
  c.send_request();
  T t;
  if (should_parse) {
    BOOST_REQUIRE(c.receive(t));
    require_equal(t, should_be);    
  } else {
    BOOST_REQUIRE_THROW(c.receive(t), ib::ib_error);
  }
  c.shutdown();
}

void server_fnc() 
{
  publish_t(int(-1));
  publish_t(float(1.2));
  publish_t(double(1.3));
  publish_t(std::string("hello world"));
  publish_t(ib::io::empty());

  std::vector<int> e;
  e.push_back(0); e.push_back(1); e.push_back(2);
  publish_t(e);

  publish_t(ib::io::empty());
}

void client_fnc()
{
  receive_t(int(-1));
  receive_t(float(1.2));
  receive_t(double(1.3));
  receive_t(std::string("hello world"));
  receive_t(ib::io::empty());

  std::vector<int> e;
  e.push_back(0); e.push_back(1); e.push_back(2);
  receive_t(e);

  receive_t((int)(0), false);
}

void server_incompatible_fnc() 
{
  ib::reliable_server< std::string > s; 
  s.startup(); 
  s.publish("hello");
  s.publish("wolrd");
  s.shutdown();
}

void client_incompatible_fnc() 
{
  ib::reliable_client< int > s; 
  s.startup();

  int i;
  s.send_request();
  BOOST_REQUIRE_THROW(s.receive(i), ib::ib_error);
  
  s.send_request();
  BOOST_REQUIRE_THROW(s.receive(i), ib::ib_error);
}


BOOST_AUTO_TEST_CASE(builtin)
{
  boost::thread_group g;
  g.create_thread(server_fnc);
  g.create_thread(client_fnc);
  g.join_all();
}

BOOST_AUTO_TEST_CASE(incompatible_types)
{
  boost::thread_group g;
  g.create_thread(server_incompatible_fnc);
  g.create_thread(client_incompatible_fnc);
  g.join_all();
}

BOOST_AUTO_TEST_SUITE_END()
