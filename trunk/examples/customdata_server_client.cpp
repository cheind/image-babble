/*! \file customdata_server_client.cpp
    \example customdata_server_client.cpp

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

/** [Include Statement] */
#include <imagebabble/imagebabble.hpp>
/** [Include Statement] */

#include <iostream>
#include <vector>
#include <string>
#include <iterator>

/** [Data] */
struct person {
  std::string name;
  int age;
  std::vector< std::string > friends;
};
/** [Data] */

/** [Serialization] */
namespace imagebabble {
  namespace io {

    /** Send a person */
    bool send(zmq::socket_t &s, const person &p, int flags = 0)
    {
      bool all_ok = true;

      all_ok &= send(s, p.name, ZMQ_SNDMORE);
      all_ok &= send(s, p.age, ZMQ_SNDMORE);
      all_ok &= send(s, p.friends, flags);

      return all_ok;
    }

    /** Receive a person */
    bool recv(zmq::socket_t &s, person &p)
    {
      IB_STOP_RECV_UNLESS(recv(s, p.name), s);
      IB_STOP_RECV_UNLESS(recv(s, p.age), s);
      IB_STOP_RECV_UNLESS(recv(s, p.friends), s);

      return true;
    }

  }
}
/** [Serialization] */

/** [Example] */
int main(int argc, char *argv[]) 
{
  namespace ib = imagebabble;

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <server|client>" << std::endl;
    std::cerr << "Example: " << argv[0] << " server" << std::endl;
    return -1;
  }

  if (std::string(argv[1]) == "server") {
    /** [Server] */
    ib::reliable_server s;
    s.startup();
    
    person p;
    p.name = "Foo Bar";
    p.age = 26;
    p.friends.push_back("Mike");
    p.friends.push_back("Freddy");

    return s.publish(p) ? 0 : -1;
    /** [Server] */
  } else {
    /** [Client] */
    ib::reliable_client c;
    c.startup();

    person p;
    if (c.receive(p, 5000)) {

      std::cout << "Received Person" << std::endl;
      std::cout << " Name " << p.name << std::endl;
      std::cout << " Age " << p.age << std::endl;
      std::cout << " Friends "; 
      std::copy(
        p.friends.begin(), 
        p.friends.end(), 
        std::ostream_iterator<std::string>(std::cout, ","));
      std::cout << std::endl;

      return 0;
    } else {
      return -1;
    }
    /** [Client] */
  }
}
/** [Example] */