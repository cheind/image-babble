/*! \file customdata_server_client.cpp
    \example customdata_server_client.cpp
    \brief Shows how to send and receive a custom data type.

    \copyright Copyright (c) 2013, PROFACTOR GmbH, Christoph Heindl
    \license This project is released under the New BSD License.
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
	template<>
    bool send(zmq::socket_t &s, const person &p, int flags)
    {
      IB_FIRST_PART(io::send(s, p.name, flags | ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(s, p.age, flags | ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(s, p.friends, flags));

      return true;
    }

    /** Receive a person */
    template<>
    bool recv(zmq::socket_t &s, person &p, int flags)
    {
      IB_FIRST_PART(io::recv(s, p.name, flags));
      IB_NEXT_PART(io::recv(s, p.age, flags));
      IB_NEXT_PART(io::recv(s, p.friends, flags));

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
    ib::reliable_server< person > s;
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
    ib::reliable_client< person > c;
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
