/*! \file core.hpp

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

#ifndef __IMAGE_BABBLE_CORE_HPP_INCLUDED__
#define __IMAGE_BABBLE_CORE_HPP_INCLUDED__

#include "zmq.hpp"
#include <zmq_utils.h>
#include <memory>
#include <streambuf>
#include <sstream>
#include <string>
#include <limits>
#include <vector>
#include <hash_set>
#include <ctime>

#define IMAGEBABBLE_HAS_RVALUE_REFS ZMQ_HAS_RVALUE_REFS
#define IMAGEBABBLE_EXCHANGE_PROTO_VERSION "1"

namespace imagebabble {

  /** Reference counted pointer to a ZMQ socket. */
  typedef std::shared_ptr<zmq::socket_t> socket_ptr;
  /** Reference counted pointer to a ZMQ context. */
  typedef std::shared_ptr<zmq::context_t> context_ptr;

  /** Base class for objects communicating on networks */
  class network_entity {
  public:

    /** Construct from context */
    network_entity(const context_ptr &c)
      :_ctx(c)
    {}

    /** Basic destructor */
    virtual ~network_entity()
    {}

  protected:    
    context_ptr _ctx;
    socket_ptr _s;

  private:
    /** Disabled copy constructor */
    network_entity (const network_entity &);
    /** Disabled assignment operator */
    network_entity &operator = (const network_entity &);
  };

  /** Base class for fast/reliable servers. */
  class basic_server : public network_entity {
  public:
    
    /** Construct from context */
    basic_server(const context_ptr &c)
      : network_entity(c)
    {}    

    /** Get bound address */
    virtual const std::string &get_address() const = 0;

  };

  /** Base class for fast/reliable clients. */
  class basic_client : public network_entity {
  public:
    
    /** Construct from context */
    basic_client(const context_ptr &c)
      : network_entity(c)
    {} 

  };

  /** Simple stopwatch implementation. */
  class stopwatch {
  public:
    /** Construct with start point */
    inline stopwatch()
      : _handle(zmq_stopwatch_start()), _elapsed(0)
    {}

    inline ~stopwatch()
    {
      zmq_stopwatch_stop(_handle);
    }

    /** Get elapsed time since construction in milli-seconds */
    inline unsigned long elapsed_msecs() {
      _elapsed += zmq_stopwatch_stop(_handle);      
      _handle = zmq_stopwatch_start();      
      return _elapsed / 1000;
    }

  private:
    void *_handle;    
    unsigned long _elapsed;
  };

  /** Convenience send and receive functions for use with ZMQ */
  namespace io {
    
    /** Empty message to be sent */
    struct empty {};

    /** Drop message */
    typedef empty drop;

    /** Simple in-memory stream buffer. Allows to adapt an istream on an existing buffer. 
      * This avoids costly copy operations of std::ostringstream and std::istringstream.*/      
    class in_memory_buffer : public std::basic_streambuf<char>
    {
    public:
      inline in_memory_buffer(char* p, size_t n) 
      {
        setg(p, p, p + n);
        setp(p, p + n);
      }
    };

    /** Generic receive method. Tries to receive a value of T from the given socket.
      * T must have stream input semantics. */
    template<class T>
    inline bool recv(zmq::socket_t &s, T &v) 
    {
      zmq::message_t msg;
      if (!s.recv(&msg))
        return false;

      in_memory_buffer mb(static_cast<char*>(msg.data()), msg.size());
      std::istream is(&mb);
      is >> v;    

      return true;
    }

    /** Receive next part of message and discard. */
    inline bool recv(zmq::socket_t &s, drop &v) 
    {
      zmq::message_t msg;
      return s.recv(&msg);
    }

    /** Receive string. */
    inline bool recv(zmq::socket_t &s, std::string &v) 
    {
      zmq::message_t msg;
      if (!s.recv(&msg)) {
        return false;
      } else {
        v.assign(
          static_cast<char*>(msg.data()), 
          static_cast<char*>(msg.data()) + msg.size()); 
      }
      return true;
    }

    /** Receive array of items as vector */
    template<class T>
    inline bool recv(zmq::socket_t &s, std::vector<T> &c)
    {
      bool all_ok = true;

      size_t count;
      all_ok &= io::recv(s, count);
      
      // Note, use resize to allow existing elements to be
      // reused.
      c.resize(count);

      // Receive elements
      for (size_t i = 0; i < count; ++i) {        
        all_ok &= io::recv(s, c[i]);          
      }

      // Array ends with empty element
      io::recv(s, drop());
      
      return all_ok;
    }

    /** Test if data to receive is pending on the socket. */
    inline bool is_data_pending(zmq::socket_t &s, int timeout)
    {
      zmq::pollitem_t items[] = {{ s, 0, ZMQ_POLLIN, 0 }};      
      zmq::poll(&items[0], 1, timeout);
      return (items[0].revents & ZMQ_POLLIN);      
    }

    /** Generic send method. T must have stream output semantics. */
    template<class T>
    inline bool send(zmq::socket_t &s, const T &v, int flags = 0) 
    {
      std::ostringstream ostr;
      ostr << v;

      const std::string &str = ostr.str();
      zmq::message_t msg(str.size());
      if (!str.empty()) {
        memcpy(msg.data(), str.c_str(), str.size());
      }

      return s.send(msg, flags);
    }

    /** Send string. */
    inline bool send(zmq::socket_t &s, const std::string &v, int flags = 0) 
    {
      zmq::message_t msg(v.size());
      if (!v.empty()) {
        memcpy(msg.data(), v.c_str(), v.size());
      }
      return s.send(msg, flags);
    }

    /** Send empty message. */
    inline bool send(zmq::socket_t &s, const empty &v, int flags = 0) 
    {
      zmq::message_t msg(0);
      return s.send(msg, flags);    
    }

    /** Send arra of elements*/
    template<class T>
    inline bool send(zmq::socket_t &s, const std::vector<T> &c, int flags = 0)
    {
      bool all_ok = true;

      const size_t nelems = c.size();
      all_ok &= io::send(s, nelems, ZMQ_SNDMORE);
      for (size_t i = 0; i < nelems; ++i) {
        all_ok &= io::send(s, c[i], ZMQ_SNDMORE);
      }
      
      all_ok &= io::send(s, empty(), flags);
     
      return all_ok;
    }
  }

  /** Fast but unreliable image server implementation. */
  class fast_server : public basic_server {
  public:

    /** Default constructor. */
    fast_server()
      : basic_server(context_ptr(new zmq::context_t(1)))
    {}

    /** Destructor. */
    ~fast_server()
    {
      shutdown();
    }

    /** Start a new connection on the given endpoint. Any previous active connection will be closed. */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {  
      shutdown();
      _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_PUB));
      _s->bind(addr.c_str());
      _addr = addr;
    }
    
    /** Shutdown server. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close();
      }      
    }

    /** Get bound address */
    virtual const std::string &get_address() const {
      return _addr;
    }

    /** Publish data */
    template<class T>
    bool publish(const T &t, int timeout = 0, size_t min_serve = 0)
    {
      return io::send(*_s, t);
    }

  private:
    std::string _addr;
  };

  /** Reliable image server implementation. */
  class reliable_server : public basic_server {
  public:
    /** Default constructor. */
    reliable_server()
      : basic_server(context_ptr(new zmq::context_t(1)))
    {}

    /** Destructor. */
    ~reliable_server()
    {
      shutdown();
    }

    /** Start a new connection on the given endpoint. Any previous active connection will be closed. */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {  
      shutdown();
      _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_ROUTER));
      _s->bind(addr.c_str());
      _addr = addr;
    }
    
    /** Shutdown server. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close();
      }      
    }

    /** Get bound address */
    const std::string &get_address() const {
      return _addr;
    }

    template<class T>
    inline bool publish(const T &t, int timeout = -1, size_t min_serve = 1)
    {
      typedef std::hash_set<std::string> client_set;

      client_set clients;
      stopwatch sw;

      bool continue_wait = true;
      int wait_time = timeout;

      do {
        bool can_read = io::is_data_pending(*_s, wait_time);
        while(can_read) {
          // Receive ready requestes by clients
          std::string address;
          io::recv(*_s, address);
          io::recv(*_s, io::drop());

          clients.insert(address);
          can_read = io::is_data_pending(*_s, 0);
        }
        continue_wait = (timeout == -1 || ((int)sw.elapsed_msecs() < timeout));
      } while ((clients.size() < min_serve) && continue_wait);

      if (clients.size() >= min_serve) {
        // Success, send to all clients
        std::hash_set<std::string>::iterator i = clients.begin();
        std::hash_set<std::string>::iterator i_end = clients.end();

        for (i; i != i_end; ++i) {
          // Construct package for client
          io::send(*_s, *i, ZMQ_SNDMORE);
          io::send(*_s, t);
        }

        return true;
      } else {
        // Failed, send to no clients at all
        return false;
      }
    }

  private:
    std::string _addr;
  };

  /** Fast image client implementation. */
  class fast_client : public basic_client {
  public:
    /** Default constructor */
    fast_client()
      : basic_client(context_ptr(new zmq::context_t(1)))
    {}

    /** Start a new connection to the given endpoint. Previous connections are closed when called multiple times. */      
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_SUB));

        int linger = 0; 
        _s->setsockopt(ZMQ_LINGER, &linger, sizeof(int));
        _s->setsockopt(ZMQ_SUBSCRIBE, 0, 0);      
      }
      _s->connect(addr.c_str());
    }

    /** Shutdown all connections. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close();
      }
    }

    /** Receive a frame. */
    template<class T>
    inline bool receive(T &t, int timeout = 0)
    {
      if (!io::is_data_pending(*_s, timeout)) {
        return false;
      }

      return io::recv(*_s, t);
    }

  };

  /** Reliable client implementation */
  class reliable_client : public basic_client {
  public:

    /** Default constructor */
    reliable_client()
      : basic_client(context_ptr(new zmq::context_t(1)))
    {}

    /** Start a new connection to the given endpoint. Previous connections are closed when called multiple times. */      
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_DEALER));     
      
        int linger = 0; 
        _s->setsockopt(ZMQ_LINGER, &linger, sizeof(int));
      }

      _s->connect(addr.c_str());
    }

    /** Shutdown all connections. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close();
      }
    }

    /** Receive a frame. */
    template<class T>
    bool receive(T &t, int timeout = -1) 
    {
      // Send ready
      io::send(*_s, io::empty());

      // Wait for reply
      if (!io::is_data_pending(*_s, timeout)) {
        return false;
      }

      // Data is here, return
      return io::recv(*_s, t);
    }
  };

}

#endif