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

/** Whether the compiler supports move constructors and assignment operators. */
#define IB_HAS_RVALUE_REFS ZMQ_HAS_RVALUE_REFS

/** The version number of the exchange protocol implementation.  */
#define IB_EXCHANGE_PROTO_VERSION 1    

/** Assert expression or throw imagebabble::ib_error */
#define IB_ASSERT(expr, reason)               \
  if (!(expr)) {                              \
    throw ::imagebabble::ib_error((reason));  \
  }                                           \

/** Assert ZMQ related expression or throw imagebabble::ib_error */
#define IB_ASSERT_ZMQ(expr)                   \
  try {                                       \
    IB_ASSERT((expr), ib_error::EZMQERROR);   \
  } catch (const zmq::error_t &e) {           \
    throw ib_error(ib_error::EZMQERROR, e);   \
  }

/** Assert ZMQ related expression or throw imagebabble::ib_error */
#define IB_CONVERT_ERROR(expr)                \
  try {                                       \
    (expr);                                   \
  } catch (const zmq::error_t &e) {           \
    throw ib_error(ib_error::EZMQERROR, e);   \
  }


/** A lightweight C++ library to send and receive images via networks */
namespace imagebabble {

  /** Reference counted pointer to a ZMQ socket. */
  typedef std::shared_ptr<zmq::socket_t> socket_ptr;
  /** Reference counted pointer to a ZMQ context. */
  typedef std::shared_ptr<zmq::context_t> context_ptr;
  
  /** Indicator to intruct reuse of existing memory */
  class share_mem {
  public:

    /** Construct with default free behaviour. The default free
      * behaviour is to perform no deletion. */
    inline share_mem()
      : _f(null_deleter), _h(0)
    {}

    /** Construct with custom free function. Allows the caller
      * to pass a custom function that is being invoked when ZMQ
      * library is done with processing it. */
    inline share_mem(zmq::free_fn *f, void *hint)
      : _f(f), _h(hint)
    {}
    
    /** Get free functions. */
    inline zmq::free_fn *get_free_fn() const
    {
      return _f;
    }

    /** Get hint. */
    inline void *get_hint() const {
      return _h;
    }

  private:

    /** Null deleter in case no custom free function is supplied. */
    static inline void null_deleter (void *data, void *hint)
    {}

    zmq::free_fn *_f;
    void *_h;
  };

  /** Indicator to instruct copy from existing memory. */
  struct copy_mem {};

  /** ImageBabble error class. */
  class ib_error : public std::exception {
  public:
    /** Reason why this error is thrown. */
    enum ereason {
      /** Unknown error occurred */
      EUNKNOWN,
      /** ZMQ threw an error. Use ib_error::get_zmq_errno to receive details. */
      EZMQERROR,
      /** Conversion from or to a specific data type failed. */
      ECONVERSION,
      /** Pre-existing user memory is too small to receive data. */
      EBUFFERTOOSMALL
    };

    /** Construct using unkown error. */
    inline ib_error()
      : _e(EUNKNOWN), _zmq_errno(0)
    {}

    /** Construct from specific reason. */
    inline ib_error(ereason e)
      : _e(e), _zmq_errno(0)
    {}

    /** Construct from ZMQ error. */
    inline ib_error(ereason e, const zmq::error_t &ex)
      : _e(e), _zmq_errno(ex.num())
    {}

    /** Get the reason. */
    inline ereason get_reason() const throw ()
    {
      return _e;
    }

    /** Get the ZMQ error number. */
    inline int get_zmq_errno() const throw ()
    {
      return _zmq_errno;
    }

    /** Get a textual representation of the error. */
    inline const char *what() const throw()
    {
      switch(_e) 
      {
      case EUNKNOWN:
        return "Unknown error";
      case EZMQERROR:
        return zmq_strerror(_zmq_errno);
      case ECONVERSION:
        return "Type conversion failed";
      case EBUFFERTOOSMALL:
        return "Pre-allocated buffer was too small";
      default:
        return "Unknown error";
      }
    }

  private:
    int _zmq_errno;
    ereason _e;
  };

  /** Base class for objects communicating via networks. 
    * Servers and clients shall derive from this base. */
  class network_entity {
  public:

    /** Construct from context. */
    network_entity(const context_ptr &c)
      :_ctx(c)
    {}

    /** Destructor. */
    virtual ~network_entity()
    {}

  protected:    

    context_ptr _ctx; ///< ZMQ context
    socket_ptr _s;    ///< ZMQ socket

  private:
    /** Disabled copy constructor */
    network_entity (const network_entity &);
    /** Disabled assignment operator */
    network_entity &operator = (const network_entity &);
  };

  /** Base class for servers. */
  class basic_server : public network_entity {
  public:
    
    /** Construct from context */
    basic_server(const context_ptr &c)
      : network_entity(c)
    {}    

  };

  /** Base class for clients. */
  class basic_client : public network_entity {
  public:
    
    /** Construct from context */
    basic_client(const context_ptr &c)
      : network_entity(c)
    {} 

  };

  /** Stopwatch implementation. */
  class stopwatch {
  public:

    /** Start stopwatch */
    inline stopwatch()
      : _handle(zmq_stopwatch_start()), _elapsed(0)
    {}

    /** Destroy stopwatch */
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

  /** Send and receive functions for data types. */
  namespace io {
    
    /** Indicates to send an empty message. */
    struct empty {};

    /** Indicator to discard next message part to be received. */
    typedef empty drop;

    /** Simple in-memory stream buffer. Allows to adapt an istream on an existing buffer. 
      * This avoids costly copy operations of std::ostringstream and std::istringstream.*/      
    class in_memory_buffer : public std::basic_streambuf<char>
    {
    public:

      /** Construct from existing buffer. */
      inline in_memory_buffer(char* p, size_t n) 
      {
        setg(p, p, p + n);
        setp(p, p + n);
      }
    };

    /** Discards the remainder of a multi-part message */
    inline void discard_remainder(zmq::socket_t &s) {
      int more;
      size_t more_size = sizeof(more);
      
      // Note, we don't allow any exceptions to bubble up,
      // this method is most often used inside error handling
      // code parts.
      try {        
        s.getsockopt(ZMQ_RCVMORE, &more, &more_size);
        while (more > 0) {
          s.recv(0, 0);
          s.getsockopt(ZMQ_RCVMORE, &more, &more_size);
        }
      } catch (const zmq::error_t &) {}
    }

    /** Generic receive method. Tries to receive a value of T from the given socket.
      * T must have locatable extraction semantics. This method will block until
      * at least one byte is readable from the socket or an error occurs. 
      * 
      * \param[in] s socket to receive from
      * \param[in,out] v value to receive
      * \throws ib_error on error.
      */
    template<class T>
    inline void recv(zmq::socket_t &s, T &v) 
    {
      zmq::message_t msg;
      IB_ASSERT_ZMQ(s.recv(&msg));
      
      in_memory_buffer mb(static_cast<char*>(msg.data()), msg.size());
      std::istream is(&mb);
      is >> v;    

      IB_ASSERT(!is.fail(), ib_error::ECONVERSION);
    }

    /** Read a message(part) from the socket and discard. */
    inline void recv(zmq::socket_t &s, drop &v) 
    {
      zmq::message_t msg;
      IB_ASSERT_ZMQ(s.recv(&msg));
    }

    /** Receive a string. */
    inline void recv(zmq::socket_t &s, std::string &v) 
    {
      zmq::message_t msg;
      IB_ASSERT_ZMQ(s.recv(&msg));

      v.assign(
        static_cast<char*>(msg.data()), 
        static_cast<char*>(msg.data()) + msg.size()); 
    }

    /** Receive a vector of elements. */
    template<class T>
    inline void recv(zmq::socket_t &s, std::vector<T> &c)
    {
      size_t count;
      io::recv(s, count);
     
      // Note, use resize to allow existing elements to be reused.
      c.resize(count);

      // Receive elements
      for (size_t i = 0; i < count; ++i) { 
        io::recv(s, c[i]);
      }

      // Array ends with empty element
      io::recv(s, drop());
    }

    /** Test if data to be read is pending on the socket. Returns true
      * when at least one byte readable within the given timeout in milli
      * seconds. */
    inline bool is_data_pending(zmq::socket_t &s, int timeout_ms)
    {
      zmq::pollitem_t items[] = {{ s, 0, ZMQ_POLLIN, 0 }};      
      IB_ASSERT_ZMQ(zmq::poll(&items[0], 1, timeout_ms) >= 0);
      return (items[0].revents & ZMQ_POLLIN);      
    }

    /** Generic send method. T must have insertion operator semantics. 
      *
      * \param[in] s socket to send data to
      * \param[in] v data to send
      * \param[in] flags ZMQ send flags.
      * \throws ib_error on error.
      */
    template<class T>
    inline void send(zmq::socket_t &s, const T &v, int flags = 0) 
    {
      std::ostringstream ostr;
      ostr << v;

      IB_ASSERT(ostr.good(), ib_error::ECONVERSION);

      const std::string &str = ostr.str();
      zmq::message_t msg(str.size());
      if (!str.empty()) {
        memcpy(msg.data(), str.c_str(), str.size());
      }

      IB_ASSERT_ZMQ(s.send(msg, flags));
    }

    /** Send string. */
    inline void send(zmq::socket_t &s, const std::string &v, int flags = 0) 
    {
      zmq::message_t msg(v.size());
      if (!v.empty()) {
        memcpy(msg.data(), v.c_str(), v.size());
      }
      IB_ASSERT_ZMQ(s.send(msg, flags));
    }

    /** Send empty message. */
    inline void send(zmq::socket_t &s, const empty &v, int flags = 0) 
    {
      zmq::message_t msg(0);
      IB_ASSERT_ZMQ(s.send(msg, flags));
    }

    /** Send vector of elements. A vector is serialized into a multi-part message.
      * First the number of elements is sent, then each element is sent in turn.
      * Finally an empty message marks the end of vector. */
    template<class T>
    inline void send(zmq::socket_t &s, const std::vector<T> &c, int flags = 0)
    {
      const size_t nelems = c.size();
      io::send(s, nelems, ZMQ_SNDMORE);
      for (size_t i = 0; i < nelems; ++i) {
        io::send(s, c[i], ZMQ_SNDMORE);
      }
      
      io::send(s, empty(), flags);
    }
  }

  /** Fast but unreliable server implementation. The fast server implementation is based
    * on a publisher/subscriber pattern to fan out data to all connected clients. It
    * avoids back-chatter which improves throughput. The basic guarantees for clients 
    * are that they either receive the complete data published or no data at all.
    *
    * The server will drop messages when no clients are connected. Messages for clients
    * in exceptional states are dropped as well. Expect to lose data when using the 
    * fast server.*/
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

    /** Start a new connection on the given endpoint. This method can be called
      * multiple times to publish the same data on multiple endpoints.
      *
      * \param[in] addr address to bind server to.
      * \throws ib_error on error.
      */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    { 
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_PUB));
      }

      IB_CONVERT_ERROR(_s->bind(addr.c_str()));
    }
    
    /** Shutdown connections. Closes all previously bound endpoints. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close(); // does not throw.
        _s.reset();
      }   
    }

    /** Publish data to clients. Data to be published must either have
      * a specific send method overload in the imagebabble::io namespace,
      * or primitive output stream insertion (operator<<) semantics.
      * 
      * \param[in] t data to be published.
      * \param[in] timeout_ms unused. Method returns always immediately.
      * \param[in] min_serve unused.
      * \returns true if data was published successfully.
      * \returns false never.
      **/
    template<class T>
    bool publish(const T &t, int timeout_ms = 0, size_t min_serve = 0)
    {
      io::send(*_s, t);
      return true;
    }
  };

  /** Reliable server implementation. The reliable server implementation is based
    * on a request/reply pattern send data to a set of connected clients. It is reliable
    * in the term that no data is lost due to filled queues on both ends.
    * 
    * When new data is to be published, it waits for the requested number of clients 
    * to report readiness. If at least the specified number of clients gets ready in 
    * the specified timeout interval, the data is sent to all clients. If not, no data 
    * at all is sent.
    */
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

    /** Start a new connection on the given endpoint. This method can be called
      * multiple times to publish the same data on multiple endpoints.
      *
      * \param[in] addr address to bind to.
      * \throws ib_error on error
      */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {  
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_ROUTER));
      }

      IB_CONVERT_ERROR(_s->bind(addr.c_str()));
    }
    
    /** Shutdown connections. Closes all previously bound endpoints. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close(); // does not throw
        _s.reset();
      }      
    }

    /** Publish data to clients. Data to be published must either have
      * a specific send method overload in the imagebabble::io namespace,
      * or primitive output stream insertion (operator<<) semantics.
      * 
      * \param [in] t data to be published.
      * \param [in] min_serve minimum number of clients to serve before returning.
      * \param [in] timeout_ms maximum wait time in milliseconds for clients to get ready.
      * \returns true when data was published successfully
      * \returns false when a send timeout occurred.
      * \throws ib_error on error.
      **/
    template<class T>
    inline bool publish(const T &t, int timeout_ms = -1, size_t min_serve = 1)
    {
      typedef std::hash_set<std::string> client_set;

      client_set clients;
      stopwatch sw;

      bool continue_wait = true;
      int wait_time = timeout_ms;

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
        continue_wait = (timeout_ms == -1 || ((int)sw.elapsed_msecs() < timeout_ms));
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
        // Not enough clients within given timeout        
        return false;
      }      
    }
  };

  /** Fast client implementation. */
  class fast_client : public basic_client {
  public:

    /** Default constructor */
    fast_client()
      : basic_client(context_ptr(new zmq::context_t(1)))
    {}

    /** Start a new connection to the given endpoint. If called multiple times, the client
      * will connect to more endpoints. 
      *
      * \param [in] addr endpoint address to connect to
      * \throws ib_error on error
      */      
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_SUB));

        int linger = 0;
        IB_CONVERT_ERROR(_s->setsockopt(ZMQ_LINGER, &linger, sizeof(int)));
        IB_CONVERT_ERROR(_s->setsockopt(ZMQ_SUBSCRIBE, 0, 0));
      }

      IB_CONVERT_ERROR(_s->connect(addr.c_str()));
    }

    /** Shutdown all connections. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close(); // does not throw
        _s.reset();
      }      
    }

    /** Receive data. Data to be received must either have a specific
      * imagebabble::io::recv method overload , or primitive output stream
      * extraction (operator>>) semantics. 
      *
      * \warning You should not rely on receiving a single specific data element 
      *          with the fast_client and fast_server implementation. Expect data to get lost.
      *
      * \param [in,out] t data to be received
      * \param [in] timeout_ms Maximum wait time in milliseconds to receive data.
      * \returns true if data was received successfully.
      * \returns false when receive timeout occurred.
      * \throws ib_error on error
      */
    template<class T>
    inline bool receive(T &t, int timeout_ms = 0)
    {
      try {
        if (!io::is_data_pending(*_s, timeout_ms)) {
          return false;
        }

        io::recv(*_s, t);
        
        return true;
      } catch (ib_error &e) {
        // clean up
        io::discard_remainder(*_s);
        throw e;
      }

    }

  };

  /** Reliable client implementation. */
  class reliable_client : public basic_client {
  public:

    /** Default constructor */
    reliable_client()
      : basic_client(context_ptr(new zmq::context_t(1)))
    {}

    /** Start a new connection to the given endpoint. If called multiple times, 
      * the client will connect to more endpoints. 
      *
      * \param [in] addr endpoint address
      * \throws ib_error on error 
      */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {

      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_DEALER));     
      
        int linger = 0; 
        IB_CONVERT_ERROR(_s->setsockopt(ZMQ_LINGER, &linger, sizeof(int)));   
      }

      IB_CONVERT_ERROR(_s->connect(addr.c_str()));
    }

    /** Shutdown all connections. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close(); // does not throw
        _s.reset();
      }
    }

    /** Receive data. Data to be received must either have a specific
      * imagebabble::io::recv method overload, or primitive output stream
      * extraction (operator>>) semantics. 
      *
      * \param [in,out] t data to be received
      * \param [in] timeout_ms Maximum wait time in milliseconds to receive data.
      * \returns true if data was received successfully
      * \returns false if receive timeout occurred
      * \throws ib_error on error */
    template<class T>
    bool receive(T &t, int timeout_ms = -1) 
    {
      try {
        // Send ready
        io::send(*_s, io::empty());

        // Wait for reply
        if (!io::is_data_pending(*_s, timeout_ms)) {
          return false;
        }
        
        // Data is here, return
        io::recv(*_s, t);

        return true;
      } catch (const ib_error &e) {
        io::discard_remainder(*_s);
        throw e;
      }
    }
  };

}

#endif