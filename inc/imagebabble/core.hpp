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
#include <unordered_set>

/** Whether the compiler supports move constructors and assignment operators. */
#define IB_HAS_RVALUE_REFS ZMQ_HAS_RVALUE_REFS

/** The version identification for fast protocol.  */
#define IB_EXCHANGE_PROTO_FAST_VERSION "f001"    
/** The version identification for reliable protocol.  */
#define IB_EXCHANGE_PROTO_RELIABLE_VERSION "r001"

/** Assert expression or throw imagebabble::ib_error */
#define IB_ASSERT(expr, reason)               \
  if (!(expr)) {                              \
    throw ::imagebabble::ib_error((reason));  \
  }                                           \

/** Catches ZMQ execptions and models them as imagebabble::ib_error */
#define IB_CATCH_ZMQ_RETHROW(expr)            \
  try {                                       \
    (expr);                                   \
  } catch (const zmq::error_t &e) {           \
    throw ib_error(ib_error::EZMQERROR, e);   \
  }

/** Try to send/recv first part of message. Returns false if sending receiving failed due to EAGAIN. 
  * Re-models any ZMQ exception as imagebabble::ib_error. This macro is generally used to assert
  * sending of the first message part. It provides an early-exit strategy in case ZMQ_DONTWAIT was
  * passed as flag.
  */
#define IB_FIRST_PART(expr)                   \
  try {                                       \
    if (!(expr)) {                            \
      return false;                           \
    }                                         \
  } catch (const zmq::error_t &e) {           \
    throw ib_error(ib_error::EZMQERROR, e);   \
  }

/** Assert sending receiving a subsequent message part. Throws an imagebabble::ib_error if sending or 
  * receiving fails, even if error code is EAGAIN. */
#define IB_NEXT_PART(expr)                    \
  try {                                       \
    IB_ASSERT((expr), ib_error::EINCOMPLETE); \
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
      /** Sending or receiving failed because of missing data. */
      EINCOMPLETE,
      /** Pre-existing user memory is too small to receive data. */
      EBUFFERTOOSMALL,
      /** Socket is invalid */
      EINVALIDSOCKET,
      /** Parameter out of range */
      EPARAMRANGE,
      /** Incompatible talk versions */
      EWRONGPROTO
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
      case EINCOMPLETE:
        return "Sending or receiving multi-part message failed. Try again.";
      case EBUFFERTOOSMALL:
        return "Pre-allocated buffer was too small";
      case EINVALIDSOCKET:
        return "Socket isn't yet allocated.";
      case EWRONGPROTO:
        return "Wrong protocol version or type.";
      case EPARAMRANGE:
        return "Parameter is out of range.";
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
      :_ctx(c), _hwm_snd(-1), _hwm_recv(-1), _no_linger(true)
    {}

    /** Destructor. */
    virtual ~network_entity()
    {
      shutdown();
    }

    /** Get context */
    inline context_ptr get_context() const {
      return _ctx;
    }

    /** Get socket */
    inline socket_ptr get_socket() const {
      return _s;
    }

    /** Startup a new service or connection to service. Calling this method multiple
      * times shall result in creating additional services or connections. */
    virtual void startup(const std::string &addr) = 0;

    /** Shutdown all services or connections*/
    virtual void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close(); // does not throw.
        _s.reset();
      }   
    }

    /** Set the maximum number of pending message(parts) in the send queue 
      * before the socket enters an exceptional state */
    void set_max_pending_outbound(int nmessages) {
      _hwm_snd = nmessages;
    }

    /** Set the maximum number of pending message(parts) in the receive queue 
      * before the socket enters an exceptional state */
    void set_max_pending_inbound(int nmessages) {
      _hwm_recv = nmessages;
    }
   
  protected:    

    /** Apply common socket options */
    void apply_socket_options() {
      if (_s) {
        
        // Don't wait for pending messages to be send on shutdown.
        if (_no_linger) {
          int linger = 0;
          IB_CATCH_ZMQ_RETHROW(_s->setsockopt(ZMQ_LINGER, &linger, sizeof(int)));
        }

        if (_hwm_snd >= 0) {
          IB_CATCH_ZMQ_RETHROW(_s->setsockopt(ZMQ_SNDHWM, &_hwm_snd, sizeof(int)));
        }

        if (_hwm_recv >= 0) {
          IB_CATCH_ZMQ_RETHROW(_s->setsockopt(ZMQ_RCVHWM, &_hwm_recv, sizeof(int)));
        }
      }
    }


    context_ptr _ctx; ///< ZMQ context
    socket_ptr _s;    ///< ZMQ socket
    int _hwm_snd;     ///< High water mark for outbound messages
    int _hwm_recv;    ///< High water mark for inbound messages
    bool _no_linger;  ///< Discard all messages on shutdown

  private:
    /** Disabled copy constructor */
    network_entity (const network_entity &);
    /** Disabled assignment operator */
    network_entity &operator = (const network_entity &);
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

  /** Poller service to query multiple network items for readability/writablility states. */
  class poller {
  public:
    
    /** Construct empty poller */
    inline poller()
    {}

    /** Add network entity for polling. Returns identifier of item. */
    inline size_t add(network_entity &ne)
    {
      socket_ptr s = ne.get_socket();
      IB_ASSERT(s, ib_error::EINVALIDSOCKET);

      size_t id = _sockets.size();

      zmq::pollitem_t pi = {*s, 0, ZMQ_POLLIN | ZMQ_POLLOUT | ZMQ_POLLERR, 0};      
      _items.push_back(pi);
      _sockets.push_back(s);

      return id;
    }

    /** Remove network entity from polling. */
    inline void remove(size_t id)
    {
       IB_ASSERT(id < _sockets.size(), ib_error::EPARAMRANGE);
       _sockets.erase(_sockets.begin() + id);
       _items.erase(_items.begin() + id);
    }

    /** Remove all network entities */
    inline void clear()
    {
      _sockets.clear();
      _items.clear();
    }

    /** Poll using a specified timeout. If timeout is -1 waits until at least one item
      * has an event. If timeout is 0 does not wait at all. */
    inline bool poll_any(int timeout_ms, short events = ZMQ_POLLIN) 
    {
      apply_events(events);

      int n;
      IB_CATCH_ZMQ_RETHROW((n = zmq::poll(&_items.at(0), _items.size(), timeout_ms)));
      return n > 0;
    }

    /** Poll using a specified timeout. If timeout is -1 waits until at all items
      * have an event. If timeout is 0 does not wait at all. */
    inline bool poll_all(int timeout_ms, short events = ZMQ_POLLIN) 
    {
      apply_events(events);

      stopwatch sw;
      const bool wait_inf = (timeout_ms == -1);
      int wait_time = timeout_ms;
      
      bool all_events = true;
      do {
        IB_CATCH_ZMQ_RETHROW(zmq::poll(&_items.at(0), _items.size(), wait_time));

        // Check if all items have events
        all_events = true;
        for (size_t i = 0; i < _items.size(); ++i) {
          if (_items[i].revents == 0) {
            all_events = false;
            break;
          }
        }

        if (!wait_inf)
            wait_time = timeout_ms - (int)sw.elapsed_msecs();

      } while (!all_events && (wait_inf || wait_time > 0));

      return all_events;
    }

    /** Test if item is readable */
    inline bool is_readable(size_t id) const
    {
      IB_ASSERT(id < _items.size(), ib_error::EPARAMRANGE);
      return (_items[id].revents & ZMQ_POLLIN) > 0;
    }

    /** Test if item is writeable. */
    inline bool is_writable(size_t id) const
    {
      IB_ASSERT(id < _items.size(), ib_error::EPARAMRANGE);
      return (_items[id].revents & ZMQ_POLLOUT) > 0;
    }

    /** Test if item has error state. */
    inline bool is_erroneous(size_t id) const
    {
      IB_ASSERT(id < _items.size(), ib_error::EPARAMRANGE);
      return (_items[id].revents & ZMQ_POLLERR) > 0;
    }

  private:

    /** Apply event flags for polling */
    void apply_events(short events) {
      for (size_t i = 0; i < _items.size(); ++i) {
        _items[i].events = events;        
      }
    }

    std::vector<socket_ptr> _sockets;
    std::vector<zmq::pollitem_t> _items;
  };


  /** Base class for servers. 
    *
    * \tparam T Datatype to publish. Data to be published must either have
    *         a specific send method overload in the imagebabble::io namespace,
    *         or primitive output stream insertion (operator<<) semantics.
    */
  template<typename T>
  class basic_server : public network_entity {
  public:
    
    /** Construct from context */
    basic_server(const context_ptr &c)
      : network_entity(c)
    {}    

    /** Publish data to clients. 
      * 
      * \param[in] t data to be published.
      * \param[in] timeout_ms Maximum time to wait for method to complete successfully.
      *            Behaviour is implementation dependent.
      * \param[in] min_serve Minimum number of clients to serve.
                   Behaviour is dependent on concrete implementation.
      * \returns true if data was published successfully.
      * \returns false if timeout occurred.
      * \throws ib_error on error.
      **/
    virtual bool publish(const T &t, int timeout_ms, size_t min_serve) = 0;  
  };

  /** Base class for clients. 
    *
    * \tparam Data type to receive. Data to be received must either have a specific
    * imagebabble::io::recv method overload, or primitive output stream
    * extraction (operator>>) semantics. 
    */
  template<typename T>
  class basic_client : public network_entity {
  public:
    
    /** Construct from context */
    basic_client(const context_ptr &c)
      : network_entity(c)
    {} 

    /** Receive data. 
      *
      * \param [in,out] t data to be received
      * \param [in] timeout_ms Maximum wait time in milliseconds to receive data.
      * \returns true if data was received successfully.
      * \returns false when receive timeout occurred.
      * \throws ib_error on error.
      */
    virtual bool receive(T &t, int timeout_ms) = 0;

  protected:

    /** Validate talk-version. The protocol version is
      * sent as first field in a message. */
    void validate_version(const char *build_version, const std::string &recv_version) 
    {
      if (recv_version != build_version)
        throw ib_error(ib_error::EWRONGPROTO);
    }
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
  
    /** Ensure clean-up of partial messages when an error occurs
      * in the middle of such messages. */
    class ensure_cleanup_partial_messages {
    public:

      /** Initialize from entity */
      ensure_cleanup_partial_messages(socket_ptr s)
        :_s(s)
      {
        IB_ASSERT(_s, ib_error::EINVALIDSOCKET);
      }

      /** Cleanup on destruction */
      ~ensure_cleanup_partial_messages() {
        io::discard_remainder(*_s);
      }

    private:
      socket_ptr _s;      
    };

    /** Generic receive method. Tries to receive a value of T from the given socket.
      * T must have locatable extraction semantics. This method will block until
      * at least one byte is readable from the socket or an error occurs. 
      * 
      * \param[in] s socket to receive from
      * \param[in,out] v value to receive
      * \param[in] flags ZMQ flags
      * \throws ib_error on error.
      */
    template<class T>
    inline bool recv(zmq::socket_t &s, T &v, int flags) 
    {
      zmq::message_t msg;
      IB_FIRST_PART(s.recv(&msg, flags));
      
      in_memory_buffer mb(static_cast<char*>(msg.data()), msg.size());
      std::istream is(&mb);
      is >> v;    

      IB_ASSERT(!is.fail(), ib_error::ECONVERSION);

      return true;
    }

    /** Read a message(part) from the socket and discard. */
    inline bool recv(zmq::socket_t &s, drop &v, int flags) 
    {
      zmq::message_t msg;
      IB_FIRST_PART(s.recv(&msg, flags));
      return true;
    }

    /** Receive a string. */
    inline bool recv(zmq::socket_t &s, std::string &v, int flags) 
    {
      zmq::message_t msg;
      
      IB_FIRST_PART(s.recv(&msg, flags));
      
      v.assign(
        static_cast<char*>(msg.data()), 
        static_cast<char*>(msg.data()) + msg.size()); 

      return true;
    }

    /** Receive a vector of elements. */
    template<class T>
    inline bool recv(zmq::socket_t &s, std::vector<T> &c, int flags)
    {
      size_t count;

      IB_FIRST_PART(io::recv(s, count, flags));

      // Note, use resize to allow existing elements to be reused.
      c.resize(count);

      // Receive elements
      for (size_t i = 0; i < count; ++i) { 
        IB_NEXT_PART(io::recv(s, c[i], flags));        
      }

      // Array ends with empty element
      IB_NEXT_PART(io::recv(s, drop(), flags));

      return true;
    }

    /** Test if data to be read is pending on the socket. Returns true
      * when at least one byte readable within the given timeout in milli
      * seconds. */
    inline bool is_data_pending(zmq::socket_t &s, int timeout_ms)
    {
      zmq::pollitem_t items[] = {{ s, 0, ZMQ_POLLIN, 0 }};      
      IB_CATCH_ZMQ_RETHROW(zmq::poll(&items[0], 1, timeout_ms));
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
    inline bool send(zmq::socket_t &s, const T &v, int flags) 
    {
      std::ostringstream ostr;
      ostr << v;

      IB_ASSERT(ostr.good(), ib_error::ECONVERSION);

      const std::string &str = ostr.str();
      zmq::message_t msg(str.size());
      if (!str.empty()) {
        memcpy(msg.data(), str.c_str(), str.size());
      }

      IB_FIRST_PART(s.send(msg, flags));
      return true;
    }

    /** Send string. */
    inline bool send(zmq::socket_t &s, const std::string &v, int flags) 
    {
      zmq::message_t msg(v.size());
      if (!v.empty()) {
        memcpy(msg.data(), v.c_str(), v.size());
      }
      
      IB_FIRST_PART(s.send(msg, flags));
      return true;
    }

    /** Send empty message. */
    inline bool send(zmq::socket_t &s, const empty &v, int flags) 
    {
      zmq::message_t msg(0);
      
      IB_FIRST_PART(s.send(msg, flags));
      return true;    
    }

    /** Send vector of elements. A vector is serialized into a multi-part message.
      * First the number of elements is sent, then each element is sent in turn.
      * Finally an empty message marks the end of vector. */
    template<class T>
    inline bool send(zmq::socket_t &s, const std::vector<T> &c, int flags)
    {
      const size_t nelems = c.size();

      IB_FIRST_PART(io::send(s, nelems, flags | ZMQ_SNDMORE));

      for (size_t i = 0; i < nelems; ++i) {
        IB_NEXT_PART(io::send(s, c[i], flags | ZMQ_SNDMORE));        
      }
      
      IB_NEXT_PART(io::send(s, empty(), flags));
      
      return true;
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
  template<typename T>
  class fast_server : public basic_server<T> {
  public:

    /** Default constructor. */
    fast_server()
      : basic_server<T>(context_ptr(new zmq::context_t(1)))
    {}

    /** Destructor. */
    virtual ~fast_server()
    {}

    /** Start a new connection on the given endpoint. This method can be called
      * multiple times to publish the same data on multiple endpoints.
      *
      * \param[in] addr address to bind server to.
      * \throws ib_error on error.
      */
    virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    { 
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_PUB));
        network_entity::apply_socket_options();
      }

      IB_CATCH_ZMQ_RETHROW(_s->bind(addr.c_str()));
    }

    /** Publish data to clients.
      * 
      * \param[in] t data to be published.
      * \param[in] timeout_ms unused. Method returns always immediately.
      * \param[in] min_serve unused.
      * \returns true if data was published successfully.
      * \returns false never.
      **/
    virtual bool publish(const T &t, int timeout_ms = 0, size_t min_serve = 0)
    {
      IB_ASSERT(_s, ib_error::EINVALIDSOCKET);
      io::send(*_s, IB_EXCHANGE_PROTO_FAST_VERSION, ZMQ_SNDMORE);
      io::send(*_s, t, 0);
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
  template<typename T>
  class reliable_server : public basic_server<T> {
  public:

    /** Default constructor. */
    reliable_server()
      : basic_server<T>(context_ptr(new zmq::context_t(1)))
    {}

    /** Destructor. */
    virtual ~reliable_server()
    {}

    /** Start a new connection on the given endpoint. This method can be called
      * multiple times to publish the same data on multiple endpoints.
      *
      * \param[in] addr address to bind to.
      * \throws ib_error on error
      */
    virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {  
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_ROUTER));
        network_entity::apply_socket_options();
      }

      IB_CATCH_ZMQ_RETHROW(_s->bind(addr.c_str()));
    }

    /** Shutdown server */
    virtual void shutdown()
    {
      _clients.clear();
      basic_server<T>::shutdown();
    }

    /** Publish data to clients.
      * 
      * \param [in] t data to be published.
      * \param [in] min_serve minimum number of clients to serve before returning.
      * \param [in] timeout_ms maximum wait time in milliseconds for clients to get ready.
      * \returns true when data was published successfully
      * \returns false when a send timeout occurred.
      * \throws ib_error on error.
      **/
    virtual bool publish(const T &t, int timeout_ms = -1, size_t min_serve = 1)
    {
      IB_ASSERT(_s, ib_error::EINVALIDSOCKET);

      stopwatch sw;

      bool new_data = false;
      const bool wait_inf = (timeout_ms == -1);
      int wait_time = timeout_ms;

      do {

        // Read off all available registrations
        do {
          new_data = receive_client_address(ZMQ_DONTWAIT);
        } while (new_data);
       
        // No more data, see if we should wait for more
        if ((_clients.size() < min_serve) && (wait_time > 0 || wait_inf)) {
          
          if (!wait_inf)
            wait_time = timeout_ms - (int)sw.elapsed_msecs();

          new_data = io::is_data_pending(*_s, wait_time);          
        }

      } while (new_data);


      if (_clients.size() >= min_serve) {
        // Success, send to all clients
        std::unordered_set<std::string>::iterator i = _clients.begin();
        std::unordered_set<std::string>::iterator i_end = _clients.end();

        for (i; i != i_end; ++i) {
          send_client_payload(*i, t);          
        }

        _clients.clear();

        return true;
      } else {
        // Not enough clients within given timeout        
        return false;
      }      
    }
  private:
    typedef std::unordered_set<std::string> client_set;    

    /** Try to receive client ready flag */
    bool receive_client_address(int flags) {
      std::string address;
      
      IB_FIRST_PART(io::recv(*_s, address, flags));
      IB_NEXT_PART(io::recv(*_s, io::drop(), flags));

      _clients.insert(address);
      return true;
    }

    /** Send payload to client */
    bool send_client_payload(const std::string &addr, const T &t)
    {
      IB_FIRST_PART(io::send(*_s, addr, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*_s, IB_EXCHANGE_PROTO_RELIABLE_VERSION, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*_s, t, 0));

      return true;
    }

    client_set _clients;     
  };

  /** Fast client implementation. */
  template<typename T>
  class fast_client : public basic_client<T> {
  public:

    /** Default constructor */
    fast_client()
      : basic_client<T>(context_ptr(new zmq::context_t(1)))
      , _enable_skip(false), _recv_skip(0)
    {}

    virtual ~fast_client()
    {}

    /** Start a new connection to the given endpoint. If called multiple times, the client
      * will connect to more endpoints. 
      *
      * \param [in] addr endpoint address to connect to
      * \throws ib_error on error
      */      
    virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {
      if (!_s) {
        _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_SUB));
      
        network_entity::apply_socket_options();
        IB_CATCH_ZMQ_RETHROW(_s->setsockopt(ZMQ_SUBSCRIBE, 0, 0));

        size_t recvhwm_size = sizeof (_recv_skip);
        IB_CATCH_ZMQ_RETHROW(_s->getsockopt(ZMQ_RCVHWM, &_recv_skip, &recvhwm_size));

      }

      IB_CATCH_ZMQ_RETHROW(_s->connect(addr.c_str()));
    }

    /** Receive data.
      *
      * \warning You should not rely on receiving a single specific data element 
      *          with the fast_client and fast_server implementation. Expect data to get lost.
      *
      * \param [in,out] t data to be received
      * \param [in] timeout_ms Maximum wait time in milliseconds to receive data.
      *             Timeout is set to 1 second by default.
      * \returns true if data was received successfully.
      * \returns false when receive timeout occurred.
      * \throws ib_error on error
      */
    virtual bool receive(T &t, int timeout_ms = 1000)
    {
      IB_ASSERT(_s, ib_error::EINVALIDSOCKET);

      io::ensure_cleanup_partial_messages ecpm(this->get_socket());

      const bool has_wait = (timeout_ms != 0);
      const int max_skip = _enable_skip ? _recv_skip : 0;
      int k = max_skip;
       
      bool has_data = false;
      while (k >= 0 && receive_message(t, ZMQ_DONTWAIT)) {
        --k;
      }

      // Received at least one message from queue.
      if (k != max_skip) {
          return true;
      }

      // We haven't received anything. See if waiting is ok.
      if (has_wait && io::is_data_pending(*_s, timeout_ms)) {
        receive_message(t, 0);
        return true;
      } else {
        return false;
      }

    }

    /** Enable skipping older elements in receive queue. Enabling
      * this property will allow the client to discard old messages
      * in its receive queue and forward to the most recent one. */
    void set_enable_most_recent(bool enable)
    {
      _enable_skip = enable;
    }

  private:

    /** Receive complete message once */
    bool receive_message(T &t, int flags)
    {
      std::string version;

      IB_FIRST_PART(io::recv(*_s, version, flags));
      basic_client<T>::validate_version(IB_EXCHANGE_PROTO_FAST_VERSION, version);
      IB_NEXT_PART(io::recv(*_s, t, flags));

      return true;
    }

    bool _enable_skip;
    int _recv_skip;
  };

  /** Reliable client implementation. */
  template<typename T>
  class reliable_client : public basic_client<T> {
  public:

    /** Default constructor */
    reliable_client()
      : basic_client<T>(context_ptr(new zmq::context_t(1)))
    {}

    virtual ~reliable_client()
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
        network_entity::apply_socket_options();
      }

      IB_CATCH_ZMQ_RETHROW(_s->connect(addr.c_str()));
      this->send_ready();
    }

    /** Receive data.
      *
      * \param [in,out] t data to be received
      * \param [in] timeout_ms Maximum wait time in milliseconds to receive data.
      *             Waits forever by default.
      * \returns true if data was received successfully
      * \returns false if receive timeout occurred
      * \throws ib_error on error */
    virtual bool receive(T &t, int timeout_ms = -1) 
    {
      IB_ASSERT(_s, ib_error::EINVALIDSOCKET);

      io::ensure_cleanup_partial_messages ecpm(this->get_socket());      
      ensure_send_ready sr(this);

      const bool has_wait = (timeout_ms != 0);

      // Try to receive without waiting.
      if (receive_message(t, ZMQ_DONTWAIT)) {
        return true;
      }

      // We haven't received anything. See if waiting is ok.
      if (has_wait && io::is_data_pending(*_s, timeout_ms)) {
        receive_message(t, 0);
        return true;
      } else {
        return false;
      }
    }

  private:

    bool send_ready() {
      // Send ready
      IB_FIRST_PART(io::send(*_s, io::empty(), 0));
      return true;
    }

    /** Sends ready to server on scope exit */
    class ensure_send_ready {
    public:
      ensure_send_ready(reliable_client *rc)
        :_rc(rc)
      {}

      ~ensure_send_ready() {
        _rc->send_ready();
      }
    private:
      reliable_client *_rc;
    };
    
    /** Receive complete message once */
    bool receive_message(T &t, int flags)
    {
      std::string version;

      IB_FIRST_PART(io::recv(*_s, version, flags));
      basic_client<T>::validate_version(IB_EXCHANGE_PROTO_RELIABLE_VERSION, version);
      IB_NEXT_PART(io::recv(*_s, t, flags));
      return true;
    }
  };

}

#endif