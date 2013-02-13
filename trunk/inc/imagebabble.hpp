/*! \file imagebabble.hpp

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

#ifndef __IMAGE_BABBLE_HPP_INCLUDED__
#define __IMAGE_BABBLE_HPP_INCLUDED__

#include <zmq.hpp>
#include <memory>
#include <streambuf>
#include <sstream>
#include <string>
#include <limits>
#include <vector>
#include <hash_set>
#include <ctime>

#define IMAGEBABBLE_IMAGE_PROTO_VERSION "1"
#define IMAGEBABBLE_DISCOVERY_PROTO_VERSION "1"
#define IMAGEBABBLE_HAS_RVALUE_REFS ZMQ_HAS_RVALUE_REFS

namespace imagebabble {

  /** Image format description. */
  class image_header {
  public:
    /** Default constructor */
    inline image_header()
      : _name(""), _width(0), _height(0), _channels(0), _bytes_per_channel(0)
    {}

    /** Construct from values */
    inline image_header(int w, int h, int c, int bc, const std::string &n)
      : _name(n), _width(w), _height(h), _channels(c), _bytes_per_channel(bc)
    {}

#ifdef IMAGEBABBLE_HAS_RVALUE_REFS

    /** Move constructor */
    inline image_header (image_header &&rhs)
      : _name(std::move(rhs._name)), 
        _width(rhs._width), _height(rhs._height), 
        _channels(rhs._channels), 
        _bytes_per_channel(rhs._bytes_per_channel)
    {}

    /** Move assignment operator */
    inline image_header &operator = (image_header &&rhs)
    {
      if (this != &rhs) {
        _name = std::move(rhs._name);
        _width = rhs._width;
        _height = rhs._height;
        _channels = rhs._channels;
        _bytes_per_channel = rhs._bytes_per_channel;
      }
      return *this;
    }

#endif

    /** Set image resolution in x dimension */
    inline void set_width(int n) { _width = n; }
    /** Set image resolution in y dimension */
    inline void set_height(int n) { _height = n; }
    /** Set number of image channels */
    inline void set_number_of_channels(int n) { _channels = n; }
    /** Set number of bytes per channels */
    inline void set_bytes_per_channel(int n) { _bytes_per_channel = n; }
    /** Set image name */
    inline void set_name(const std::string &n) { _name = n; }

    /** Get resolution in x dimension */
    inline int get_width() const { return _width; }
    /** Get resolution in y dimension */
    inline int get_height() const { return _height; }
    /** Get number of image channels */
    inline int get_number_of_channels() const { return _channels; }
    /** Get number of bytes per channel channels */
    inline int get_bytes_per_channel() const { return _bytes_per_channel; }
    /** Get number of total bytes this format describes */
    inline int get_total_bytes() const { return _bytes_per_channel * _channels * _width * _height; }
    /** Get image name */
    inline const std::string &get_name() const { return _name; }

  private:

    friend std::ostream & operator<<(std::ostream &os, const image_header& i);
    friend std::istream & operator>>(std::istream &is, image_header& i);

    std::string _name;
    int _width;
    int _height;
    int _channels;
    int _bytes_per_channel; 
  };

  /** Serialze image_header to stream. */
  inline std::ostream & operator<<(std::ostream &os, const image_header& i)
  {
    os  << i.get_width() << " " 
        << i.get_height() << " "
        << i.get_number_of_channels() << " " 
        << i.get_bytes_per_channel() << " "
        << i.get_name();  

    return os;    
  }

  /** Deserialize image_header from stream. */
  inline std::istream & operator>>(std::istream &is, image_header& i)
  {
    int field;
    is >> field; i.set_width(field);
    is >> field; i.set_height(field);
    is >> field; i.set_number_of_channels(field);
    is >> field; i.set_bytes_per_channel(field);

    std::string n;
    is >> n; i.set_name(n);

    return is;    
  }

  struct share_mem {};
  struct copy_mem {};

  /** Image data buffer. */
  class image_data {
  public:
    /** Construct an empty image data buffer */
    inline image_data()
      : _pre_allocated(false)
    {}

    /** Construct a data buffer by copying from the given buffer. 
      * The constructor does not copy the data but instead increase its
      * reference count. */
    inline image_data(const image_data &other) 
      : _pre_allocated(other._pre_allocated)
    {
      _msg.copy(const_cast<zmq::message_t*>(&other._msg));
    }
  
    /** Construct a data buffer from existing memory. The implementation
      * does not take ownership of the passed memory bock. Freeing it is
      * a responsibility of the caller. */
    inline explicit image_data(void *data, size_t length, const share_mem &) 
      :_msg(data, length, null_deleter, 0), _pre_allocated(true)
    {}

    /** Construct a data buffer from existing memory.*/
    inline explicit image_data(void *data, size_t length, const copy_mem &) 
      : _msg(length), _pre_allocated(false)
    {
      memcpy(_msg.data(), data, length);      
    }

#ifdef IMAGEBABBLE_HAS_RVALUE_REFS

    /** Move constructor */
    inline image_data (image_data &&rhs)
      : _msg(std::move(rhs._msg)),
        _pre_allocated(rhs._pre_allocated)        
    {}

    /** Move assignment operator */
    inline image_data &operator = (image_data &&rhs)
    {
      if (this != &rhs) {
        _msg = std::move(rhs._msg);
        _pre_allocated = rhs._pre_allocated;
      }
      return *this;
    }
#endif

    /** Assign from other buffer. The implementation does not copy the data 
      * but instead increase its reference count. */
    inline image_data &operator=(const image_data &rhs) 
    {
      _msg.copy(const_cast<zmq::message_t*>(&rhs._msg));
      _pre_allocated = rhs._pre_allocated;
      return *this;
    }

    /** Get a pointer to the beginning of the data. */
    template<class T>
    inline const T *ptr() const 
    {
      return static_cast<const T *>(_msg.data());
    }

    /** Get a pointer to the beginning of the data. */
    template<class T>
    inline T *ptr() 
    {
      return static_cast<T * >(_msg.data());
    }

    /** Get the number of bytes stored in the buffer. */
    size_t size() const 
    {
      return _msg.size();
    }

    inline size_t copy_data(void *dst, size_t length) const {
      size_t n = std::min<size_t>(length, _msg.size());
      memcpy(dst, _msg.data(), n);
      return n;
    }

    /** Get the underlying ZMQ message. */
    zmq::message_t &message() 
    {
      return _msg;
    }

    /** Test if existing user memory was passed at allocation. */
    bool pre_allocated() const 
    {
      return _pre_allocated;
    }

  private:
    /** Null deleter in case of supplied user memory. */
    static void null_deleter (void *data, void *hint)
    {}

    zmq::message_t _msg;
    bool _pre_allocated;
  };

  /** Collection of image headers and data to be sent/received. */
  class frame {
  public:    
    /** Construct an empty frame. */
    inline frame() 
    {}

    /** Construct from size */
    inline frame(size_t count_headers, size_t count_data)
      :_headers(count_headers), _data(count_data)
    {}

    /** Construct frame from info */
    inline frame(const std::vector<image_header> &ih, std::vector<image_data> &id, const std::string &ud)
      : _headers(ih), _data(id), _user_data(ud)
    {}

    /** Set image headers */
    inline void set_image_headers(const std::vector<image_header> &v) { _headers = v; }
    /** Set image data buffers */
    inline void set_image_data(const std::vector<image_data> &v) { _data = v; }
    /** Set user data */
    inline void set_user_data(const std::string &v) { _user_data = v; }

    /** Get image headers */
    inline std::vector<image_header> &get_image_headers() { return _headers; }
    /** Get image headers */
    inline const std::vector<image_header> &get_image_headers() const { return _headers; }
    /** Get image data */
    inline std::vector<image_data> &get_image_data() { return _data; }
    /** Get image data */
    inline const std::vector<image_data> &get_image_data() const { return _data; }
    /** Get user data */
    inline std::string &get_user_data() { return _user_data; }
    /** Get user data */
    inline const std::string &get_user_data() const { return _user_data; }

    inline void append_image_header(const image_header &h) { _headers.push_back(h); }
    inline void append_image_data(const image_data &d) { _data.push_back(d); }

#ifdef IMAGEBABBLE_HAS_RVALUE_REFS
    inline void append_image_header(image_header &&h) { _headers.push_back(h); }
    inline void append_image_data(image_data &&d) { _data.push_back(d); }
#endif

  private:
    std::vector<image_header> _headers;
    std::vector<image_data> _data;
    std::string _user_data;
  };

  /** Performance options for sending/receiving frames. */
  class frame_options {
  public:

    /** Default constructor. Doesn't skip anything */
    inline frame_options()
      : _skip_headers(false), _skip_data(false), _skip_user_data(false)
    {}

    /** Enable or disable skipping of image headers. */
    inline void set_skip_image_headers(bool enable) { _skip_headers = enable; }
    /** Enable or disable skipping of image data. */
    inline void set_skip_image_data(bool enable) { _skip_data = enable; }
    /** Enable or disable skipping of user data. */
    inline void set_skip_user_data(bool enable) { _skip_user_data = enable; }

    /** Test if skipping image headers is turned on. */
    inline bool get_skip_image_headers() const { return _skip_headers; }
    /** Test if skipping image data is turned on. */
    inline bool get_skip_image_data() const { return _skip_data; }
    /** Test if skipping of user data is turned on. */
    inline bool get_skip_user_data() const { return _skip_user_data; }

  private:
    bool _skip_headers;
    bool _skip_data;
    bool _skip_user_data;
  };

  /** Reference counted pointer to a ZMQ socket. */
  typedef std::shared_ptr<zmq::socket_t> socket_ptr;
  /** Reference counted pointer to a ZMQ context. */
  typedef std::shared_ptr<zmq::context_t> context_ptr;

  /** Base class for objects communicating on networks */
  class network_entity {
  public:

    /** Defines the protocol type */
    enum e_protocol {
      PROTO_IMAGE_FAST = 0,
      PROTO_IMAGE_RELIABLE = 1,
      PROTO_DISCOVERY = 2,
      PROTO_USER = 100
    };

    /** Construct from context */
    network_entity(const context_ptr &c)
      :_ctx(c)
    {}

    /** Basic destructor */
    virtual ~network_entity()
    {}

    /** Retrieve the protocol type spoken */
    virtual e_protocol get_protocol_type() const = 0;

    /** Retrieve the protocol version */
    virtual std::string get_protocol_version() const = 0;

  protected:    
    context_ptr _ctx;
    socket_ptr _s;

  private:
    /** Disabled copy constructor */
    network_entity (const network_entity &);
    /** Disabled assignment operator */
    network_entity &operator = (const network_entity &);
  };

  /** Holds node information for discovery */
  class discovery_info {
  public:    

    /** Default constructor. */
    inline discovery_info()
      : _name(""), _addr(""), _proto(network_entity::PROTO_USER), _image_version(IMAGEBABBLE_IMAGE_PROTO_VERSION)
    {}

    /** Construct from arguments. */
    inline discovery_info(const std::string &name, const std::string &addr, network_entity::e_protocol proto, const std::string &image_version)
      : _name(name), _addr(addr), _proto(proto), _image_version(image_version)
    {}

    /** Set name of node */
    inline void set_name(const std::string &n) { _name = n; }    
    /** Set endpoint address of node */
    inline void set_address(const std::string &n) { _addr = n; }
    /** Set protocol type. */
    inline void set_protocol_type(const network_entity::e_protocol &n) { _proto = n; }
    /** Set protocol version */
    inline void set_protocol_version(const std::string &n) { _image_version = n; }

    /** Get name of node */
    inline const std::string &get_name() const { return _name; }
    /** Get address of node */
    inline const std::string &get_address() const { return _addr; }
    /** Get protocol type. */
    inline network_entity::e_protocol get_protocol_type() const { return _proto; }
    /** Get protocol version */
    inline const std::string &get_protocol_version() const { return _image_version; }

  private:
    std::string _name;
    network_entity::e_protocol _proto;
    std::string _image_version;
    std::string _addr;
  };

  /** Serialze image_header to stream. */
  inline std::ostream & operator<<(std::ostream &os, const discovery_info& i)
  {
    os  << i.get_name() << " "         
        << i.get_address() << " "         
        << i.get_protocol_version() << " "
        << static_cast<int>(i.get_protocol_type());

    return os;    
  }

  /** Deserialize image_header from stream. */
  inline std::istream & operator>>(std::istream &is, discovery_info& i)
  {
    std::string n;
    is >> n; i.set_name(n);
    is >> n; i.set_address(n);
    is >> n; i.set_protocol_version(n);

    int field;
    is >> field; i.set_protocol_type(static_cast<network_entity::e_protocol>(field));

    return is;    
  }

  /** Base class for fast/reliable image servers. */
  class basic_image_server : public network_entity {
  public:
    
    /** Construct from context */
    basic_image_server(const context_ptr &c)
      : network_entity(c)
    {}    

    /** Publish a frame. */
    virtual bool publish(const frame &f, int timeout, size_t min_serve, const frame_options &fo) = 0;

    /** Get bound address */
    virtual const std::string &get_address() const = 0;

    /** Retrieve the protocol version */
    virtual std::string get_protocol_version() const {
      return IMAGEBABBLE_IMAGE_PROTO_VERSION;
    }

  };

  /** Base class for fast/reliable image clients. */
  class basic_image_client : public network_entity {
  public:
    
    /** Construct from context */
    basic_image_client(const context_ptr &c)
      : network_entity(c)
    {} 

    /** Retrieve the protocol version */
    virtual std::string get_protocol_version() const {
      return IMAGEBABBLE_IMAGE_PROTO_VERSION;
    }

    /** Basic receive prototype */
    virtual bool receive(frame &f, int timeout, const frame_options &fo) = 0;
  };

  /** A CPU timer. */
  class timer {
  public:
    /** Construct with current time */
    inline timer()
      :_begin(std::clock())
    {}

    /** Get elapsed time since construction in milli-seconds */
    inline int elapsed_msecs() const {
      std::clock_t now = std::clock();
      return static_cast<int>(double(now - _begin) / CLOCKS_PER_SEC) * 1000;
    }

  private:
    std::clock_t _begin;
  };

  /** Convenience send and receive functions for use with ZMQ */
  namespace zmq_ext {
    
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

    /** Receive image data. If image data points to pre-allocated user memory,
      * the implementation attempts to receive data directly into that buffer.
      * If the buffer is too small to fit the content, the received bytes are
      * truncated to fit and false is returned. */
    inline bool recv(zmq::socket_t &s, image_data &v) 
    {
      if (v.pre_allocated()) {
        int bytes = zmq_recv(s, v.ptr<void>(), v.size(), 0);
        return bytes <= static_cast<int>(v.size());      
      } else {
        return s.recv(&v.message());
      }    
    }

    /** Receive array*/
    template<class T>
    inline bool recv_array(zmq::socket_t &s, std::vector<T> &c, size_t max_elems)
    {
      bool all_ok = true;

      size_t count;
      all_ok &= zmq_ext::recv(s, count);
      size_t fit = std::min<size_t>(count, max_elems);
      
      c.resize(fit);

      // Receive elements
      for (size_t i = 0; i < fit; ++i) {        
        all_ok &= zmq_ext::recv(s, c[i]);          
      } 

      // Drop whatever doesn't fit
      for (size_t i = fit; i < count; ++i) {
        all_ok &= zmq_ext::recv(s, zmq_ext::drop());
      }
      
      return all_ok;
    }

    inline bool recv(zmq::socket_t &s, const frame_options &o, frame &f)
    {
      bool all_ok = true;
      
      // Receive user data
      if (o.get_skip_user_data()) {
        all_ok &= zmq_ext::recv(s, drop());
      } else {
        all_ok &= zmq_ext::recv(s, f.get_user_data());
      }

      // Receive image headers
      all_ok &= recv_array(
        s, f.get_image_headers(), 
        (o.get_skip_image_headers() ? size_t(0) : (std::numeric_limits<size_t>::max)()));

      // Receive image data
      all_ok &= recv_array(
        s, f.get_image_data(), 
        (o.get_skip_image_data() ? size_t(0) : (std::numeric_limits<size_t>::max)()));      

      // Delimiter
      zmq_ext::recv(s, zmq_ext::drop());


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

    /** Send image data. */
    inline bool send(zmq::socket_t &s, const image_data &v, int flags = 0) 
    {
      // Need to copy in order to increment reference count, otherwise
      // input buffer is nullified.
      image_data d = v;
      return s.send(d.message(), flags);
    }

    /** Send array*/
    template<class T>
    inline bool send_array(zmq::socket_t &s, const std::vector<T> &c, size_t max_elems, int flags = 0)
    {
      bool all_ok = true;

      size_t nelems = std::min<size_t>(c.size(), max_elems);

      all_ok &= zmq_ext::send(s, nelems, ZMQ_SNDMORE);
      for (size_t i = 0; i < nelems; ++i) {
        all_ok &= zmq_ext::send(s, c[i], ZMQ_SNDMORE);
      }

      if (!(flags & ZMQ_SNDMORE)) {
        all_ok &= zmq_ext::send(s, empty());
      }

      return all_ok;
    }

    /** Send image frame */
    inline bool send(zmq::socket_t &s, const frame_options &o, const frame &f, int flags = 0) 
    {
      bool all_ok = true;

      // Send user data
      if (o.get_skip_user_data()) {
        all_ok &= zmq_ext::send(s, zmq_ext::empty(), ZMQ_SNDMORE);
      } else {
        all_ok &= zmq_ext::send(s, f.get_user_data(), ZMQ_SNDMORE);
      }

      // Send headers
      size_t n = (o.get_skip_image_headers() ? size_t(0) : f.get_image_headers().size());
      zmq_ext::send_array(s, f.get_image_headers(), n, ZMQ_SNDMORE);

      // Send data
      n =  (o.get_skip_image_data() ? size_t(0) : f.get_image_data().size());
      zmq_ext::send_array(s, f.get_image_data(), n, ZMQ_SNDMORE);

      // Delimiter
      all_ok &= zmq_ext::send(s, zmq_ext::empty());

      return all_ok;
    }
  }

  /** Fast but unreliable image server implementation. */
  class fast_image_server : public basic_image_server {
  public:

    /** Default constructor. */
    fast_image_server()
      : basic_image_server(context_ptr(new zmq::context_t(1)))
    {}

    /** Destructor. */
    ~fast_image_server()
    {
      shutdown();
    }

    /** Start a new connection on the given endpoint. Any previous active connection will be closed. */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:5562")
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

    /** Retrieve the protocol type spoken */
    virtual e_protocol get_protocol_type() const {
      return PROTO_IMAGE_FAST;
    }

    /** Publish a frame. */
    bool publish(const frame &f, int timeout = 0, size_t min_serve = 0, const frame_options &fo = frame_options())
    {
      return zmq_ext::send(*_s, fo, f);
    }

  private:
    std::string _addr;
  };

  /** Reliable image server implementation. */
  class reliable_image_server : public basic_image_server {
  public:
    /** Default constructor. */
    reliable_image_server()
      : basic_image_server(context_ptr(new zmq::context_t(1)))
    {}

    /** Destructor. */
    ~reliable_image_server()
    {
      shutdown();
    }

    /** Start a new connection on the given endpoint. Any previous active connection will be closed. */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:5562")
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

    /** Retrieve the protocol type spoken */
    e_protocol get_protocol_type() const {
      return PROTO_IMAGE_RELIABLE;
    }

    /** Publish a frame. */
    bool publish(const frame &f, int timeout = -1, size_t min_serve = 1, const frame_options &fo = frame_options())
    {
      typedef std::hash_set<std::string> client_set;

      client_set clients;
      timer t;

      bool continue_wait = true;
      int wait_time = timeout;

      do {
        bool can_read = zmq_ext::is_data_pending(*_s, wait_time);
        while(can_read) {
          // Receive ready requestes by clients
          std::string address;
          zmq_ext::recv(*_s, address);
          zmq_ext::recv(*_s, zmq_ext::drop());

          clients.insert(address);
          can_read = zmq_ext::is_data_pending(*_s, 0);
        }
        continue_wait = (timeout == -1 || (t.elapsed_msecs() < timeout));
      } while ((clients.size() < min_serve) && continue_wait);

      if (clients.size() >= min_serve) {
        // Success, send to all clients
        std::hash_set<std::string>::iterator i = clients.begin();
        std::hash_set<std::string>::iterator i_end = clients.end();

        for (i; i != i_end; ++i) {
          // Construct package for client
          zmq_ext::send(*_s, *i, ZMQ_SNDMORE);
          zmq_ext::send(*_s, fo, f);
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
  class fast_image_client : public basic_image_client {
  public:
    /** Default constructor */
    fast_image_client()
      : basic_image_client(context_ptr(new zmq::context_t(1)))
    {}

    /** Start a new connection to the given endpoint. Previous connections are closed when called multiple times. */      
    inline void startup(const std::string &addr = "tcp://127.0.0.1:5562")
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

    /** Retrieve the protocol type spoken */
    virtual e_protocol get_protocol_type() const {
      return PROTO_IMAGE_FAST;
    }

    /** Receive a frame. */
    bool receive(frame &f, int timeout = 0, const frame_options &fo = frame_options()) 
    {
      if (!zmq_ext::is_data_pending(*_s, timeout)) {
        return false;
      }

      return zmq_ext::recv(*_s, fo, f);
    }

  };

  /** Reliable image client implementation */
  class reliable_image_client : public basic_image_client {
  public:

    /** Default constructor */
    reliable_image_client()
      : basic_image_client(context_ptr(new zmq::context_t(1)))
    {}

    /** Start a new connection to the given endpoint. Previous connections are closed when called multiple times. */      
    inline void startup(const std::string &addr = "tcp://127.0.0.1:5562")
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

    /** Retrieve the protocol type spoken */
    virtual e_protocol get_protocol_type() const {
      return PROTO_IMAGE_RELIABLE;
    }

    /** Receive a frame. */
    bool receive(frame &f, int timeout = -1, const frame_options &fo = frame_options()) 
    {
      // Send ready
      zmq_ext::send(*_s, zmq_ext::empty());

      if (!zmq_ext::is_data_pending(*_s, timeout)) {
        return false;
      }

      return zmq_ext::recv(*_s, fo, f);
    }

  };


  /** A network entity that allows clients to find image servers. */
  class discovery_server : public network_entity {
  public:
    
    /** Default constructor. */
    discovery_server()
      : network_entity(context_ptr(new zmq::context_t(1)))
    {}

    /** Destructor. */
    ~discovery_server()
    {
      shutdown();
    }

    /** Start a new discovery service on the given endpoint. Any previous active connection will be closed. */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {  
      shutdown();
      _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_ROUTER));
      _s->bind(addr.c_str());
    }
    
    /** Shutdown server. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close();
      }      
    }

    
    /** Retrieve the protocol type spoken */
    virtual e_protocol get_protocol_type() const  {
      return PROTO_DISCOVERY;
    }

    /** Retrieve the protocol version */
    virtual std::string get_protocol_version() const {
      return IMAGEBABBLE_DISCOVERY_PROTO_VERSION;
    }

    inline void process_events(int timeout = 0) {
      zmq::socket_t &s = *_s;

      bool can_read = zmq_ext::is_data_pending(s, timeout);
      while (can_read) {
        std::string id; zmq_ext::recv(s, id);
        std::string v; zmq_ext::recv(s, v);
        if (v == IMAGEBABBLE_DISCOVERY_PROTO_VERSION) {
          std::string req; zmq_ext::recv(s, req);
          discovery_info di; zmq_ext::recv(s, di);

          if (req == "register") {
            zmq_ext::send(s, id, ZMQ_SNDMORE);
            zmq_ext::send(s, register_server(di));
          } else if (req == "find") {
            std::vector<discovery_info> results = discover_servers(di);
            zmq_ext::send(s, id, ZMQ_SNDMORE);
            zmq_ext::send(s, !results.empty(), ZMQ_SNDMORE);
            zmq_ext::send_array(s, results, results.size());
          } else if (req == "unregister") {
            zmq_ext::send(s, id, ZMQ_SNDMORE);
            zmq_ext::send(s, unregister_server(di));
          }
        }
        can_read = zmq_ext::is_data_pending(s, 0);
      }
    }

  private:

    std::vector<discovery_info> discover_servers(const discovery_info &di) 
    {
      std::vector<discovery_info> found;
      std::vector<discovery_info>::iterator iter;
      for (iter = _servers.begin(); iter != _servers.end(); ++iter) {
        if (di.get_name() == iter->get_name() &&
            di.get_protocol_type() == iter->get_protocol_type() &&
            di.get_protocol_version() == iter->get_protocol_version())
        {
          found.push_back(*iter);
        }
      }

      return found;
    }

    bool register_server(const discovery_info &di) {
      _servers.push_back(di);
      return true;
    }

    bool unregister_server(const discovery_info &di) 
    {
      _servers.erase(
        std::remove_if(
          _servers.begin(), _servers.end(), 
          [&di](const discovery_info &x) { return x.get_address() == di.get_address(); }),
       _servers.end());
      return true;
    }

    std::vector<discovery_info> _servers;
  };

  /** A network entity that talks to a discovery server. */
  class discovery_client : public network_entity {
  public:
    
    /** Construct from context */
    discovery_client()
      : network_entity(context_ptr(new zmq::context_t(1)))
    {} 

    /** Destructor. */
    ~discovery_client()
    {
      shutdown();
    }

    /** Start a new discovery service on the given endpoint. Any previous active connection will be closed. */
    inline void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {  
      shutdown();
      _s = socket_ptr(new zmq::socket_t(*_ctx, ZMQ_DEALER));
      _s->connect(addr.c_str());
    }
    
    /** Shutdown server. */
    inline void shutdown()
    {
      if (_s && _s->connected()) {
        _s->close();
      }      
    }

    
    /** Retrieve the protocol type spoken */
    virtual e_protocol get_protocol_type() const
    {
      return PROTO_DISCOVERY;
    }

    /** Retrieve the protocol version */
    virtual std::string get_protocol_version() const
    {
      return IMAGEBABBLE_DISCOVERY_PROTO_VERSION;
    }

    inline bool register_server(const discovery_info &di, int timeout = -1) {
      zmq::socket_t &s = *_s;

      zmq_ext::send(s, get_protocol_version(), ZMQ_SNDMORE);
      zmq_ext::send(s, "register", ZMQ_SNDMORE);      
      zmq_ext::send(s, di);
      
      if (zmq_ext::is_data_pending(s, timeout)) {
        bool ok;
        zmq_ext::recv(s, ok);
        return ok;
      } else {
        return false;
      }
    }

    inline bool unregister_server(const discovery_info &di, int timeout = -1) 
    {
      zmq::socket_t &s = *_s;

      zmq_ext::send(s, get_protocol_version(), ZMQ_SNDMORE);
      zmq_ext::send(s, "unregister", ZMQ_SNDMORE);
      zmq_ext::send(s, di);
      
      if (zmq_ext::is_data_pending(s, timeout)) {
        bool ok;
        zmq_ext::recv(s, ok);
        return ok;
      } else {
        return false;
      }
    }

    inline bool find_servers(const discovery_info &di, std::vector<discovery_info> &results, int timeout = -1) 
    {
      zmq::socket_t &s = *_s;

      zmq_ext::send(s, get_protocol_version(), ZMQ_SNDMORE);
      zmq_ext::send(s, "find", ZMQ_SNDMORE);
      zmq_ext::send(s, di);
      
      if (zmq_ext::is_data_pending(s, timeout)) {
        bool ok;
        zmq_ext::recv(s, ok);
        zmq_ext::recv_array(s, results, (std::numeric_limits<size_t>::max)());
        return ok;
      } else {
        return false;
      }
    }
  };

  inline discovery_info make_discovery_info(const std::string &name, const basic_image_server &s) 
  {
    discovery_info di;

    di.set_name(name);
    di.set_protocol_type(s.get_protocol_type());
    di.set_protocol_version(s.get_protocol_version());
    di.set_address(s.get_address());    

    return di;
  }

  inline discovery_info make_discovery_info(const std::string &name, const basic_image_client &s) 
  {
    discovery_info di;

    di.set_name(name);
    di.set_protocol_type(s.get_protocol_type());
    di.set_protocol_version(s.get_protocol_version());
    di.set_address("unused");

    return di;
  }

}

#endif