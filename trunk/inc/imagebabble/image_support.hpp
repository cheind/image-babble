/*! \file image_support.hpp

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

#ifndef __IMAGE_BABBLE_IMAGE_SUPPORT_HPP_INCLUDED__
#define __IMAGE_BABBLE_IMAGE_SUPPORT_HPP_INCLUDED__

#include "core.hpp"

namespace imagebabble {

  class image;
  namespace io {
    bool send(zmq::socket_t &, const image &, int);
    bool recv(zmq::socket_t &, image &);
  };
  
  /** Image datatype */
  class image {
  public:
    struct share_mem {};
    struct copy_mem {};

    /** Construct an empty image data buffer */
    inline image()
      : _w(0), _h(0), _bbp(0), _shared_mem(false)
    {}

    /** Construct a data buffer by copying from the given buffer. 
      * The constructor does not copy the data but instead increase its
      * reference count. */
    inline image(const image &other) 
      : _w(other._w), _h(other._h), _bbp(other._bbp), _shared_mem(other._shared_mem)
    {
      _msg.copy(const_cast<zmq::message_t*>(&other._msg));
    }
  
    /** Construct a data buffer from existing memory. The implementation
      * does not take ownership of the passed memory bock. Freeing it is
      * a responsibility of the caller. */
    inline explicit image(int w, int h, int bbp, void *data, const share_mem &) 
      : _msg(data, w*h*bbp, null_deleter, 0), _w(w), _h(h), _bbp(bbp), _shared_mem(true)
    {}

    /** Construct a data buffer from existing memory.*/
    inline explicit image(int w, int h, int bbp, void *data, const copy_mem &) 
      : _msg(w*h*bbp), _w(w), _h(h), _bbp(bbp), _shared_mem(false)
    {
      memcpy(_msg.data(), data, w*h*bbp);      
    }

#ifdef IMAGEBABBLE_HAS_RVALUE_REFS

    /** Move constructor */
    inline image(image &&rhs)
      : _msg(std::move(rhs._msg)), _shared_mem(rhs._shared_mem), _w(rhs._w), _h(rhs._h), _bbp(rhs._bbp)
    {}

    /** Move assignment operator */
    inline image& operator=(image &&rhs)
    {
      if (this != &rhs) {
        _msg = std::move(rhs._msg);
        _shared_mem = rhs._shared_mem;
        _w = rhs._w;
        _h = rhs._h;
        _bbp = rhs._bbp;
      }
      return *this;
    }
#endif

    /** Assign from other buffer. The implementation does not copy the data 
      * but instead increase its reference count. */
    inline image &operator=(const image &rhs) 
    {
      if (this != &rhs) {
        _msg.copy(const_cast<zmq::message_t*>(&rhs._msg));
        _shared_mem = rhs._shared_mem;
        _w = rhs._w;
        _h = rhs._h;
        _bbp = rhs._bbp;
      }
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
    inline size_t size() const 
    {
      return _msg.size();
    }

    /** Get resolution in x dimension */
    inline int get_width() const { return _w; }
    /** Get resolution in y dimension */
    inline int get_height() const { return _h; }
    /** Get number of bytes per channel channels */
    inline int get_bytes_per_pixel() const { return _bbp; }

    inline void copy_to(void *dst) const 
    {
      memcpy(dst, _msg.data(), size());
    }

  private:

    friend bool io::send(zmq::socket_t &, const image &, int);
    friend bool io::recv(zmq::socket_t &, image &);

    /** Null deleter in case of supplied user memory. */
    static inline void null_deleter (void *data, void *hint)
    {}

    zmq::message_t _msg;
    int _w, _h, _bbp;
    bool _shared_mem;
  };
  
  namespace io {
    
    /** Send image. */
    inline bool send(zmq::socket_t &s, const image &v, int flags = 0) 
    {
      bool all_ok = true;
      
      all_ok &= io::send(s, v.get_width(), ZMQ_SNDMORE);
      all_ok &= io::send(s, v.get_height(), ZMQ_SNDMORE);
      all_ok &= io::send(s, v.get_bytes_per_pixel(), ZMQ_SNDMORE);
      
      // Need to copy in order to increment reference count, 
      // otherwise input buffer is nullified.
      
      zmq::message_t m;
      m.copy(const_cast<zmq::message_t*>(&v._msg));

      all_ok &= s.send(m, flags);

      return all_ok;
    }

    /** Receive image data. If image data points to pre-allocated user memory,
      * the implementation attempts to receive data directly into that buffer.
      * If the buffer is too small to fit the content, the received bytes are
      * truncated to fit and false is returned. */
    inline bool recv(zmq::socket_t &s, image &v) 
    {

      bool all_ok = true;

      all_ok &= io::recv(s, v._w);
      all_ok &= io::recv(s, v._h);
      all_ok &= io::recv(s, v._bbp);

      if (v._shared_mem) {
        int bytes = zmq_recv(s, v._msg.data(), v._msg.size(), 0);
        all_ok &= (bytes <= static_cast<int>(v._msg.size()));      
      } else {
        all_ok &= s.recv(&v._msg);
      }

      return all_ok;
    }


  }

}




#endif