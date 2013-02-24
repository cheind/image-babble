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
#include <string>
#include <vector>
#include <exception>

namespace imagebabble {

  class image;
  class image_group;

  namespace io {
    void send(zmq::socket_t &, const image &, int);
    void recv(zmq::socket_t &, image &);
    void send(zmq::socket_t &, const image_group &, int);
    void recv(zmq::socket_t &, image_group &);
  };
  
  /** Represents a generic image. An image consists of basic header information
    * and a data buffer. The header describes the image format using common fields
    * such as 
    *  - \a width number of columns in image 
    *  - \a height number of rows in image
    *  - \a step the number of bytes between two subsequent rows.
    *  - \a type opaque type information. 
    *
    * \note the \a type field is application dependent. If the sender uses a specific
    *       type value, the client should know its meaning and act accordingly. For example when 
    *       sending/receiving OpenCV images the type could be CV_8UC3 (3 channels, 1 byte per channel).
    *
    * \note the \a step field has to be calculated by the user of the library as the number of 
    *       bytes between two rows. For example, assuming a 640x480 image with 3 channels and 1 byte
    *       per channel, step would be calculated as <code>step = 640*3*1</code>. The step field can
    *       also be used to describe non-continous images in memory.
    * 
    * The data buffer is a block of memory. By default the memory is allocated when the image header
    * information is provided. Memory allocated by the library is auto-freed using reference counting,
    * when the last image pointing to the same memory goes out of scope and ZMQ is done with pending
    * send operations. 
    *
    * Alternatively, the user can instruct the library to use pre-existing memory by using specialized
    * constructors. Such user passed memory is not freed by library and it maintains the responsibility
    * of the caller to free it. A custom free function can be stored inside a shared_mem to tell the library 
    * to call the given function when the user data buffer is not needed anymore. The default behavior of 
    * shared_mem is call no callback.
    *
    * \note Copying images by value will not cause the memory to be duplicated, but instead the respective
    *       reference count will be increased.
    */
  class image {
  public:

    /** Predefined image format. If the given formats are not
      * suited, choose FORMAT_UNKOWN and set the external image type
      * via image::set_external_type. */
    enum eformat {
      /** Unknown image format */
      FORMAT_UNKNOWN,
      /** RGB format using 8 bits per channel. */
      FORMAT_RGB_888,
      /** BGR format using 8 bits per channel. */
      FORMAT_BGR_888,
      /** Grayscale image using 8 bit channel. */
      FORMAT_GRAY_8
    };

    /** Free function prototype when sharing user memory */
    typedef void free_fn(void *data, void *hint);

    /** Construct a new image. */
    inline image()
      : _w(0), _h(0), _step(0), _external_type(-1), _format(FORMAT_UNKNOWN), _shared_mem(false)
    {}

    /** Construct a new image. The implementation will copy the header 
      * information and share the buffer by incrementing its reference count. */
    inline image(const image &other) 
      : _w(other._w), _h(other._h), _step(other._step), 
      _external_type(other._external_type), 
      _format(other._format), 
      _shared_mem(other._shared_mem)
    {
      _msg.copy(const_cast<zmq::message_t*>(&other._msg));
    }

    /** Construct a new image. Allocates the necessary image data buffer size. */
    inline explicit image(int w, int h, int step) 
      : _msg(h*step), _w(w), _h(h), _step(step), _external_type(-1), _format(FORMAT_UNKNOWN), _shared_mem(false)
    {}
  
    /** Construct a new image. The implementation does not take ownership of the passed 
      * bock. Freeing it is a responsibility of the caller. The implementation will ensure
      * that any custom free function of share_mem is being called. */
    inline explicit image(int w, int h, int step, void *data, const share_mem &s) 
      : _msg(data, h*step, s.get_free_fn(), s.get_hint()), _w(w), _h(h), _step(step), _external_type(-1), _format(FORMAT_UNKNOWN), _shared_mem(true)
    {}

    /** Construct a new image. The implementation will copy the data given. The newly
      * allocated buffer will be released when its reference count hits zero. */
    inline explicit image(int w, int h, int step, void *data, const copy_mem &) 
      : _msg(h*step), _w(w), _h(h), _step(step), _external_type(-1), _format(FORMAT_UNKNOWN), _shared_mem(false)
    {
      memcpy(_msg.data(), data, _msg.size());      
    }

#ifdef IB_HAS_RVALUE_REFS

    /** Construct a new image. Renders the source invalid. */
    inline image(image &&rhs)
      : _msg(std::move(rhs._msg)), 
        _shared_mem(rhs._shared_mem), 
        _w(rhs._w), _h(rhs._h), 
        _external_type(rhs._external_type), 
        _format(rhs._format),
        _step(rhs._step)
    {}

    /** Move assignment operator. Renders the source invalid. */
    inline image& operator=(image &&rhs)
    {
      if (this != &rhs) {
        _msg = std::move(rhs._msg);
        _shared_mem = rhs._shared_mem;
        _w = rhs._w;
        _h = rhs._h;
        _external_type = rhs._external_type;
        _format = rhs._format;
        _step = rhs._step;
      }
      return *this;
    }
#endif

    /** Assign from other buffer. The implementation will copy the header 
      * information and share the buffer by incrementing its reference count. */
    inline image &operator=(const image &rhs) 
    {
      if (this != &rhs) {
        _msg.copy(const_cast<zmq::message_t*>(&rhs._msg));
        _shared_mem = rhs._shared_mem;
        _w = rhs._w;
        _h = rhs._h;
        _step = rhs._step;
        _external_type = rhs._external_type;
        _format = rhs._format;
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

    /** Get the number of pixels in width. */
    inline int get_width() const 
    { 
      return _w; 
    }

    /** Get the number of pixels in height. */
    inline int get_height() const 
    { 
      return _h; 
    }

    /** Get number of bytes per pixel. */
    inline int get_step() const 
    { 
      return _step; 
    }

    /** Get external type information. The external type information
      * is of informative usage to the caller only. No calculations are
      * performed upon it internally. */
    inline int get_external_type() const 
    { 
      return _external_type; 
    }

    /** Set external type information. The external type information
      * is of informative usage to the caller only. No calculations are
      * performed upon it internally. */
    inline void set_external_type(int type) 
    { 
      _external_type = type; 
    }

    /** Get image format information. Setting the format helps the implementation
      * to take correct decisions such as conversion. */
    inline eformat get_format() const 
    { 
      return _format; 
    }

    /** Set image format information. Setting the format helps the implementation
      * to take correct decisions such as conversion.*/
    inline void set_format(eformat f) 
    { 
      _format = f; 
    }

    /** Copy image data buffer to given destination. */
    inline void copy_to(void *dst) const 
    {
      memcpy(dst, _msg.data(), size());
    }

  private:

    friend void io::send(zmq::socket_t &, const image &, int);
    friend void io::recv(zmq::socket_t &, image &);

    zmq::message_t _msg;
    int _w, _h, _step, _external_type;
    eformat _format;
    bool _shared_mem;
  };
  
  /** A collection of images to be sent/received at once. */
  class image_group {
  public:

    /** Construct a new image group. */
    inline image_group() 
    {}

    /** Construct a new image group. */
    inline image_group( const std::string &id) 
      : _id(id)
    {}

#ifdef IB_HAS_RVALUE_REFS

    /** Construct a new image group. */
    inline image_group(image_group &&rhs)
      : _images(std::move(rhs._images)), _names(rhs._names), _id(rhs._id)
    {}

    /** Move assignment operator.  */
    inline image_group& operator=(image_group &&rhs)
    {
      if (this != &rhs) {
        _images = std::move(rhs._images);
        _names = std::move(rhs._names);
        _id = std::move(rhs._id);        
      }
      return *this;
    }

    /** Move append named image. */
    inline void add_image(image &&i, std::string &&name = std::string())
    {
      _images.push_back(i);
      _names.push_back(name);
    }

#endif

    /** Append a named image. */
    inline void add_image(const image &i, const std::string &name = std::string())
    {
      _images.push_back(i);
      _names.push_back(name);
    }

    /** Clear images and names. */
    inline void clear() 
    {
      _images.clear();
      _names.clear();
    }

    /** Get the number of named images */
    inline size_t size() const 
    {
      return _names.size();
    }

    /** Get the image collection. */
    inline const std::vector<image> &get_images() const
    {
      return _images;
    }

    /** Get the image collection. */
    inline std::vector<image> &get_images() 
    {
      return _images;
    }

    /** Get the image name collection. */
    inline const std::vector<std::string> &get_names() const
    {
      return _names;
    }

    /** Get the image name collection. */
    inline std::vector<std::string> &get_names() 
    {
      return _names;
    }

    /** Get the image group identifier. */
    inline const std::string &get_id() const 
    {
      return _id;
    }

    /** Get the image group identifier. */
    inline std::string &get_id() 
    {
      return _id;
    }

    /** Set the image group identifier. */
    inline void set_id(const std::string &id) 
    {
      _id = id;
    }

    /** Find a named image. Finds the first image with the given name.
      * Search is case sensitive.
      *
      * \param[in] name name of image
      * \returns pointer to image when found
      * \return null-pointer when no such image is found. 
      */
    inline image *find_named_image(const std::string &name) 
    {
      // Invoke the const version, so we have to write the algorithm only once. */
      const image_group &self = *this;
      return const_cast< image* >(self.find_named_image(name));      
    }

    /** Find a named image. Finds the first image with the given name.
      * Search is case sensitive.
      *
      * \param[in] name name of image
      * \returns pointer to image when found
      * \return null-pointer when no such image is found. 
      */
    inline const image *find_named_image(const std::string &name) const
    {
      std::vector<std::string>::const_iterator iter = std::find(_names.begin(), _names.end(), name);
      if (iter == _names.end()) 
        return 0;

      size_t id = static_cast<size_t>(std::distance(_names.begin(), iter));
      return &_images[id];
    }

  private:
    
    friend void io::send(zmq::socket_t &, const image_group &, int);
    friend void io::recv(zmq::socket_t &, image_group &);

    std::vector<image> _images;
    std::vector<std::string> _names;
    std::string _id;
  };


  namespace io {
    
    /** Send image. */
    inline void send(zmq::socket_t &s, const image &v, int flags = 0) 
    { 
      std::ostringstream ostr;
      ostr << v.get_width() << " "
           << v.get_height() << " "
           << v.get_step() << " "
           << v.get_external_type() << " "
           << v.get_format();

      IB_ASSERT(ostr.good(), ib_error::ECONVERSION);
      io::send(s, ostr.str(), ZMQ_SNDMORE);
      
      // Need to copy in order to increment reference count, otherwise the 
      // input buffer is nullified.
      
      zmq::message_t m;
      m.copy(const_cast<zmq::message_t*>(&v._msg));

      IB_ASSERT_ZMQ(s.send(m, flags));
    }

    /** Receive image data. If image data points to pre-allocated user memory,
      * the implementation attempts to receive data directly into that buffer.
      * If the buffer is too small to fit the content, the received bytes are
      * truncated to fit and false is returned. */
    inline void recv(zmq::socket_t &s, image &v) 
    {
      zmq::message_t msg;
      IB_ASSERT_ZMQ(s.recv(&msg));

      in_memory_buffer mb(static_cast<char*>(msg.data()), msg.size());
      std::istream is(&mb);

      is  >> v._w 
          >> v._h
          >> v._step
          >> v._external_type
          >> reinterpret_cast<int&>(v._format);

      IB_ASSERT(!is.fail(), ib_error::ECONVERSION);

      if (v._shared_mem) {
        int bytes = zmq_recv(s, v._msg.data(), v._msg.size(), 0);
        int maxbytes = static_cast<int>(v._msg.size());
        IB_ASSERT(bytes <= maxbytes, ib_error::EBUFFERTOOSMALL);
      } else {
        IB_ASSERT_ZMQ(s.recv(&v._msg));
      }
    }

    /** Send image group. */
    inline void send(zmq::socket_t &s, const image_group &v, int flags = 0) 
    {     
      io::send(s, v.get_id(), ZMQ_SNDMORE);
      io::send(s, v.get_names(), ZMQ_SNDMORE);
      io::send(s, v.get_images(), flags);
    }

    /** Receive image group. */
    inline void recv(zmq::socket_t &s, image_group &v) 
    {
      io::recv(s, v._id);
      io::recv(s, v._names);
      io::recv(s, v._images);
    }
  }

  /** Generic image conversion. Specializations of this method handle
    * conversion of external image types to and from internal image types.
    * Such conversion implementations should be implemented in separate files
    * that are not included by default. */
  template<class From, class To, class MemOp>
  void cvt_image(const From &src, To &to, const MemOp&) 
  {

    /* If your compiler gets trapped here, it means that no suitable
     * image conversion function was found by your compiler. Either it does
     * not exist or is not included. */
    static_assert(false);
  }

  /** Generic image conversion. Default implementation uses conversion methods
    * to imitade return by value behaviour of the output. */
  template<class To, class From, class MemOp>
  To cvt_image(const From &src, const MemOp &m) 
  {
    To to;
    cvt_image(src, to, m);
    return to;
  }

}




#endif