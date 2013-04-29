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

#ifndef __IMAGE_BABBLE_FAST_HPP_INCLUDED__
#define __IMAGE_BABBLE_FAST_HPP_INCLUDED__

#include "core.hpp"

namespace imagebabble {

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
      if (!network_entity::_s) {
        network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx, ZMQ_PUB));
        network_entity::apply_socket_options();
      }

      IB_CATCH_ZMQ_RETHROW(network_entity::_s->bind(addr.c_str()));
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
      IB_ASSERT(network_entity::_s, ib_error::EINVALIDSOCKET);
      io::send(*network_entity::_s, IB_EXCHANGE_PROTO_FAST_VERSION, ZMQ_SNDMORE);
      io::send(*network_entity::_s, t, 0);
      return true;
    }
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
      if (!network_entity::_s) {
        network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx, ZMQ_SUB));
      
        network_entity::apply_socket_options();
        IB_CATCH_ZMQ_RETHROW(network_entity::_s->setsockopt(ZMQ_SUBSCRIBE, 0, 0));

        size_t recvhwm_size = sizeof (_recv_skip);
        IB_CATCH_ZMQ_RETHROW(network_entity::_s->getsockopt(ZMQ_RCVHWM, &_recv_skip, &recvhwm_size));

      }

      IB_CATCH_ZMQ_RETHROW(network_entity::_s->connect(addr.c_str()));
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
      IB_ASSERT(network_entity::_s, ib_error::EINVALIDSOCKET);

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
      if (has_wait && io::is_data_pending(*network_entity::_s, timeout_ms)) {
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

      IB_FIRST_PART(io::recv(*network_entity::_s, version, flags));
      network_entity::validate_version(IB_EXCHANGE_PROTO_FAST_VERSION, version);
      IB_NEXT_PART(io::recv(*network_entity::_s, t, flags));

      return true;
    }

    bool _enable_skip;
    int _recv_skip;
  };
}

#endif