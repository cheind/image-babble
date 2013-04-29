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

#ifndef __IMAGE_BABBLE_RELIABLE_HPP_INCLUDED__
#define __IMAGE_BABBLE_RELIABLE_HPP_INCLUDED__

#include "core.hpp"
#include <unordered_map>

#define IB_EXCHANGE_PROTO_RELIABLE_REGISTER "client_register"
#define IB_EXCHANGE_PROTO_RELIABLE_ACK "client_ack"
#define IB_EXCHANGE_PROTO_RELIABLE_PAYLOAD "server_payload"
#define IB_EXCHANGE_PROTO_RELIABLE_DISCONNECT "disconnect"

namespace imagebabble {

  /** Reliable server implementation. The reliable server implementation is based
    * on data acknowledgement. It is reliable in the term that no data is lost due 
    * to filled queues on both ends.
    * 
    * When new data is to be published, the server first waits for registration of
    * the necessary number of clients. It then sends the message to all registered
    * clients and waits for ACKS. 
    * 
    * A timeout may be passed to to the publish process in which case the server 
    * might end the publishing preliminarily.
    */
  template<typename T>
  class reliable_server : public basic_server<T> {
  public:

    /** Default constructor. */
    reliable_server()
      : basic_server<T>(context_ptr(new zmq::context_t(1)))
      , _next_id(0)
    {}

    /** Destructor. */
    virtual ~reliable_server()
    {}

    /** Start a new connection on the given endpoint. Multiple enpoints are
      * not supported. Calling this method more than once will cause previous
      * connections to be dropped.
      *
      * \param[in] addr address to bind to.
      * \throws ib_error on error
      */
    virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {  
      if (!network_entity::_s) {
        network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx, ZMQ_ROUTER));
        network_entity::apply_socket_options();
      } else {
        basic_server<T>::shutdown();
        startup(addr);
      }

      IB_CATCH_ZMQ_RETHROW(network_entity::_s->bind(addr.c_str()));
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
      * \param [in] min_serve minimum number of clients to service.
      * \param [in] timeout_ms maximum wait time in milliseconds.
      * \returns true when data was published successfully
      * \returns false when a send timeout occurred.
      * \throws ib_error on error.
      **/
    virtual bool publish(const T &t, int timeout_ms = -1, size_t min_serve = 1)
    {
      IB_ASSERT(network_entity::_s, ib_error::EINVALIDSOCKET);

      timeout tout(timeout_ms);
      
      bool new_data = false;      
      do {
        // Read off all available registrations
        do {
          new_data = recv_from_client(ZMQ_DONTWAIT);
        } while (new_data);
       
        // No more data, see if we should wait for more
        int timeleft = tout.timeleft();
        if ((_clients.size() < min_serve) && timeout::is_timeleft(timeleft)) {          
          new_data = io::is_data_pending(*network_entity::_s, timeleft);          
        }

      } while (new_data);

      // Drop unresponsive clients
      disconnect_unresponsive_clients(_next_id - 10);

      // Test if there are enough clients
      if (_clients.size() < min_serve) {
        return false;
      }

      // Send data
      for (client_map::iterator i = _clients.begin(); i != _clients.end(); ++i) {
        send_client_payload(i->first, _next_id, t);          
      }

      // Wait for ACKs
      do {
        do {
          new_data = recv_from_client(ZMQ_DONTWAIT);
        } while (new_data);
       
        // No more data, see if we should wait for more
        int timeleft = tout.timeleft();
        if ((count_acks(_next_id) < _clients.size()) && timeout::is_timeleft(timeleft)) {
          new_data = io::is_data_pending(*network_entity::_s, timeleft);          
        }

      } while (new_data);

      return count_acks(_next_id++) == _clients.size();
    }

  private:
    typedef std::unordered_map<std::string, long> client_map;    

    /** Receive from a single client */
    bool recv_from_client(int flags) {
      std::string address, version, type;
      long id;

      IB_FIRST_PART(io::recv(*network_entity::_s, address, flags));
      IB_NEXT_PART(io::recv(*network_entity::_s, version, flags));
      network_entity::validate_version(IB_EXCHANGE_PROTO_RELIABLE_VERSION, version);
      IB_NEXT_PART(io::recv(*network_entity::_s, type, flags));

      if (type == IB_EXCHANGE_PROTO_RELIABLE_REGISTER) {
        _clients[address] = -1;
      } else if (type == IB_EXCHANGE_PROTO_RELIABLE_DISCONNECT) {
        _clients.erase(address);
      } else if (type == IB_EXCHANGE_PROTO_RELIABLE_ACK) {
        IB_NEXT_PART(io::recv(*network_entity::_s, id, flags));
        client_map::iterator iter = _clients.find(address);
        if (iter != _clients.end() && iter->second < id) {
          iter->second = id;
        }
      }

      return true;
    }

    /** Count ACKs for given id */
    size_t count_acks(long id) const {
      size_t count = 0;
      client_map::const_iterator iter;

      for (iter = _clients.begin(); iter != _clients.end(); ++iter) {
        if (iter->second == id)
          ++count;
      }

      return count;
    }

    /** Disconnect clients that fail to ACK */
    void disconnect_unresponsive_clients(long threshold) {
      client_map::iterator iter;

      for (iter = _clients.begin(); iter != _clients.end();) {
        if (iter->second < threshold) {
          send_client_disconnect(iter->first);
          iter = _clients.erase(iter);
        } else {
          ++iter;
        }
      }
    }

    /** Send payload to client */
    bool send_client_payload(const std::string &addr, long id, const T &t)
    {
      IB_FIRST_PART(io::send(*network_entity::_s, addr, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_VERSION, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_PAYLOAD, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, id, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, t, 0));

      return true;
    }

    /** Send disconnect to client */
    bool send_client_disconnect(const std::string &addr)
    {
      IB_FIRST_PART(io::send(*network_entity::_s, addr, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_VERSION, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_DISCONNECT, 0));
      
      return true;
    }

    client_map _clients;
    long _next_id;
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

    /** Disconnect from server. */
    virtual void shutdown() {
      if (network_entity::_s) {
        send_disconnect(ZMQ_DONTWAIT);
      }
      basic_client<T>::shutdown();
    }

    /** Start a new connection to the given endpoint. If called multiple times
      * will disconnect previous connections.
      *
      * \param [in] addr endpoint address
      * \throws ib_error on error 
      */
    virtual void startup(const std::string &addr = "tcp://127.0.0.1:6000")
    {
      if (!network_entity::_s) {
        network_entity::_s = socket_ptr(new zmq::socket_t(*network_entity::_ctx, ZMQ_DEALER));     
        network_entity::apply_socket_options();
      } else {
        shutdown();
        startup(addr);
      } 
      
      IB_CATCH_ZMQ_RETHROW(network_entity::_s->connect(addr.c_str()));
      _addr = addr;
      send_registration(0);
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
      IB_ASSERT(network_entity::_s, ib_error::EINVALIDSOCKET);

      io::ensure_cleanup_partial_messages ecpm(this->get_socket());      

      // Try receiving data
      std::string version, type;

      if (!io::recv(*network_entity::_s, version, ZMQ_DONTWAIT)) {
        if (timeout_ms == 0 || !io::is_data_pending(*network_entity::_s, timeout_ms)) {
          return false;
        }
        IB_FIRST_PART(io::recv(*network_entity::_s, version, ZMQ_DONTWAIT));
      } 

      network_entity::validate_version(IB_EXCHANGE_PROTO_RELIABLE_VERSION, version);
      IB_NEXT_PART(io::recv(*network_entity::_s, type, ZMQ_DONTWAIT));

      if (type == IB_EXCHANGE_PROTO_RELIABLE_PAYLOAD) {
        std::string id;
        IB_NEXT_PART(io::recv(*network_entity::_s, id, ZMQ_DONTWAIT));        
        send_ack(id, 0);
        IB_NEXT_PART(io::recv(*network_entity::_s, t, ZMQ_DONTWAIT));        
        return true;
      } else if (type == IB_EXCHANGE_PROTO_RELIABLE_DISCONNECT) {
        startup(_addr);
        return false;
      } else {
        return false;
      }

    }

  private:

    // Send registration to server
    bool send_registration(int flags) {
      IB_FIRST_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_VERSION, ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_REGISTER, flags));
      return true;
    }

    // Send disconnect to server
    bool send_disconnect(int flags) {
      IB_FIRST_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_VERSION, ZMQ_SNDMORE));      
      IB_NEXT_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_DISCONNECT, flags));
      return true;
    }

    // Send ACK to server
    bool send_ack(const std::string &id, int flags) {
      IB_FIRST_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_VERSION, ZMQ_SNDMORE));      
      IB_NEXT_PART(io::send(*network_entity::_s, IB_EXCHANGE_PROTO_RELIABLE_ACK, flags | ZMQ_SNDMORE));
      IB_NEXT_PART(io::send(*network_entity::_s, id, flags));
      return true;
    }

    std::string _addr;
  };

}

#endif