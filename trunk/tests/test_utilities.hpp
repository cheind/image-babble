/*! \file test_utilities.hpp

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


#include <imagebabble/core.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <functional>

/** Utility class to send and receive data via a reliable connection. */
class send_receive {
public:
  typedef boost::function<void(imagebabble::reliable_server &s)> srv_handler;
  typedef boost::function<void(imagebabble::reliable_client &c)> clt_handler;

  inline void set_send(srv_handler h) 
  { 
    _s = h;
  }

  inline void set_recv(clt_handler h)
  {
    _c = h;
  }

  inline bool run() {
    _g.join_all();

    bool srv_ok = true;
    bool clt_ok = true;

    srv_handler sh = _s;
    _g.create_thread([sh, &srv_ok](){
      imagebabble::reliable_server srv;
      try {
        srv.startup();
        sh(srv);        
        srv.shutdown();
      } catch (...) {
        srv_ok = false;
      }      
    });

    clt_handler ch = _c;
    _g.create_thread([ch, &clt_ok](){
      imagebabble::reliable_client clt;
      try {
        clt.startup();
        ch(clt);
        clt.shutdown();
      } catch (...) {
        clt_ok = false;
      }
    });

    _g.join_all();

    return srv_ok && clt_ok;

  }


private:
  srv_handler _s;
  clt_handler _c;
  boost::thread_group _g;
};