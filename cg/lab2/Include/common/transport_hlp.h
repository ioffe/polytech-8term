#pragma once 

#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>

#include "transport.h"    

//////////////////////////////////////////////////////////////////////////
// DECLARATION 

typedef boost::function<void ()>          net_event_t ;  
typedef boost::function<void ( int ) >    net_error_t ;

//////////////////////////////////////////////////////////////////////////
// net_connector
struct net_connector
{
   typedef boost::function<void (SOCKET)> connected_event_t ; 

   net_connector( ulong addr, ushort port, connected_event_t on_connected, net_event_t on_failed = NULL, net_error_t on_error = NULL ) ;
  ~net_connector() ;

   void reconnect () ;

private :
   class Impl ;
   Impl * pImpl_ ;
};

//////////////////////////////////////////////////////////////////////////
// net_stream
template< class SocketStream >
struct net_stream 
{
   typedef boost::function<void ( size_t size, void const* ) > receive_event_t ;

   net_stream ( SOCKET sock, receive_event_t on_receive, net_event_t on_disconnected, net_error_t on_error = NULL ) ;
  ~net_stream();

   void send        ( size_t size, void const * data ) ;
   int  set_no_delay( bool nodelay = true ) ;
   bool is_connected() const ;

private:
   class Impl ;
   Impl * pImpl_ ; 
};

typedef net_stream<TFragmentStream> net_fragment_stream ; 
typedef net_stream<TSockStream>     net_socket_stream ; 

//////////////////////////////////////////////////////////////////////////
// net_connecting_stream 
template< class SocketStream >
struct net_connecting_stream 
{
   typedef boost::function<void ( size_t size, void const* ) >  receive_event_t ;

   net_connecting_stream ( ulong addr, ushort port, 
                           receive_event_t on_receive, net_event_t on_connected, 
                           net_event_t on_diconnected, net_event_t on_failed = NULL, net_error_t on_error = NULL ) ;

   void send ( size_t size, void const * data ) ;
   void reconnect () ; 

   bool connected () const ; 
   int  set_no_delay( bool nodelay = true ) ;
   bool is_connected() const ;

private:
   void on_connected ( SOCKET sock ) ; 

private:
   net_event_t                                   connected_event_    ;
   net_event_t                                   disconnected_event_ ;
   receive_event_t                               receive_event_      ;
   net_error_t                                   error_event_        ;
   
   net_connector                                 connector_ ;
   boost::scoped_ptr<net_stream<SocketStream> >  stream_    ;
};

typedef net_connecting_stream<TFragmentStream> net_fragment_connecting_stream ; 
typedef net_connecting_stream<TSockStream>     net_socket_connecting_stream ; 

//////////////////////////////////////////////////////////////////////////
// net_listener

struct net_listener : TListener // no need in impl
{
   typedef boost::function< bool ( SOCKET, ulong ) >   accepted_event_t ;  // ip address and socket  

   net_listener( ulong addr, ushort port, accepted_event_t on_accepted, net_error_t on_error = NULL ) ;
   
   TINETAddr   get_connect_name() const ;
   bool        valid           () const ;
   
// TListener 
private:
   void OnAccepted ( SOCKET ) ;
   void SocketError( int code ) const    ;

private: 
   bool             valid_          ;
   accepted_event_t accepted_event_ ;
   net_error_t      error_event_    ;
};


//////////////////////////////////////////////////////////////////////////
// net_pinger 
struct net_pinger 
{
   net_pinger( LPCSTR host_name, net_event_t on_echo, net_event_t on_timeout,  
                                 net_event_t on_ping_failed, net_error_t on_error = NULL, ulong subnet = 0, ulong subnet_mask = 0 ) ;
  ~net_pinger() ;

private:
   struct Impl  ;
   Impl* pImpl_ ;
};

//////////////////////////////////////////////////////////////////////////
// net_broadcaster 
struct net_broadcaster : TSingleDestDgram // no need in impl 
{
   typedef boost::function<void ( size_t size, void const*, TINETAddr const& ) > receive_event_t ;

   net_broadcaster( receive_event_t on_receive, net_error_t on_error = NULL  ) ; // if you only want to receive 
   net_broadcaster( TINETAddr const& addr, net_error_t on_error = NULL  ) ; // if you only want to send 
   net_broadcaster( TINETAddr const& addr, receive_event_t revent, net_error_t on_error = NULL ) ;
   net_broadcaster( TINETAddr const& sendAddr, TINETAddr const& listenAddr, receive_event_t revent, net_error_t on_error = NULL ) ;

   void send( size_t size, void const * data ) ;
   int  bind( TINETAddr const& addr ) ;

   int  set_reuse  ( bool reuse = true ) ;

private:
   void OnReceive  ( size_t size, void const* data, TINETAddr const& ) ;
   void SocketError( int code ) const ;

private:
   receive_event_t receive_event_ ;
   net_error_t     error_event_   ;
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION 

// net_connector

// impl declaration 
class net_connector::Impl : public TConnector
{
public :
   Impl ( ulong addr, ushort port, connected_event_t cevent, net_event_t fevent, net_error_t on_error ) ;

private: 
   void OnConnected     ( SOCKET sock ) ;
   void OnConnectFailed () ;
   void SocketError     ( int code ) const ;

private: 
   void OnFinalMessage ( HWND ) ; 

private: 
   connected_event_t connected_event_ ;
   net_event_t       failed_event_    ;
   net_error_t       error_event_     ;
};

inline net_connector::net_connector( ulong addr, ushort port, connected_event_t cevent, net_event_t fevent, net_error_t on_error ) 
   : pImpl_ ( new Impl ( addr, port, cevent, fevent, on_error )) 
{
}

inline net_connector::~net_connector()
{
   pImpl_ -> DestroyWindow();
}

inline void net_connector::reconnect () 
{
   pImpl_ -> Reconnect();
}

inline net_connector::Impl::Impl( ulong addr, ushort port, connected_event_t cevent, net_event_t fevent, net_error_t on_error ) 
   : connected_event_ (cevent) 
   , failed_event_    (fevent)
   , error_event_     (on_error )
{
   ConnectTo( TINETAddr( addr, port )) ;
}

inline void net_connector::Impl::OnConnected ( SOCKET sock ) 
{
   if (connected_event_)
   {
      connected_event_(sock) ;
   }
   else 
   {
      closesocket(sock);
   }
}

inline void net_connector::Impl::OnConnectFailed () 
{
   if (failed_event_)
      failed_event_() ;
   else 
      TConnector::OnConnectFailed() ;
}

inline void net_connector::Impl::OnFinalMessage( HWND ) 
{
   delete this ; 
}

inline void net_connector::Impl::SocketError( int code ) const
{
   if( error_event_ )
      error_event_( code ) ;
   else     
      TConnector::SocketError( code ) ;
}

// net_stream 

// impl declaration 
template< class SocketStream >
class net_stream<SocketStream>::Impl : public SocketStream
{
public:
   Impl(SOCKET sock, receive_event_t revent, net_event_t devent, net_error_t on_error ) ;

private: 
   void OnReceive       ( size_t size, void const* data ) ;
   void OnDisconnected  () ;
   void SocketError     ( int code ) const ;

private: 
   void OnFinalMessage( HWND ) ; 

private: 
   net_event_t      disconnected_event_ ;
   receive_event_t  receive_event_      ;
   net_error_t      error_event_        ;
};

template<class SocketStream>
   net_stream<SocketStream>::net_stream ( SOCKET sock, receive_event_t revent, net_event_t devent, net_error_t on_error ) 
      : pImpl_( new Impl(sock, revent, devent, on_error ))
   {
   }

template<class SocketStream>
   net_stream<SocketStream>::~net_stream()
   {
      pImpl_ -> DestroyWindow();
   }

template<class SocketStream>
   void net_stream<SocketStream>::send ( size_t size, void const* data ) 
   {
      pImpl_ -> Send ( size, data ) ; 
   }
   
template<class SocketStream>
   int net_stream<SocketStream>::set_no_delay( bool nodelay )
{
   const BOOL nodelay_param = nodelay ;
   return pImpl_->SetOption( IPPROTO_TCP, TCP_NODELAY, &nodelay_param, sizeof(nodelay_param));
}

template<class SocketStream>
   bool net_stream<SocketStream>::is_connected() const 
{
   return pImpl_->IsConnected() ;
}

template<class SocketStream>
   net_stream<SocketStream>::Impl::Impl(SOCKET sock, receive_event_t revent, net_event_t devent, net_error_t on_error ) 
      : SocketStream          (sock)
      , receive_event_        (revent)
      , disconnected_event_   (devent)
      , error_event_          (on_error )
   {
   }

template<class SocketStream>
   void net_stream<SocketStream>::Impl::OnReceive( size_t size, void const* data ) 
   {
      if ( receive_event_ )
         receive_event_( size, data ) ; 
   }

template<class SocketStream>
   void net_stream<SocketStream>::Impl::OnDisconnected() 
   {
      if ( disconnected_event_ )
         disconnected_event_() ; 
   }


template<class SocketStream>
   void net_stream<SocketStream>::Impl::SocketError( int code ) const
{
   if( error_event_ )
      error_event_( code ) ;
   else 
      SocketStream::SocketError( code ) ;
}

template<class SocketStream>
   void net_stream<SocketStream>::Impl::OnFinalMessage( HWND )  
   {
      delete this ; 
   }

// net_connecting_stream
template< class SocketStream >
   net_connecting_stream<SocketStream>::net_connecting_stream ( ulong addr, ushort port, 
                    receive_event_t revent, net_event_t cevent, 
                    net_event_t devent, net_event_t fevent, net_error_t on_error ) 
      : connected_event_   (cevent)
      , disconnected_event_(devent)
      , receive_event_     (revent)
      , error_event_       (on_error)
      , connector_         (addr, port, boost::bind(&net_connecting_stream::on_connected, this, _1), fevent, on_error)
   {
   }

template< class SocketStream >
   void net_connecting_stream<SocketStream>::send ( size_t size, void const * data ) 
   {
      stream_ -> send ( size, data ) ;
   }

template< class SocketStream >
   void net_connecting_stream<SocketStream>::reconnect () 
   {
      if ( stream_ )
         stream_.reset();

      connector_.reconnect();
   }

template< class SocketStream >
   bool net_connecting_stream<SocketStream>::connected () const 
   {
      return stream_ ; 
   }

template< class SocketStream >
   int net_connecting_stream<SocketStream>::set_no_delay( bool nodelay )
{
   return stream_->set_no_delay( nodelay ) ;
};

template< class SocketStream >
   bool net_connecting_stream<SocketStream>::is_connected() const 
{
   return stream_->is_connected() ;
}

template< class SocketStream >
   void net_connecting_stream<SocketStream>::on_connected ( SOCKET sock ) 
{
   stream_.reset( new net_stream<SocketStream>(sock, receive_event_, disconnected_event_, error_event_)) ;

   if (connected_event_)
      connected_event_();
}

//////////////////////////////////////////////////////////////////////////
// net_listener 
inline net_listener::net_listener( ulong addr, ushort port, accepted_event_t on_accepted, net_error_t on_error ) 
   : accepted_event_ ( on_accepted )
   , error_event_    ( on_error    )
{
   valid_ = ListenAs( TINETAddr( addr, port )) != 0 ;
}

inline TINETAddr net_listener::get_connect_name() const
{
   TINETAddr addr ;
   GetSocketName( addr ) ;
   return addr ;    
}

inline bool net_listener::valid() const 
{
   return valid_ ; 
}

inline void net_listener::OnAccepted ( SOCKET sock ) 
{
   sockaddr_in addr ;
   int sz = sizeof ( addr ) ;
   if ( getpeername( sock, (PSOCKADDR)&addr, &sz ) == ERROR_SUCCESS )
   {
      if ( ! accepted_event_( sock, addr.sin_addr.S_un.S_addr ) )
         closesocket ( sock ) ;
   }
   else
      closesocket ( sock ) ;
}

inline void net_listener::SocketError( int code ) const
{
   if( error_event_ )
      error_event_( code ) ;
   else 
      TListener::SocketError( code ) ;
}

//////////////////////////////////////////////////////////////////////////
// net_broadcaster

inline net_broadcaster::net_broadcaster( receive_event_t revent, net_error_t on_error ) 
   : TSingleDestDgram( TINETAddr( INADDR_ANY, 0 ), true )
   , receive_event_( revent )
   , error_event_(on_error)
{
}

inline net_broadcaster::net_broadcaster( TINETAddr const& addr, net_error_t on_error ) 
   : TSingleDestDgram( addr, true )
   , error_event_(on_error)
{
}


inline net_broadcaster::net_broadcaster( TINETAddr const& addr, receive_event_t revent, net_error_t on_error ) 
   : TSingleDestDgram( addr, true )
   , receive_event_  ( revent     )
   , error_event_    ( on_error   )
{
}

inline net_broadcaster::net_broadcaster( TINETAddr const& sendAddr, TINETAddr const& listenAddr, receive_event_t revent, net_error_t on_error )
   :  TSingleDestDgram  ( sendAddr, true )
   ,  receive_event_    ( revent )
   ,  error_event_      ( on_error )
{
   bind( listenAddr ) ;
}

inline void net_broadcaster::send( size_t size, void const * data ) 
{
   Send( size, data ) ;
}

inline int net_broadcaster::bind( TINETAddr const& addr ) 
{
   return Bind( addr ) ;
}

inline int net_broadcaster::set_reuse( bool reuse )
{
   const BOOL reuse_param = reuse;
   return SetOption( SOL_SOCKET, SO_REUSEADDR, &reuse_param, sizeof(reuse_param));
}

inline void net_broadcaster::OnReceive( size_t size, void const* data, TINETAddr const& addr ) 
{
   if( receive_event_ )
      receive_event_( size, data, addr ) ;
}

inline void net_broadcaster::SocketError( int code ) const
{
   if( error_event_ )
      error_event_( code ) ;
   else 
      TSingleDestDgram::SocketError( code ) ;
}


//////////////////////////////////////////////////////////////////////////
// net_pinger 

struct net_pinger::Impl : TPinger 
{
   Impl( net_event_t on_echo, net_event_t on_timeout, net_event_t on_ping_failed,
         net_error_t on_error, ulong subnet, ulong subnet_mask ) ;

private:
   void OnFinalMessage( HWND ) ;

private:
   void OnEchoReceived() ;
   void OnTimeOut     () ; 
   void OnPingFailed  () ;
   void SocketError   ( int ) const ;

private:
   net_event_t on_echo_         ;
   net_event_t on_timeout_      ;
   net_event_t on_ping_failed_  ;
   net_error_t on_error_        ;
};

inline net_pinger::net_pinger( LPCSTR host_name, net_event_t on_echo, net_event_t on_timeout, 
                              net_event_t ping_failed, net_error_t on_error, 
                              ulong subnet, ulong subnet_mask)
   : pImpl_( new net_pinger::Impl( on_echo, on_timeout, ping_failed, on_error, subnet, subnet_mask )) 
{
   pImpl_->Ping( host_name ) ;   
}

inline net_pinger::~net_pinger()
{
   pImpl_->StopPinging  () ;
   pImpl_->DestroyWindow() ;
}

// impl
inline net_pinger::Impl::Impl( net_event_t on_echo, net_event_t on_timeout, net_event_t on_ping_failed, net_error_t on_error, ulong subnet, ulong subnet_mask )
   : TPinger ( subnet, subnet_mask )
   , on_echo_( on_echo ), on_timeout_( on_timeout ), on_ping_failed_( on_ping_failed )
{
}

inline void net_pinger::Impl::OnFinalMessage( HWND )     { delete this ; }
inline void net_pinger::Impl::OnEchoReceived()           { if (on_echo_) on_echo_() ; }
inline void net_pinger::Impl::OnTimeOut     ()           { if (on_timeout_) on_timeout_() ; }
inline void net_pinger::Impl::OnPingFailed  ()           { if (on_ping_failed_) on_ping_failed_() ; }
inline void net_pinger::Impl::SocketError   ( int code ) const
{  
   if( on_error_ )
      on_error_( code ) ;
   else 
      TPinger::SocketError( code ) ;
}

