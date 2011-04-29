//////////////////////////////////////////////////////////////////////////
//
// transport.h
// provides async socket wrappers 
//
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
// DECLARATION 

#pragma once
#pragma comment( lib, "ws2_32.lib" )

#include <list>
#include <string>
#include <vector>
#include <utility>

#include "lock.h"

//////////////////////////////////////////////////////////////////////////
// socket library initialization
void InitWSA () ; 

typedef unsigned short ushort ;
typedef unsigned long  ulong  ; 

struct TINETAddr 
{
public:
   TINETAddr() ;
   TINETAddr( ulong ip, ushort port ) ;
   TINETAddr( unsigned char b1, unsigned char b2, unsigned char b3, unsigned char b4, ushort port ) ;
   TINETAddr( SOCKADDR const& address ) ;

   std::string GetText     () const ;
   SOCKADDR    GetSockAddr () const ;
   bool        Valid       () const ;

public:
   ulong  address ;
   ushort port    ; // in host order 
} ;

//////////////////////////////////////////////////////////////////////////
// TSocket 
class TSocket 
   : public CWindowImpl< TSocket >
{
public:
   int  SetOption( int level, int optname, const void * optval, int optlen ) ; 

   int  GetSocketName ( TINETAddr& name ) const ;
   int  GetPeerName   ( TINETAddr& name ) const ;

protected :
   TSocket () ;
   TSocket ( int type, int protocol, long asyncMode ) ;
   TSocket ( const SOCKET& sock, long asyncMode ) ;
  ~TSocket () ;

   void   Attach( SOCKET const& sock, long asyncMode ) ; 
   SOCKET Detach() ;

   void AsyncSelect( long event ) ;

   int  Bind      ( TINETAddr const& addr ) ;
   int  Connect   ( TINETAddr const& addr ) ;
   int  Shutdown  () ;
   int  Listen    () ;

protected:
   BEGIN_MSG_MAP( TSocket )
      MESSAGE_HANDLER( WM_TSOCKET_EVENT, OnSocketEvent )
   END_MSG_MAP  () 

protected:
   // size on input = sizeof ( buf ), on output = number of bytes ( read or written )
   // with destination 
   int  DoRecv( size_t* size,       void * buf, TINETAddr      & from ) ;
   int  DoSend( size_t* size, const void * buf, TINETAddr const& dest ) ;
   // without destination 
   int  DoSend( size_t* size, const void * buf ) ;

protected:
   virtual void SocketError( int code ) const ;

private :
   virtual void Process_FD_ACCEPT ( int error ) {}
   virtual void Process_FD_CONNECT( int error ) {}
   virtual void Process_FD_CLOSE  ( int error ) {}
   virtual void Process_FD_READ   ( int error ) {}
   virtual void Process_FD_WRITE  ( int error ) {}

private:
   LRESULT OnSocketEvent( UINT, WPARAM, LPARAM, BOOL& ) ;

protected :      
   SOCKET socket_ ;

private :
   static const UINT WM_TSOCKET_EVENT  = WM_USER + 1000 ;
} ;

class TSockPoint 
   : public TSocket
{
protected: 
   struct Packet
   {
      size_t  size ;
      char *  data ;

      Packet () ; 
      ~Packet() ;

      void Assign ( size_t size, void const* data ) ;
   };    

protected: 
   TSockPoint() ;
   TSockPoint( int type, int protocol, long asyncMode ) ;
   TSockPoint( SOCKET const& sock, long asyncMode ) ;

protected:
   virtual void ProcessReceive   ( size_t size, void const* data, TINETAddr const& ) {}
   virtual void ProcessReadySend ()                                                  {}

private:
   void Process_FD_READ ( int error ) ;
   void Process_FD_WRITE( int error ) ;
};


//////////////////////////////////////////////////////////////////////////
// TSockDgram - UDP socket 
class TSockDgram 
   : public TSockPoint
{
public:
   TSockDgram( bool broadcasting = false ) ;
   void Send ( size_t size, const void * data, TINETAddr const& dest ) ;

protected:
   void ProcessReceive   ( size_t size, const void * data, TINETAddr const& from ) ;
   void ProcessReadySend () ;

protected:
   virtual void OnReceive ( size_t size, const void * data, TINETAddr const& from ) {}

private :
   std::list< std::pair< TINETAddr, Packet > > outlist_ ;
   lock::critsec cslock_ ;
} ; 

//////////////////////////////////////////////////////////////////////////
// TSingleDestDgram - UDP socket for one destination point  

class TSingleDestDgram 
   : public TSockPoint
{
public:
   TSingleDestDgram( TINETAddr const& addr, bool broadcasting = true ) ;
   void Send( size_t size, const void * data ) ;

protected:
   void ProcessReceive   ( size_t size, const void * data, TINETAddr const& from ) ;
   void ProcessReadySend () ;

protected:
   virtual void OnReceive ( size_t size, const void * data, TINETAddr const& from ) {}

private :
   TINETAddr            defDest_ ; 
   std::list< Packet >  outlist_ ;
   lock::critsec     cslock_ ;
} ; 

//////////////////////////////////////////////////////////////////////////
// TSockStream - TCP socket 

class TSockStream 
   : public TSockPoint
{
public:
   TSockStream () ; 
   TSockStream ( const SOCKET& sock ) ;
  ~TSockStream () ;

   void Attach( SOCKET const& sock ) ; 
   void Send  ( size_t size, void const* data ) ;
   
   bool IsConnected() const ;

protected:
   void ProcessReceive   ( size_t size, const void * data, TINETAddr const& from ) ;
   void ProcessReadySend () ;
   void Process_FD_CLOSE ( int error ) ;
   
protected:
   virtual void OnReceive     ( size_t size, void const* data ) {}  
   virtual void OnDisconnected() {}

private: 
   std::list< Packet >  outlist_ ; // destination is always the same 
   size_t               offset_  ; 
   lock::critsec       cslock_  ;
   bool                 connected_ ;
} ;

//////////////////////////////////////////////////////////////////////////
// TFragmentStream 
// treats first 4 bytes of incoming message as fragment length

class TFragmentStream
   : public TSockStream
{
public :
   TFragmentStream() ; 
   TFragmentStream( SOCKET const& sock ) ;

protected :
   void ProcessReceive( size_t size, const void * data, TINETAddr const& from ) ;

private: 
   size_t  MsgSize() const ;

private: 
   std::vector< char >  buf_ ;
} ;

//////////////////////////////////////////////////////////////////////////
// TConnectingStream

template< class SockStream >
class TConnectingStream : public SockStream
{
public:
   TConnectingStream() ;
  ~TConnectingStream() ;

  void ConnectTo( TINETAddr const& ) ;
  void Reconnect() ;
  
  virtual void OnConnectFailed() ; 
  virtual void OnConnected    () = 0 ; 

private:
   class Impl   ;
   Impl* pImpl_ ; 
};

typedef TConnectingStream< TSockStream     > TSocketConnectingStream   ;
typedef TConnectingStream< TFragmentStream > TFragmentConnectingStream ;

//////////////////////////////////////////////////////////////////////////
// TConnector

class TConnector 
   : public TSocket
{
public:
   TConnector() ;
   ~TConnector() ;

   void ConnectTo  ( TINETAddr const& addr ) ;
   void Reconnect  () ;

public:
   BEGIN_MSG_MAP( TConnector )
      MESSAGE_HANDLER( WM_TIMER, OnTimer )
      CHAIN_MSG_MAP  ( TSocket )
   END_MSG_MAP() 

private:
   LRESULT OnTimer( UINT, WPARAM, LPARAM, BOOL& ) ;

protected :
   void Process_FD_CONNECT( int error ) ;

   virtual void OnConnectFailed () ;
   virtual void OnConnected     ( SOCKET sock ) = 0 ; 

private:
   TINETAddr lastAddr_  ; 
} ;

//////////////////////////////////////////////////////////////////////////
// TListener 

class TListener 
   : public TSocket
{
public :
   TListener() ;
   int ListenAs( TINETAddr const& addr ) ;

protected :
   void Process_FD_ACCEPT ( int error ) ;

protected:
   virtual void OnAccepted  ( SOCKET sock ) = 0 ;
} ;

//////////////////////////////////////////////////////////////////////////
// TPinger 

class TPinger 
   : public TSockPoint
{
public :
   TPinger ( DWORD subnet = 0, DWORD subnetmask = 0 ) ;
  ~TPinger () ; 

   void Ping       ( const char * hostname ) ; 
   void StopPinging() ;
   bool IsPinging  () const ; 

protected :
   void ProcessReceive( size_t size, void const* data, TINETAddr const& from ) ;

private:
   void StartPinging() ;
   void CheckPingTimeout() ;

private :
   virtual void OnEchoReceived () = 0 ; 
   virtual void OnTimeOut      () = 0 ; 
   virtual void OnPingFailed   () = 0 ;                 

private:
   BEGIN_MSG_MAP( TPinger )
      MESSAGE_HANDLER( WM_TIMER, OnTimer )
      CHAIN_MSG_MAP  ( TSockPoint )
   END_MSG_MAP()

   LRESULT OnTimer      ( UINT, WPARAM, LPARAM, BOOL& ) ;
   LRESULT OnSearchHost ( UINT, WPARAM, LPARAM, BOOL& ) ;

private:
   ushort Checksum ( ushort *buffer, size_t size ) ;

protected :
   TINETAddr   m_dest ; 
   std::string m_host ;

private:
   struct IcmpHeader * m_pdata ; 
   ushort              m_echoid ; 
   ushort              m_seqno ; 
   BOOL                m_bPinging ; 
   BOOL                m_bNoReply ;  
   DWORD               m_subnet ;
   DWORD               m_subnetmask   ;
} ; 

//////////////////////////////////////////////////////////////////////////
// TSyncPinger 

class TSyncPinger 
   : private TPinger
{
public:

   TSyncPinger ( LPCSTR host_name, DWORD subnet, DWORD subnetmask ) ;
   static DWORD WINAPI ThreadProc ( LPVOID lpParameter ) ;

   // TPinger 
protected : 
   void  OnEchoReceived() ;
   void  OnTimeOut     () ;
   void  OnPingFailed  () ;

   friend bool PingHost ( const char *, DWORD *, DWORD, DWORD, DWORD ) ; 

private:
   struct ping_params 
   {
      LPCSTR  hostname ; 
      DWORD   subnet ;  
      DWORD   subnetmask ;  
      DWORD * ipaddress ;
      HANDLE  hEvent ; 
   } ; 
} ; 

bool PingHost ( const char * hostname, DWORD * ipaddress, DWORD timeout, DWORD subnet = 0, DWORD subnetmask = 0 ) ;
bool ConnectTo( TINETAddr const& addr, SOCKET& sock ) ;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION 

//////////////////////////////////////////////////////////////////////////
// Packet. struct is not generaly safe. Only empty Packet instances can be copied
inline TSockPoint::Packet::Packet ()  
   : size( 0 ) 
   , data( NULL )
{
}

inline TSockPoint::Packet::~Packet() 
{
   delete [] data ; 
}

inline void TSockPoint::Packet::Assign ( size_t msg_size, void const* pdata ) 
{
   size = msg_size ;
   data = new char [ size ] ; 
   memcpy ( data, pdata, size ) ;
}

//////////////////////////////////////////////////////////////////////////
// TINETAddr 

inline TINETAddr::TINETAddr() 
   : address( INADDR_NONE )
   , port   ( 0 )
{
}

inline TINETAddr::TINETAddr( ulong addr, ushort prt )
{
   address = addr ;
   port    = prt ;
}

inline TINETAddr::TINETAddr( unsigned char b1,
                             unsigned char b2,
                             unsigned char b3,
                             unsigned char b4,
                             ushort prt ) 
{
   port    = prt ;
   address = b4 << 24 |
             b3 << 16 |
             b2 << 8  |
             b1;
}

inline TINETAddr::TINETAddr( SOCKADDR const& addr )
{
   sockaddr_in const& ip_addr = ( sockaddr_in const& ) addr ;  

   port    = ntohs( ip_addr.sin_port ) ;
   address = ip_addr.sin_addr.s_addr ;
}

inline std::string TINETAddr::GetText() const 
{
   union { struct { u_char b1, b2, b3, b4 ; } byte_address ; u_long long_address ; } ip_addr ; 
   ip_addr.long_address = address ; 

   char buf [ 0x40 ] ;
   sprintf ( buf, "%04x\\%d.%d.%d.%d", port, ip_addr.byte_address.b1, ip_addr.byte_address.b2, 
                                             ip_addr.byte_address.b3, ip_addr.byte_address.b4 ) ;
   return buf ;
}

inline SOCKADDR TINETAddr::GetSockAddr() const 
{
   sockaddr_in ip_addr ;

   ip_addr.sin_family      = AF_INET ;
   ip_addr.sin_port        = htons( port ) ;
   ip_addr.sin_addr.s_addr = address ;

   return ( SOCKADDR& ) ip_addr ;
}

inline bool TINETAddr::Valid() const 
{
   return address != INADDR_NONE ; 
}

//////////////////////////////////////////////////////////////////////////
// TSocket

inline int TSocket::SetOption( int level, int optname, const void * optval, int optlen ) 
{
   return setsockopt( socket_, level, optname, (const char *)optval, optlen ) ;
}

inline int TSocket::GetSocketName( TINETAddr& addr ) const
{
   SOCKADDR sock_address ;
   int addrLen = sizeof ( sock_address ) ;
   if ( getsockname ( socket_, &sock_address, &addrLen ) != SOCKET_ERROR )
   {
      addr = TINETAddr( sock_address ) ; 
      return 1 ;
   }

   SocketError ( WSAGetLastError()) ;
   return 0 ;
}

inline int TSocket::GetPeerName( TINETAddr& addr ) const
{
   SOCKADDR sock_addr ; 
   int addrLen = sizeof( sock_addr ) ;
   if ( getpeername ( socket_, &sock_addr, &addrLen ) != SOCKET_ERROR )
   {
      addr = TINETAddr( sock_addr ) ;
      return 1 ;
   }

   SocketError ( WSAGetLastError()) ;
   return 0 ;
}

inline TSocket::TSocket()
   : socket_( INVALID_SOCKET )
{
   InitWSA () ;
   Create ( NULL, NULL, _T( "TSocket Window" ), WS_POPUP ) ;
}

inline TSocket::TSocket( int type, int protocol, long asyncMode ) 
   : socket_( INVALID_SOCKET )
{
   InitWSA () ;
   socket_ = socket ( AF_INET, type, protocol ) ;
   Create ( NULL, NULL, _T( "TSocket Window" ), WS_POPUP ) ;
   AsyncSelect( asyncMode ) ;
}

inline TSocket::TSocket( const SOCKET& sock, long asyncMode ) 
   : socket_( sock  )
{
   InitWSA () ;
   Create ( NULL, NULL, _T( "TSocket Window" ), WS_POPUP ) ;
   AsyncSelect( asyncMode ) ;
}

inline TSocket::~TSocket()
{
   if ( socket_ != INVALID_SOCKET ) 
      closesocket ( socket_ ) ;
   
   if( IsWindow())
      DestroyWindow() ;
}

inline void TSocket::Attach( SOCKET const& sock, long asyncMode ) 
{
   if ( socket_ != INVALID_SOCKET ) 
      closesocket ( socket_ ) ;
   
   socket_ = sock ;
   AsyncSelect( asyncMode ) ;
}

inline SOCKET TSocket::Detach() 
{
   SOCKET sock = socket_ ;
   socket_ = INVALID_SOCKET ;

   return sock ;
}

inline void TSocket::AsyncSelect( long event )
{
   WSAAsyncSelect( socket_, m_hWnd, WM_TSOCKET_EVENT, event ) ;
}

inline int TSocket::Bind( TINETAddr const& addr )
{
   if ( bind( socket_, &addr.GetSockAddr(), sizeof( SOCKADDR )) != SOCKET_ERROR ) 
      return 1 ;                              

   SocketError ( WSAGetLastError()) ;
   return 0 ;
}

inline int TSocket::Connect( TINETAddr const& addr )
{
   if ( connect( socket_, &addr.GetSockAddr(), sizeof( SOCKADDR )) != SOCKET_ERROR ) 
      return 1 ;
   
   int ret = WSAGetLastError () ;
   if ( ret == WSAEWOULDBLOCK ) 
      return 1 ;
   
   SocketError( ret ) ;
   return 0 ;
}

inline int TSocket::Shutdown()
{
   return shutdown( socket_, SD_BOTH ) != SOCKET_ERROR ;
}

inline int TSocket::Listen()
{
   if ( listen( socket_, 50 ) != SOCKET_ERROR ) 
      return 1 ;

   SocketError ( WSAGetLastError()) ;
   return 0 ;
}

inline LRESULT TSocket::OnSocketEvent( UINT, WPARAM, LPARAM lParam, BOOL& ) 
{
   int err = WSAGETSELECTERROR ( lParam ) ;
   if ( err ) 
      SocketError ( err ) ;
   
   switch ( WSAGETSELECTEVENT ( lParam ))
   {
   case FD_READ   : Process_FD_READ   ( err ) ; break ;
   case FD_WRITE  : Process_FD_WRITE  ( err ) ; break ;
   case FD_ACCEPT : Process_FD_ACCEPT ( err ) ; break ;
   case FD_CONNECT: Process_FD_CONNECT( err ) ; break ;
   case FD_CLOSE  : Process_FD_CLOSE  ( err ) ; break ;
   }

   return 0 ;
}

inline int TSocket::DoRecv( size_t* size, void * buf, TINETAddr& from )
{
   SOCKADDR addr ;
   int addrLen = sizeof( SOCKADDR ) ;
   int ret = recvfrom( socket_, (char *)buf, *size, 0, &addr, &addrLen ) ;
   from = TINETAddr( addr ) ;

   if( ret == SOCKET_ERROR )
   {
      int err = WSAGetLastError ();
      if ( err != WSAEWOULDBLOCK )
         SocketError ( err ) ;    

      *size = 0 ;
      return 0 ;
   }

   *size = ret ;
   return ret != 0 ;
}

inline int TSocket::DoSend( size_t* size, const void * buf, TINETAddr const& dest )
{
   int ret = sendto( socket_, (char *)buf, *size, 0, &dest.GetSockAddr(), sizeof( SOCKADDR )) ;
   if( ret == SOCKET_ERROR )
   {    
      *size = 0 ;

      int err = WSAGetLastError () ;
      if ( err == WSAEWOULDBLOCK ) 
         return 1 ;

      SocketError ( err ) ;
      return 0 ;
   }

   *size = ret ; 
   return 1 ; 
}

inline int TSocket::DoSend( size_t* size, const void * buf )
{
   int ret = send( socket_, (char *)buf, *size, 0 ) ;
   if( ret == SOCKET_ERROR )
   {    
      *size = 0 ;

      int err = WSAGetLastError () ;
      if ( err == WSAEWOULDBLOCK ) 
         return 1 ;

      SocketError ( err ) ;
      return 0 ;
   }

   *size = ret ; 
   return 1 ; 
}

inline void TSocket::SocketError( int code ) const 
{
   TCHAR buf[256];
   _stprintf (buf, _T("Socket error on socket: %d; code: %d\n"), socket_, code ) ;
   OutputDebugString(buf);
}

//////////////////////////////////////////////////////////////////////////
// TSockPoint 
inline TSockPoint::TSockPoint()
{
}

inline TSockPoint::TSockPoint( int type, int protocol, long asyncMode ) 
   : TSocket( type, protocol, asyncMode ) 
{
}

inline TSockPoint::TSockPoint( SOCKET const& sock, long asyncMode ) 
   : TSocket( sock, asyncMode )
{
}            

inline void TSockPoint::Process_FD_READ( int error ) 
{
   if ( !error )
   {
      TINETAddr from ;

      char buf  [ 1500 ] ;  // approx. max length of Ethernet packet 
      size_t  size = sizeof( buf ) ;

      // receive as many messages as the input buffer has
      while ( DoRecv( &size, buf, from ))
      {    
         ProcessReceive( size, buf, from ) ;
         size = sizeof ( buf ) ;
      }
   }
}

inline void TSockPoint::Process_FD_WRITE( int error )
{
   if ( !error )
      ProcessReadySend() ;
}

//////////////////////////////////////////////////////////////////////////
// TSockDgram

inline TSockDgram::TSockDgram( bool broadcasting )   
   : TSockPoint( SOCK_DGRAM, IPPROTO_UDP, FD_WRITE | FD_READ )
{
   // on the sender site there will be no warning in case of small receive socket buffer!
   size_t size = 64*1024;
   SetOption( SOL_SOCKET, SO_RCVBUF, &size, sizeof( int ));
   SetOption( SOL_SOCKET, SO_BROADCAST, &broadcasting, sizeof ( broadcasting )) ;
}

inline void TSockDgram::Send( size_t size, const void * data, TINETAddr const& dest ) 
{
   lock::scoped_lock lock( cslock_ ) ;

   if ( outlist_.empty())
   {
      size_t sz = size ;
      if ( DoSend( &sz, data, dest ) == 0 || sz == size ) 
         return ;
   }
   outlist_.push_back( std::make_pair( dest, Packet())) ;
   outlist_.back().second.Assign( size, data ) ; 
}

inline void TSockDgram::ProcessReadySend()
{
   lock::scoped_lock lock( cslock_ ) ;

   while ( !outlist_.empty())
   {
      TINETAddr const&  addr = outlist_.front().first  ;
      Packet const&     item = outlist_.front().second ;
      size_t            sz   = item.size ;

      if ( DoSend( &sz, item.data, addr ) == 0 || sz == 0 )
         return ; 

      // Assert( sz == item.size ) ; // UDP!
      outlist_.pop_front () ;
   }
}

inline void TSockDgram::ProcessReceive ( size_t size, const void * data, TINETAddr const& from ) 
{
   OnReceive ( size, data, from ) ; 
}

//////////////////////////////////////////////////////////////////////////
// TOneDestDgram

inline TSingleDestDgram::TSingleDestDgram( TINETAddr const& dest, bool broadcasting )   
   : TSockPoint( SOCK_DGRAM, IPPROTO_UDP, FD_WRITE | FD_READ )               
   , defDest_  ( dest )
{
   // on the sender site there will be no warning in case of small receive socket buffer!
   size_t size = 64*1024;
   SetOption( SOL_SOCKET, SO_RCVBUF, &size, sizeof( int ));
   SetOption( SOL_SOCKET, SO_BROADCAST, &broadcasting, sizeof ( broadcasting )) ;
}

inline void TSingleDestDgram::Send( size_t size, const void * data ) 
{
   lock::scoped_lock lock( cslock_ ) ;

   if ( outlist_.empty())
   {
      size_t sz = size ;
      if ( DoSend( &sz, data, defDest_ ) == 0 || sz == size ) 
         return ;
   }
   outlist_.push_back( Packet()) ;
   outlist_.back().Assign( size, data ) ; 
}

inline void TSingleDestDgram::ProcessReadySend()
{
   lock::scoped_lock lock( cslock_ ) ;

   while ( !outlist_.empty())
   {
      Packet const& item = outlist_.front() ;
      size_t        sz   = item.size ;

      if ( DoSend( &sz, item.data, defDest_ ) == 0 || sz == 0 )
         return ; 

      // Assert( sz == item.size ) ; // UDP!
      outlist_.pop_front () ;
   }
}

inline void TSingleDestDgram::ProcessReceive ( size_t size, const void * data, TINETAddr const& from ) 
{
   OnReceive ( size, data, from ) ; 
}

//////////////////////////////////////////////////////////////////////////
// TConnectingStream 

template< class SocketStream >
   class TConnectingStream< SocketStream >::Impl : public TConnector
{
public:
   Impl( TConnectingStream& ) ;

protected:
   void OnConnected     ( SOCKET sock ) ;
   void OnConnectFailed () ;

protected: 
   void OnFinalMessage ( HWND ) ; 

private:
   TConnectingStream& stream_ ;
};

template< class SocketStream >
   TConnectingStream< SocketStream >::TConnectingStream()
      : pImpl_( new Impl( *this )) 
{

};

template< class SocketStream >
TConnectingStream< SocketStream >::~TConnectingStream()
{
   pImpl_->DestroyWindow() ;
};


template< class SocketStream >
   void TConnectingStream< SocketStream >::ConnectTo( TINETAddr const& addr ) 
{
   pImpl_->ConnectTo( addr ) ;
}

template< class SocketStream >
   void TConnectingStream< SocketStream >::Reconnect() 
{
   pImpl_->Reconnect() ;
}

template< class SocketStream >
   void TConnectingStream< SocketStream >::OnConnectFailed() 
{
   pImpl_->Reconnect() ;
}

// impl implementation 
template< class SocketStream >
TConnectingStream< SocketStream >::Impl::Impl( TConnectingStream& stream ) 
   : stream_( stream )    
{
}

template< class SocketStream >
   void TConnectingStream< SocketStream >::Impl::OnConnected( SOCKET sock ) 
{
   stream_.Attach( sock ) ;
   stream_.OnConnected() ;
}

template< class SocketStream >
   void TConnectingStream< SocketStream >::Impl::OnConnectFailed () 
{
   stream_.OnConnectFailed() ;
}

template< class SocketStream >
void TConnectingStream< SocketStream >::Impl::OnFinalMessage( HWND ) 
{
   delete this ;
}

//////////////////////////////////////////////////////////////////////////
// TConnector 
inline TConnector::TConnector () 
   : TSocket ( SOCK_STREAM, IPPROTO_TCP, FD_CONNECT | FD_CLOSE )
{
}

inline TConnector::~TConnector()
{
   if( IsWindow()) 
      KillTimer( 1 ) ;
   
   Shutdown () ;
}

inline void TConnector::ConnectTo( TINETAddr const& addr ) 
{
   lastAddr_ = addr ;
   SetTimer( 1, 0 ) ; // avoid sync callback
}

inline void TConnector::Reconnect()
{
   // Assert(lastAddr_.Valid());
   if ( socket_ == INVALID_SOCKET)
      Attach ( socket(AF_INET, SOCK_STREAM, IPPROTO_TCP), FD_CONNECT | FD_CLOSE ) ; 

   SetTimer( 1, 1000 ) ;
}

inline void TConnector::Process_FD_CONNECT( int error )
{
   if ( !error )
   {
      OnConnected ( Detach() ) ; 
   }
   else 
   {
      OnConnectFailed() ;
   }
}

inline void TConnector::OnConnectFailed()
{
   Reconnect ();
}

inline LRESULT TConnector::OnTimer( UINT, WPARAM, LPARAM, BOOL& )
{
   KillTimer( 1 ) ;
   if ( !Connect(lastAddr_) )
   {
      OnConnectFailed();
   }
   return 0 ;
}


//////////////////////////////////////////////////////////////////////////
// TSockStream 
inline TSockStream::TSockStream() 
   : offset_   ( 0     )
   , connected_( false )
{
}

inline TSockStream::TSockStream ( const SOCKET& sock ) 
   : TSockPoint( sock, FD_WRITE | FD_READ | FD_CLOSE ) 
   , offset_   ( 0    )
   , connected_( true )
{
}

inline TSockStream::~TSockStream()
{
   Shutdown() ;
}

inline void TSockStream::Attach( SOCKET const& sock )
{
   TSocket::Attach( sock, FD_WRITE | FD_READ | FD_CLOSE ) ;
   connected_ = true ;
}

inline void TSockStream::Send( size_t size, void const* data ) 
{
   lock::scoped_lock lock( cslock_ ) ;

   if ( outlist_.empty())
   {
      size_t sz = size ;
      if ( DoSend( &sz, data ) == 0 || sz == size ) return ;
      data = (char*)data + sz ;
      size -= sz ;
   }
   outlist_.push_back( Packet()) ;   
   outlist_.back().Assign( size, data ) ; 
}

inline bool TSockStream::IsConnected() const 
{
   return connected_ ;
}

inline void TSockStream::ProcessReadySend() 
{
   lock::scoped_lock lock( cslock_ ) ;

   while ( !outlist_.empty())
   {
      Packet const& item = outlist_.front() ;
      size_t        size = item.size - offset_ ;

      if ( DoSend( &size, item.data + offset_ ))
      {
         offset_ += size ;
         if ( offset_ != item.size ) // have some more to send, but send was blocked
            return ; 
      }
      else 
         return ;

      offset_ = 0 ;
      outlist_.pop_front () ;
   }
}

inline void TSockStream::Process_FD_CLOSE( int error )
{
   connected_ = false ;
   OnDisconnected() ;   
}

inline void TSockStream::ProcessReceive( size_t size, const void * incoming_data, TINETAddr const& )
{
   OnReceive( size, incoming_data ) ;
}

//////////////////////////////////////////////////////////////////////////
// TFragmentStream

inline TFragmentStream::TFragmentStream () 
{
   buf_.reserve( 0x400 ) ;
}

inline TFragmentStream::TFragmentStream( const SOCKET& sock ) 
   : TSockStream( sock )
{
   buf_.reserve( 0x400 ) ;
}

inline void TFragmentStream::ProcessReceive( size_t size, const void * incoming_data, TINETAddr const& )
{
   size_t      used = 0 ;
   byte const* data = static_cast< byte const* >( incoming_data ) ;

   while( used < size )
   {
      if( buf_.size() < sizeof( size_t )) // reading the header 
      {
         buf_.push_back( data[ used ++ ] ) ;
      }
      else 
      {
         size_t count = std::min< size_t >( MsgSize() - buf_.size(), size_t( size - used )) ;
         buf_.insert( buf_.end(), data + used, data + used + count ) ;
         used += count ;
      }

      if( buf_.size() >= sizeof( size_t ) && MsgSize() == buf_.size()) // finally, we have the whole message 
      {
         OnReceive( buf_.size(), &buf_[0] ) ;
         buf_.clear() ;
      }
   }     
}

inline size_t TFragmentStream::MsgSize() const 
{
   // Assert( buf_.size() >= sizeof( size_t )) ;
   return *( size_t const* )&buf_[ 0 ] ; 
}

//////////////////////////////////////////////////////////////////////////
// TListener 

// ------------------------------------------------------ class TListener
inline TListener::TListener() 
   : TSocket ( SOCK_STREAM, IPPROTO_TCP, FD_ACCEPT )
{
}

inline int TListener::ListenAs( TINETAddr const& addr )
{
   return Bind( addr ) && Listen() ;
}

inline void TListener::Process_FD_ACCEPT( int error )
{
   if ( !error )
   {
      SOCKET s = accept ( socket_, NULL, NULL ) ;
      if ( s != INVALID_SOCKET ) 
         OnAccepted( s ) ;
   }
}

//////////////////////////////////////////////////////////////////////////
// TPinger 

///////////////////////////////////////////////////////////////////////////
// ping messages 

#pragma pack(push, 4)
struct IpHeader 
{
   unsigned int h_len:4;          // length of the header
   unsigned int version:4;        // Version of IP
   unsigned char tos;             // Type of service
   unsigned short total_len;      // total length of the packet
   unsigned short ident;          // unique identifier
   unsigned short frag_and_flags; // flags
   unsigned char  ttl; 
   unsigned char proto;           // protocol (TCP, UDP etc)
   unsigned short checksum;       // IP checksum

   unsigned int sourceIP;
   unsigned int destIP;
};

struct IcmpHeader {
   BYTE i_type;
   BYTE i_code; /* type sub code */
   USHORT i_cksum;
   USHORT i_id;
   USHORT i_seq;
   /* This is not the std header, but we reserve space for time */
   ULONG timestamp;
};

#pragma pack ( pop )

enum 
{
   ICMP_ECHO      = 8,
   ICMP_ECHOREPLY = 0,
   ICMP_MIN       = 8, 
   ICMPSIZE       = sizeof( IcmpHeader ) + 32 
} ;

//////////////////////////////////////////////////////////////////////////
// timer 1 используется при создании для того, чтобы убрать синхронность вызова ping и возможного OnPingFailed 
// timer 2 используется для проверки, что прошла секунда, а ответного сообщения все еще нет

inline TPinger::TPinger ( DWORD subnet, DWORD subnet_mask ) 
   : TSockPoint( SOCK_RAW, IPPROTO_ICMP, FD_WRITE | FD_READ )
   , m_bPinging   ( FALSE )
   , m_bNoReply   ( FALSE )
   , m_subnet     ( subnet)
   , m_subnetmask ( subnet_mask )
{
   m_echoid = (ushort)GetCurrentProcessId() ; 

   int timeout = 1000 ; 
   setsockopt ( socket_, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout) );

   timeout = 1000 ;
   setsockopt ( socket_, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout) );

   int b = 1 ;
   setsockopt ( socket_, SOL_SOCKET, SO_BROADCAST, (char *) &b, sizeof ( b ) ) ;
   
   m_pdata = (IcmpHeader *) new char [ICMPSIZE];

   memset ( m_pdata, 'E', ICMPSIZE ) ; 

   m_pdata -> i_type  = ICMP_ECHO;
   m_pdata -> i_code  = 0;
   m_pdata -> i_id    = m_echoid ;
}

inline TPinger::~TPinger() 
{
   StopPinging() ;
   delete m_pdata ; 
}  

inline void TPinger::Ping ( const char * hostname ) 
{
   if ( !m_bPinging ) 
   {
      SetTimer( 1, 1 ) ;

      m_bPinging = TRUE ; 
      m_host = hostname ; 
   }
}

inline void TPinger::StopPinging()
{
   if( m_bPinging )
   {
      KillTimer( 2 ) ; 
      m_bPinging = FALSE ; 
   }
}

inline bool TPinger::IsPinging() const 
{
   return m_bPinging == TRUE ;
}
   
inline void TPinger::ProcessReceive( size_t size, void const* data, TINETAddr const& from ) 
{
   if ( m_bPinging )
   {
      char const * buf = ( char const* ) data ; 

      IpHeader   * iphdr    = (IpHeader *) buf ;
      size_t       iphdrlen = iphdr -> h_len * 4 ; 

      IcmpHeader * icmphdr  = (IcmpHeader*)(buf + iphdrlen) ;

      if ( size >= iphdrlen + ICMP_MIN       && 
         icmphdr -> i_type == ICMP_ECHOREPLY && 
         icmphdr -> i_id == m_echoid         &&
         m_dest.address == from.address )
      {
         m_bNoReply  = FALSE ; 
         OnEchoReceived() ; 
      }
   }
}

inline LRESULT TPinger::OnTimer( UINT, WPARAM id, LPARAM, BOOL& ) 
{
   switch( id )
   {
      case 1 : 
         StartPinging(); 
         break ;
      case 2 : 
         CheckPingTimeout();
         break ;
      //default:
         //Assert( 0 ) ;
   }
   
   return 0 ;
}

inline void TPinger::StartPinging() 
{
   KillTimer( 1 ) ;

   if( !m_dest.Valid())
   {
      hostent * hp = gethostbyname( m_host.c_str()) ;
      if ( !hp )
      {
         OnPingFailed() ;
         m_bPinging = FALSE ;
         return ;
      }

      m_dest.address = *((DWORD*) hp->h_addr) ;

      if ( m_subnetmask != 0 ) 
      {
         for( size_t i = 0; hp->h_addr_list[i] != NULL ; ++i ) 
         {
            DWORD addr = *((DWORD*)hp->h_addr_list[i] ) ;
            if ( (addr & m_subnetmask) == m_subnet )
            {
               m_dest.address = addr ;
               break ;
            }
         }
      }

      if ( socket_ == INVALID_SOCKET ) 
      {
         OnPingFailed () ; 
         m_bPinging = FALSE ; 
         return ;
      }

      m_bNoReply = FALSE ; 
      m_seqno    = 0 ; 
      SetTimer( 2, 1000 ) ; 

      CheckPingTimeout() ;
   }
}

inline void TPinger::CheckPingTimeout()
{
   // если окажется, что в StopPinging мы останавливаем timer, а в очереди сообщений есть еще 
   // сообщение таймера, то мы рискуем вызвать OnTimeOut, поэтому добавляем в условие m_bPinging == TRUE 
   if ( m_bNoReply && m_bPinging ) 
      OnTimeOut() ; 

   m_pdata -> i_cksum   = 0 ;
   m_pdata -> timestamp = GetTickCount() ;
   m_pdata -> i_seq     = m_seqno++ ;

   m_pdata -> i_cksum = Checksum( (ushort*)m_pdata, ICMPSIZE ) ;

   if ( sendto( socket_, (char *)m_pdata, ICMPSIZE, 0, &m_dest.GetSockAddr(), sizeof(SOCKADDR)) == SOCKET_ERROR ) 
   {
      KillTimer( 2 ) ; 
      OnPingFailed() ; 
      m_bPinging = FALSE ; 
   }
   m_bNoReply = TRUE ; 
}

inline ushort TPinger::Checksum( ushort *buffer, size_t size ) 
{
   unsigned long cksum=0;

   while ( size > 1 ) 
   {
      cksum += *buffer++;
      size  -= sizeof(ushort);
   }

   if( size )
      cksum += *(UCHAR*)buffer;

   cksum = (cksum >> 16) + (cksum & 0xffff);
   cksum += (cksum >>16);

   return static_cast<ushort>((~cksum) & 0xFFFF);
}



//////////////////////////////////////////////////////////////////////////
// TSyncPinger 

inline TSyncPinger::TSyncPinger ( LPCSTR host_name, DWORD subnet, DWORD subnetmask )  
   : TPinger( subnet, subnetmask )
{
   Ping( host_name ) ; 
}

inline DWORD WINAPI TSyncPinger::ThreadProc( LPVOID lpParameter )
{
   ping_params& params = *(ping_params *)lpParameter ; 

   MSG msg ; 
   PeekMessage ( &msg, NULL, 0, 0, PM_NOREMOVE ) ; 
   SetEvent ( params.hEvent ) ; 

   TSyncPinger pinger ( params.hostname, params.subnet, params.subnetmask ) ; 

   while( GetMessage( &msg, NULL, 0, 0 ))
      DispatchMessage( &msg ) ; 

   if( params.ipaddress != NULL )
      *(params.ipaddress) = pinger.m_dest.address ; 

   return msg.wParam ; 
}


inline void TSyncPinger::OnEchoReceived()   
{
   PostQuitMessage ( 2 ) ; 
}

inline void TSyncPinger::OnTimeOut ()
{
}

inline void TSyncPinger::OnPingFailed () 
{
   PostQuitMessage ( 1 ) ; 
}

//////////////////////////////////////////////////////////////////////////
// PingHost 

inline bool PingHost( const char * hostname, DWORD * ipaddress, DWORD timeout, DWORD subnet, DWORD subnetmask ) 
{
   TSyncPinger::ping_params params = { hostname, subnet, subnetmask, ipaddress, CreateEvent ( NULL, FALSE, FALSE, NULL ) } ; 

   DWORD  dwThreadId ; 
   HANDLE hThread = CreateThread ( NULL, 0, TSyncPinger::ThreadProc, &params, 0, &dwThreadId ) ; 

   WaitForSingleObject ( params.hEvent, INFINITE ) ; 
   CloseHandle ( params.hEvent ) ; 

   if ( WaitForSingleObject ( hThread, timeout ) == WAIT_TIMEOUT ) 
   {
      PostThreadMessage ( dwThreadId, WM_QUIT, 0, 0 ) ; 
      WaitForSingleObject ( hThread, INFINITE ) ; 
   }

   DWORD code ; 
   GetExitCodeThread ( hThread, &code ) ; 
   CloseHandle ( hThread ) ; 

   return code != 0 ; 
}

inline bool ConnectTo( TINETAddr const& addr, SOCKET& sock )
{
   InitWSA () ;
   SOCKET s = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP ) ;

   if( s == INVALID_SOCKET )
      return false ;

   if( connect( s, &addr.GetSockAddr(), sizeof( SOCKADDR )) == SOCKET_ERROR ) 
   {
      closesocket( s ) ;
      return false ;
   }

   sock = s ;
   return true ;
}

//
struct WSA_RAII_Wrapper
{
   WSA_RAII_Wrapper()
   {
      WORD wVersionRequested ;
      WSADATA wsaData ;
      wVersionRequested = MAKEWORD( 2, 0 ) ;
      WSAStartup( wVersionRequested, &wsaData ) ;
   }

   ~WSA_RAII_Wrapper()
   {
      WSACleanup() ;
   }
};

inline void InitWSA ()
{
   static WSA_RAII_Wrapper wrapper ; 
}

