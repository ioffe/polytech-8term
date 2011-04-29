#pragma once
#include "safe_bool.h"


template<class Interface> 
class ConnectionEnum
{
public : 
   ConnectionEnum ( IUnknown * cont, REFIID iid = __uuidof(Interface) ) 
   {
      IConnectionPointContainerPtr container ( cont ) ; 
      if ( container ) 
      {
         IConnectionPointPtr cpoint ; 
         if ( container -> FindConnectionPoint ( iid, &cpoint ) == S_OK )
             cpoint -> EnumConnections ( &enum_ ) ; 
      }
   }

   HRESULT Next ( Interface ** connection ) 
   {
      if ( enum_ ) 
      {
         CONNECTDATA cdata ; 
         if ( enum_ -> Next ( 1, &cdata, NULL ) == S_OK ) 
         {
            *connection = (Interface *)cdata.pUnk ; 
            return S_OK ; 
         }

         enum_ = NULL ; 
      }
      return S_FALSE ;
   }

private : 

   IEnumConnectionsPtr enum_ ; 
} ; 


template<class Interface> 
class ConnectionIter
{
public : 
   ConnectionIter ( IUnknown * cont, REFIID iid = __uuidof(Interface) ) 
      : enum_ ( cont, iid ) 
      , event_ ( NULL )  
   {
      enum_.Next(&event_) ; 
   }
   ~ConnectionIter ()
   {
      release_event() ; 
   }

   SAFE_BOOL_OPERATOR(event_ != NULL)

   Interface * operator -> () const 
   {
      return event_ ; 
   }

   ConnectionIter& operator ++ () 
   {
      release_event() ; 
      enum_.Next(&event_) ; 
      return *this ; 
   }
private :
   ConnectionEnum<Interface> enum_ ; 
   Interface *               event_ ;      // can't use ptr, because of nondefault iid

   void release_event()
   {
      if ( event_ ) 
      {
         event_ -> Release () ; 
         event_ = NULL ; 
      }
   }
} ; 

