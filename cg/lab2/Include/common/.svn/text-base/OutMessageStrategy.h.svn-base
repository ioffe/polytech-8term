#pragma once

#include "trnkernel\trncontrol.h"

template< class SystemPtr >
struct OutMessageStrategy
{
   typedef 
      typename SystemPtr::Interface 
      System ;
      
   void Init( System * system, long object_id, IUnknown * sender = NULL )
   {
      Assert( system );
      system_ = system;
      
      message_.CreateInstance( __uuidof(OutControlMessage) ); 
      
      message_ -> SetObjectId ( object_id ) ;

      if( sender )
         message_ -> SetSender( sender );
   }

   void Init( IUnknown * environment, IUnknown * object )
   {
      Init ( SystemPtr(environment), IObjectInfoPtr(object)->ObjectId(), object ) ; 
   }

   void Deinit( )
   {
      system_  = NULL;
      message_ = NULL;      
   }

   void SendMessage() const
   {
      system_  -> SendControlMessage( message_ );
      message_ -> Clear( );      
   }

   IOutControlMessage * OutMessage( ) const
   {
      return message_;
   }

protected: 
   IOutControlMessagePtr message_;
   SystemPtr             system_;
};

