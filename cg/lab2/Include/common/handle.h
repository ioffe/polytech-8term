#pragma once

#include <common/safe_bool.h>

template < class Tag >
   struct handle
{
   explicit handle( const int id = -1 ) : id( id ) {}
   int id;

   SAFE_BOOL_OPERATOR( id != -1 )

   friend bool operator < ( handle lhs, handle rhs ) { return lhs.id < rhs.id; }
   friend bool operator ==( handle lhs, handle rhs ) { return lhs.id == rhs.id; }
   friend bool operator !=( handle lhs, handle rhs ) { return !( lhs == rhs ); }
};