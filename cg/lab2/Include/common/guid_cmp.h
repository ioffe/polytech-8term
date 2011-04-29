#pragma once

#include <ostream>

inline bool operator < ( GUID const & a, GUID const & b )
{
   if ( a.Data1 < b.Data1 )
      return true;

   if ( a.Data1 > b.Data1 )
      return false;

   if ( a.Data2 < b.Data2 )
      return true;

   if ( a.Data2 > b.Data2 )
      return false;

   if ( a.Data3 < b.Data3 )
      return true;

   if ( a.Data3 > b.Data3 )
      return false;

   return memcmp( a.Data4, b.Data4, 8 ) < 0;
}

inline std::wstring GUIDToString( GUID const & guid )
{
   OLECHAR * buf;
   StringFromIID( guid, &buf );
   std::wstring res( buf );
   CoTaskMemFree( buf );
   return res;
}

template < class Stream >
   Stream & operator <<( Stream & out, GUID const & guid )
{
   out << GUIDToString( guid ).c_str();
   return out;
}

