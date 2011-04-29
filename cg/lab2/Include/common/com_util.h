#pragma once

#include <stdexcept>
#include <comutil.h>

#include "common/make_str.h"

namespace util
{
   inline _bstr_t StringFromCLSID(CLSID const & clsid)
   {
      LPOLESTR str; 
      StringFromCLSID ( clsid, &str) ; 
      _bstr_t res( str );
      CoTaskMemFree( str );
      return res;
   }

   inline CLSID CLSIDFromProgId(_bstr_t const & progId)
   {
      CLSID clsid;
      CLSIDFromProgID(progId, &clsid);
      return clsid;
   }

   inline _bstr_t ProgIDFromCLSID(CLSID const & clsid)
   {
      LPOLESTR progId;
      DWORD res = ProgIDFromCLSID(clsid, &progId);

      if (res == S_OK)
      {
         _bstr_t res = progId;
         CoTaskMemFree(progId);
         return res;
      }
      else if (res == REGDB_E_CLASSNOTREG)
         throw std::runtime_error( make_str() << "A class with CLSID = '" << StringFromCLSID(clsid) << "' hasn't been registred" );
      else if (res == REGDB_E_READREGDB)
         throw std::runtime_error( make_str() << "Error reading registry at getProgIdByCLSID" );

      return "";
   }
}