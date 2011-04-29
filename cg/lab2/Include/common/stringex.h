#pragma once

#include <comdef.h>

inline BOOL LoadStringIndirect ( HINSTANCE hInstance, HRSRC hResource, UINT nResID, _bstr_t & s )
{
  if ( hResource )
  {
    LPWSTR buffer = ( LPWSTR ) LoadResource ( hInstance, hResource );

    if ( NULL != buffer )
    {
      LPWSTR ptr = buffer ;

      for ( UINT i=0; i!=16; i++ )
      {
        if ( *ptr )
        {
          size_t len = *ptr++ ;

          if ( nResID == i )
          {
            wchar_t * tmp = new wchar_t[ len + 1 ];

            wcsncpy ( tmp, ptr, len );

            tmp[len] = L'\0';

            s = tmp;

            delete [] tmp;

            return TRUE ;
          }

          ptr += len ;
        }
        else
            ptr++;
      }
    }
  }

  return FALSE ;
}

inline _bstr_t LoadStringEx ( HINSTANCE hInstance, UINT nID )
{
  const UINT nResUnit = ( nID>>4 )+1;
  const int  nResID   = ( nID%16 );

  LCID theLCID = GetThreadLocale () ;

  WORD LangID = PRIMARYLANGID( LANGIDFROMLCID( theLCID ));
    
  HRSRC hResource = FindResourceEx ( hInstance, RT_STRING, MAKEINTRESOURCE( nResUnit ), MAKELANGID( LangID, SUBLANG_NEUTRAL ));

  _bstr_t s ;

  if ( ! LoadStringIndirect ( hInstance, hResource, nResID, s ))
  {
    hResource = FindResource ( hInstance, MAKEINTRESOURCE( nResUnit ), RT_STRING );

    if ( ! LoadStringIndirect ( hInstance, hResource, nResID, s ) )
      s = _T("");
  }

  return s;
}

