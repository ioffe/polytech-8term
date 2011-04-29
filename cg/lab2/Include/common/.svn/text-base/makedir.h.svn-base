#ifndef __MAKEDIR_H__
#define __MAKEDIR_H__

inline BOOL MakeDirectory ( LPCOLESTR path ) 
{
  WCHAR * buf = (WCHAR*)_alloca(2048);

  _wfullpath ( buf, path, 1024 );

  size_t len = wcslen(buf);
  
  if ( buf[len-1] != L'\\' ) wcscat ( buf, L"\\" );

  WCHAR * p = buf;
  
  int count = wcsstr ( buf, L"\\\\" ) ? 4 : 2;
  
  for ( int i = 0; i < count; i++ )
  {
    p = wcschr ( p+1, L'\\' );
  }

  while ( p )               
  {
    *p = 0;

    if ( !CreateDirectoryW ( buf, NULL ) && 
          GetLastError () != ERROR_ALREADY_EXISTS ) 
      return FALSE;

    *p = L'\\';

    p = wcschr ( p+1, L'\\' );
  }

  return TRUE;
}

#endif
