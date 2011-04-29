#ifndef __INTERCEPT_H
#define __INTERCEPT_H

class CInterceptor
{
   unsigned char   origin_code[5]; 
   unsigned char   goto_code  [5]; 
   void          * oldAddress ; 
   BOOL            intercepted ; 

public : 
   CInterceptor () : intercepted ( FALSE ) {}
  ~CInterceptor()
   {
       Restore () ; 
   }

   BOOL Intercept ( LPCTSTR module, LPCSTR proc, const void * newAddress ) 
   {
       if ( ! intercepted ) 
       {
            HINSTANCE hLib = GetModuleHandle ( module ) ;
            if ( hLib ) 
            {
                 oldAddress = GetProcAddress ( hLib, proc ) ;
                 if ( oldAddress ) 
                 {
                      memcpy ( origin_code, oldAddress, 5 ) ; 
                      goto_code[0] = 0xE9 ; 
                      *(long *)(goto_code + 1) = ((long)newAddress - ((long)oldAddress + 5) ); 

                      DWORD dwProtect ;
                      if ( VirtualProtect ( oldAddress, 5, PAGE_EXECUTE_READWRITE, &dwProtect ) )
                      {
                           memcpy ( oldAddress, goto_code, 5 ) ; 
                           VirtualProtect ( oldAddress, sizeof( DWORD ), dwProtect, NULL ) ; 

                           intercepted = TRUE ; 
                           return TRUE ; 
                      }
                 }
            }
       }
       return FALSE ; 
   }

   BOOL Restore () 
   {
       if ( intercepted ) 
       {
            DWORD dwProtect ;
            if ( VirtualProtect ( oldAddress, sizeof( DWORD ), PAGE_EXECUTE_READWRITE, &dwProtect ) )
            {
                 memcpy ( oldAddress, origin_code, 5 ) ; 
                 VirtualProtect ( oldAddress, sizeof( DWORD ), dwProtect, NULL ) ; 

                 intercepted = FALSE ; 
                 return TRUE ; 
            }
       }
       return FALSE ; 
   }
} ; 

#endif