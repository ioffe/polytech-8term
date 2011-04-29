#pragma once 


struct thread_context 
{
   thread_context ( LPCSTR name ) ;
  ~thread_context ();

   LPVOID get     () const ; 
   void   set     ( LPVOID context ) ; 

private : 
   #pragma pack(push,1) 
   struct mapped 
   {
       DWORD tls_index ; 
       DWORD usage ; 
   } ; 
   #pragma pack(pop) 

   HANDLE mutex_ ; 
   HANDLE mapping_ ; 
   DWORD  tls_index_ ; 
} ; 

//////////////////////////////////////////////////////////////////////////////////
inline thread_context :: thread_context ( LPCSTR name )
   : mutex_    ( NULL ) 
   , mapping_  ( NULL ) 
   , tls_index_ ( 0    ) 
{
   char buf [MAX_PATH] ;

   sprintf ( buf, "ThreadContextMutex_%s_%d", name, GetCurrentProcessId () ) ; 
   mutex_ = CreateMutexA ( NULL, FALSE, buf ) ; 

   WaitForSingleObject ( mutex_, INFINITE ) ; 

   sprintf ( buf, "ThreadContextMapping_%s_%d", name, GetCurrentProcessId ()) ; 
   mapping_ = CreateFileMappingA ( INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE | SEC_COMMIT, 0, 4096, buf ) ;

   BOOL exist = GetLastError () == ERROR_ALREADY_EXISTS ;

   mapped * pMapping = (mapped *)MapViewOfFile ( mapping_, FILE_MAP_WRITE, 0, 0, 4096 ) ; 

   if ( ! exist )
   {
      pMapping -> tls_index = TlsAlloc () ;
      pMapping -> usage    = 0 ; 
   }

   tls_index_ = pMapping -> tls_index ; 
   pMapping -> usage ++ ; 

   UnmapViewOfFile ( pMapping ) ;

   ReleaseMutex ( mutex_ ) ; 
}

inline thread_context :: ~thread_context ()
{
   WaitForSingleObject ( mutex_, INFINITE ) ; 

   mapped * pMapping = (mapped *)MapViewOfFile ( mapping_, FILE_MAP_WRITE, 0, 0, 4096 ) ; 

   pMapping -> usage -- ; 
   if ( pMapping -> usage == 0 ) 
        TlsFree ( pMapping -> tls_index ) ;

   UnmapViewOfFile ( pMapping ) ;

   CloseHandle ( mapping_ ) ;

   ReleaseMutex ( mutex_ ) ; 
   CloseHandle ( mutex_ ) ; 
}

inline LPVOID thread_context :: get () const 
{
   return TlsGetValue(tls_index_) ; 
} 

inline void thread_context :: set ( LPVOID context ) 
{
   TlsSetValue( tls_index_, context ) ; 
} 

