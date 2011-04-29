#pragma once

#include <typeinfo.h>

template<class T, int N = 0x10000, bool b = (sizeof(T) > N) > struct StaticPtr ; 

template<class T>
   struct GlobalPtr
   {
      GlobalPtr () 
      {
         char name[1024] ; 
         sprintf ( name, "GlobalPtr_Mapping_%s", typeid(T).name() ) ; 

         HANDLE mutex = CreateMutexA  ( NULL, FALSE, "GlobalPtr_Mutex" ) ; 
         WaitForSingleObject ( mutex, INFINITE ) ; 

         mapping_ = CreateFileMappingA ( INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(T), name ) ; 
         BOOL exists = GetLastError() == ERROR_ALREADY_EXISTS ; 
         view_    = MapViewOfFile ( mapping_, FILE_MAP_WRITE, 0, 0, 0 ) ; 

         if ( ! exists )
            new ((T *)view_) T(); 

         CloseHandle ( mutex ) ; 
      }
      ~GlobalPtr()
      {
         UnmapViewOfFile ( view_ ) ; 
         CloseHandle ( mapping_ ) ; 
      }
      T const& operator * () const 
      {
         return *(T const *)view_ ; 
      }
      T const * operator -> () const 
      {
         return  (T const *)view_ ; 
      }

   private :
      HANDLE mapping_ ; 
      LPVOID view_ ; 

      GlobalPtr ( GlobalPtr const& ) ; 
      GlobalPtr& operator = ( GlobalPtr const& ) ; 
   } ; 

template<class T>
   struct LocalPtr
   {
      LocalPtr () 
         : ptr_ ( new T() ) 
      {
      }
      ~LocalPtr () 
      {
         delete ptr_ ; 
      }
      T const& operator * () const 
      {
         return *ptr_ ; 
      }
      T const * operator -> () const 
      {
         return  ptr_ ; 
      }
   private:
      T * ptr_ ; 
      LocalPtr ( LocalPtr const& ) ; 
      LocalPtr& operator = ( LocalPtr const& ) ; 
   } ; 

template<class T, int N> struct StaticPtr<T, N, true>  : GlobalPtr<T> {} ;
template<class T, int N> struct StaticPtr<T, N, false> : LocalPtr <T> {} ; 

