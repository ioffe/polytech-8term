#pragma once 
#include "safe_bool.h"


template<HANDLE invalid_value>
   struct win_handle_t
   {
      win_handle_t () 
         : handle_ ( invalid_value ) 
      {}

      win_handle_t ( HANDLE handle ) 
         : handle_ ( handle ) 
      {}

      ~win_handle_t () 
      {
         reset () ; 
      }

      HANDLE operator * () const
      {
         return handle_ ; 
      }

      HANDLE get () const
      {
         return handle_ ; 
      }

      SAFE_BOOL_OPERATOR(handle_ != invalid_value)

      bool operator ! () const
      {
         return handle_ == invalid_value ; 
      }

      void reset ( HANDLE handle = invalid_value ) 
      {
         if ( handle_ != invalid_value ) 
            CloseHandle ( handle_ ) ; 
         handle_ = handle ; 
      }
   private:
      HANDLE handle_ ; 

   private:
      win_handle_t ( win_handle_t const& ) ; 
      win_handle_t& operator = ( win_handle_t const& ) ; 
   } ; 

typedef win_handle_t<INVALID_HANDLE_VALUE> file_handle ; 
typedef win_handle_t<NULL>                 kernel_handle ; 

