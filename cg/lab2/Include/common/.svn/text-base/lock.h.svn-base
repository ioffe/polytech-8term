#pragma once
#include <winbase.h>
#include "safe_bool.h"


namespace lock
{
   namespace impl
   {
      class noncopyable
      {
         noncopyable(noncopyable const&);
         noncopyable const& operator = (noncopyable const&);

      protected:
         noncopyable() {}
        ~noncopyable() {}
      };
   }


   struct critsec: private impl::noncopyable
   {
      critsec()
      {
         InitializeCriticalSection(&cs_);
      }

     ~critsec()
      {
         DeleteCriticalSection(&cs_);
      }

      void lock() 
      {
         EnterCriticalSection(&cs_);
      }

      bool try_lock()
      {
         return TryEnterCriticalSection(&cs_) != 0;
      }

      void unlock()
      {
         LeaveCriticalSection(&cs_);
      }

   private:
      CRITICAL_SECTION cs_;
   };


   struct scoped_lock: private impl::noncopyable
   {
      scoped_lock(critsec const& cs): cs_( const_cast<critsec *>(&cs) )
      {
         cs_ -> lock();
      }

     ~scoped_lock()
      {
         unlock();
      }

      void unlock () 
      {
         if ( cs_ ) 
         {
            cs_ -> unlock();
            cs_ = NULL ; 
         }
      }

   private:
      critsec * cs_;
   };


   struct scoped_try_lock: private impl::noncopyable
   {
      scoped_try_lock(critsec& cs):
         cs_(cs),
         locked_(cs.try_lock())
      {}

      SAFE_BOOL_OPERATOR(locked_)

     ~scoped_try_lock()
      {
         if (locked_)
            cs_.unlock();
      }

   private:
      critsec&   cs_;
      bool const locked_;
   };


   template <bool Automatic>
   struct event: private impl::noncopyable
   {
      event(): event_handle_(CreateEvent(NULL, Automatic ? FALSE : TRUE, FALSE, NULL)) {}

     ~event()
      {
         CloseHandle(event_handle_);
      }

      void wait()
      {
         WaitForSingleObject(event_handle_, INFINITE);
      }

      bool wait(unsigned timeout_ms)
      {
         return WaitForSingleObject(event_handle_, timeout_ms) == WAIT_OBJECT_0;
      }

      void set()
      {
         SetEvent(event_handle_);
      }

      void reset()
      {
         ResetEvent(event_handle_);
      }

      HANDLE get_handle()
      {
         return event_handle_;
      }

      SAFE_BOOL_OPERATOR(WaitForSingleObject(event_handle_, 0) == WAIT_OBJECT_0)

   private:
      HANDLE event_handle_;
   };

   typedef event<true>  automatic_event;
   typedef event<false> manual_event;
}
