// OpenMP utility functions and classes

#pragma once

#include <boost/noncopyable.hpp>
 
namespace omp
{
#ifdef _OPENMP

   inline int get_max_threads()
   {
      return omp_get_max_threads();
   }

   inline int get_thread_num()
   {
      return omp_get_thread_num();
   }

   struct lock
      : boost::noncopyable
   {
      lock()
      {
         omp_init_lock( &lock_ );
      }

      ~lock()
      {
         omp_destroy_lock( &lock_ );
      }

      void acquire()
      {
         omp_set_lock( &lock_ );
      }

      void release()
      {
         omp_unset_lock( &lock_ );
      }

   private:
      omp_lock_t  lock_;
   };

   struct guard
      : boost::noncopyable
   {
       guard( lock & l )
          : lock_( l )
       {
          lock_.acquire();
       }

       ~guard()
       {
          lock_.release();
       }
    
   private:
       lock & lock_;
   };

#else

   inline int get_max_threads()
   {
      return 1;
   }

   inline int get_thread_num()
   {
      return 0;
   }

   struct lock
      : boost::noncopyable
   {
      void acquire() {}
      void release() {}
   };

   struct guard
      : boost::noncopyable
   {
       guard( lock & ) {}
       ~guard() {}
   };

#endif
}