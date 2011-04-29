#pragma once

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace threads
{
   typedef
      boost::function0< void >
      thread_func_t;

   struct com_thread
   {
      com_thread( thread_func_t const & func ) : func_( func ) {}

      void operator()()
      {
         CoInitialize( 0 );
         func_();
         CoUninitialize();
      }

   private:
      thread_func_t func_;
   };

   inline boost::thread run_com_thread( thread_func_t const & c )
   {
      return boost::thread( com_thread( c ) );
   }

   namespace details
   {
      template < class Viewer >
      void run_viewer_impl()
      {
         com_impl_obj< Viewer >()->DoModal();
      }

      template < class Viewer, class P1 >
      void run_viewer_impl1( P1 const & p1 )
      {
         com_impl_obj< Viewer >( p1 )->DoModal();
      }
   }

   template < class Viewer >
   boost::thread run_viewer()
   {
      return run_com_thread( &details::run_viewer_impl< Viewer > );
   }

   template < class Viewer, class P1 >
   boost::thread run_viewer( P1 const & p1 )
   {
      return run_com_thread( boost::bind( &details::run_viewer_impl1< Viewer, P1 >, boost::cref( p1 ) ) );
   }
}