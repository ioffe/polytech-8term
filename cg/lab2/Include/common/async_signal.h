#pragma once 

#include <boost/signals2/signal.hpp>
#include "w32/function_queue.h"

template< typename signature>
struct async_signal 
   : boost::noncopyable
{
   typedef boost::signals2::signal<signature>   signal_t;
   typedef typename signal_t::slot_type         slot_type;
   typedef boost::signals2::connection          connection_t;

   connection_t connect(slot_type const& slot)
   {
      return sig_.connect(slot);
   }
   
   void operator()()
   {
      // unfortunately I can't use combiner here 
      queue_.enqueue(cref(sig_));
   }

private:
   signal_t                sig_;
   w32::thread_function_queue   queue_;
};