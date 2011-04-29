#pragma once
#include "common/wintimer.h"


struct timer
{
   typedef boost::function<void ()> callback_t;

   timer(callback_t const& callback = callback_t(),
         unsigned timeout = std::numeric_limits<unsigned>::max(),
         bool start_now = false)
   {
      set_timeout(timeout);
      set_callback(callback);

      if (start_now)
         start();
   }

   void start()
   {
      assert(callback_);

      timer_ = boost::in_place(this, &timer::exec, timeout_);
   }

   void stop()
   {
      timer_.reset();
   }

   void set_timeout(unsigned msec)
   {
      timeout_ = msec;
      if (timer_)
         start();
   }

   void set_callback(callback_t const& c)
   {
      callback_ = c;
   }

   bool enabled() const
   {
      return timer_;
   }

private:
   void exec()
   {
      assert(callback_);
      if (callback_)
         callback_();
   }

private:
   unsigned timeout_;
   callback_t callback_;
   boost::optional<win_timer> timer_;
};
