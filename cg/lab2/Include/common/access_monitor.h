#pragma once
#include <boost/optional.hpp>
#include <boost/thread.hpp>


//
// `access_monitor<T>` is a wrapper that lets you use `T` from multiple threads,
// with automatic scoped-based locking/unlocking
//
// example:
//
// typedef std::vector<int>  T;
// typedef access_monitor<T> AM;
//
// AM am;
//
// void thread_1()
// {
//    AM::scoped_lock_type x = am.lock();
//
//    // from now on, `x` holds lock assocciated with `T` until end of scope
//
//    std::sort(x->begin(), x->end());
//    x->pop_back();
//    x->pop_back();
// }
//
// void thread_2()
// {
//    global_foo()
//
//    {
//       AM::scoped_lock_type y = am.lock();
//
//       // from now on, `y` holds lock assocciated with `T` until end of scope
//
//       y->erase(std::find(y->begin(),
//                          y->end(),
//                          0),
//                y->end());
//
//       y->push_back(0);
//    }
//
//    global_bar();
// }
//


template <typename T, typename L>
struct scoped_access_lock;


namespace access_monitor_impl
{
   template <typename T, typename L>
   struct moveable
   {
      moveable(T* data, L& lock):
         lock_(lock.move()),
         data_(data)
      {
      }

      moveable(moveable& m):
         lock_(m.lock_.move()),
         data_(m.data_)
      {
      }

      template <typename T, typename L>
      friend struct scoped_access_lock;

   private:
      L        lock_;
      T* const data_;
   };
}


template <typename T, typename Lockable>
struct scoped_access_lock
{
   typedef T value_type;

private:
   typedef boost::unique_lock<Lockable> lock_type;

public:
   scoped_access_lock(access_monitor_impl::moveable<value_type, lock_type>& m):
      lock_(m.lock_.move()),
      data_(m.data_)
   {
   }

   value_type const* operator -> () const
   {
      return data_;
   }

   value_type* operator -> ()
   {
      return data_;
   }

   value_type const& operator * () const
   {
      return *data_;
   }

   value_type& operator * ()
   {
      return *data_;
   }

private:
   scoped_access_lock();

private:
   lock_type lock_;
   T* const  data_;
};


template <typename T, typename Lockable = boost::recursive_mutex>
struct access_monitor
{
   typedef T                                     value_type;
   typedef scoped_access_lock<T, Lockable>       scoped_lock_type;
   typedef scoped_access_lock<T const, Lockable> const_scoped_lock_type;

private:
   typedef boost::unique_lock<Lockable> lock_type;

public:
   typedef access_monitor_impl::moveable<value_type const, lock_type> const_locked_type;
   typedef access_monitor_impl::moveable<value_type,       lock_type> locked_type;

public:

   access_monitor():
      data_(boost::in_place())
   {
   }

   template <typename Expr>
   explicit access_monitor(Expr const& e):
      data_(e)
   {
   }

   const_locked_type lock() const
   {
      lock_type lock(mutex_);
      return const_locked_type(get_pointer(data_), lock);
   }

   locked_type lock()
   {
      lock_type lock(mutex_);
      return locked_type(get_pointer(data_), lock);
   }

private:
   mutable Lockable   mutex_;
   boost::optional<T> data_;
};
