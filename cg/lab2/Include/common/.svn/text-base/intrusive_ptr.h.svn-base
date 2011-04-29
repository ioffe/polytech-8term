#pragma once

//  Copied from boost
//  intrusive_ptr.hpp
//
//  Copyright (c) 2001, 2002 Peter Dimov
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
//  See http://www.boost.org/libs/smart_ptr/intrusive_ptr.html for documentation.
//

#include <functional>         // for std::less
#include <iosfwd>            // for std::basic_ostream
#include "common\assert.h"
#include "common\safe_bool.h"

namespace util
{
//
//  intrusive_ptr
//
//  A smart pointer that uses intrusive reference counting.
//
//  Relies on unqualified calls to
//  
//     void intrusive_ptr_add_ref(T * p);
//     void intrusive_ptr_release(T * p);
//
//        (p != 0)
//
//  The object is responsible for destroying itself.
//

template<class T> class intrusive_ptr
{
private:

   typedef intrusive_ptr this_type;

public:

   typedef T element_type;

   __forceinline intrusive_ptr(): p_(0)
   {
   }

   __forceinline intrusive_ptr(T * p, bool add_ref = true): p_(p)
   {
      if(p_ != 0 && add_ref) intrusive_ptr_add_ref(p_);
   }

   template<class U> __forceinline intrusive_ptr(intrusive_ptr<U> const & rhs): p_(rhs.get())
   {
      if(p_ != 0) intrusive_ptr_add_ref(p_);
   }

   __forceinline intrusive_ptr(intrusive_ptr const & rhs): p_(rhs.p_)
   {
      if(p_ != 0) intrusive_ptr_add_ref(p_);
   }

   __forceinline ~intrusive_ptr()
   {
      if(p_ != 0) intrusive_ptr_release(p_);
   }

   template<class U> __forceinline intrusive_ptr & operator=(intrusive_ptr<U> const & rhs)
   {
      this_type(rhs).swap(*this);
      return *this;
   }

   __forceinline intrusive_ptr & operator=(intrusive_ptr const & rhs)
   {
      this_type(rhs).swap(*this);
      return *this;
   }

   __forceinline void reset(T * rhs = 0)
   {
      this_type(rhs).swap(*this);
   }

   __forceinline intrusive_ptr & operator=(T * rhs)
   {
      this_type(rhs).swap(*this);
      return *this;
   }

   __forceinline T * get() const
   {
      return p_;
   }

   __forceinline T & operator*() const
   {
      Assert( p_ );
      return *p_;
   }

   __forceinline T * operator->() const
   {
      Assert( p_ );
      return p_;
   }

   SAFE_BOOL_OPERATOR(p_ != NULL)

   // operator! is a Borland-specific workaround
   __forceinline bool operator! () const
   {
      return p_ == 0;
   }

   __forceinline void swap(intrusive_ptr & rhs)
   {
      T * tmp = p_;
      p_ = rhs.p_;
      rhs.p_ = tmp;
   }

private:

   T * p_;
};

template<class T, class U> inline bool operator==(intrusive_ptr<T> const & a, intrusive_ptr<U> const & b)
{
   return a.get() == b.get();
}

template<class T, class U> inline bool operator!=(intrusive_ptr<T> const & a, intrusive_ptr<U> const & b)
{
   return a.get() != b.get();
}

template<class T> inline bool operator==(intrusive_ptr<T> const & a, T * b)
{
   return a.get() == b;
}

template<class T> inline bool operator!=(intrusive_ptr<T> const & a, T * b)
{
   return a.get() != b;
}

template<class T> inline bool operator==(T * a, intrusive_ptr<T> const & b)
{
   return a == b.get();
}

template<class T> inline bool operator!=(T * a, intrusive_ptr<T> const & b)
{
   return a != b.get();
}

template<class T> inline bool operator<(intrusive_ptr<T> const & a, intrusive_ptr<T> const & b)
{
   return std::less<T *>()(a.get(), b.get());
}

template<class T> void swap(intrusive_ptr<T> & lhs, intrusive_ptr<T> & rhs)
{
   lhs.swap(rhs);
}

// mem_fn support

template<class T> T * get_pointer(intrusive_ptr<T> const & p)
{
   return p.get();
}

template<class T, class U> intrusive_ptr<T> static_pointer_cast(intrusive_ptr<U> const & p)
{
   return static_cast<T *>(p.get());
}

template<class T, class U> intrusive_ptr<T> const_pointer_cast(intrusive_ptr<U> const & p)
{
   return const_cast<T *>(p.get());
}

template<class T, class U> intrusive_ptr<T> dynamic_pointer_cast(intrusive_ptr<U> const & p)
{
   return dynamic_cast<T *>(p.get());
}

// operator<<
template<class Y> std::ostream & operator<< (std::ostream & os, intrusive_ptr<Y> const & p)
{
   os << p.get();
   return os;
}

} // namespace util


// -------------- ::intrusive_ptr adapters ---------------
namespace intrusive_ptr_details
{
   struct IntrusiveCounter
   {
      IntrusiveCounter() : count( 0 ) {}
      IntrusiveCounter( IntrusiveCounter const & ) : count( 0 ) {} 
      IntrusiveCounter & operator=( IntrusiveCounter const & ) { return *this; }
      size_t count;
   };
}

#define DECLARE_INTRUSIVE_COUNTER(name)                                            \
   intrusive_ptr_details::IntrusiveCounter _ptr_count_;                                \
   __forceinline friend void intrusive_ptr_add_ref( name * p ) { ++p->_ptr_count_.count; }                \
   __forceinline friend void intrusive_ptr_release( name * p ) { if( --p->_ptr_count_.count == 0 ) delete p; } 

#define INTRUSIVE_SMART_PTR(name) util::intrusive_ptr<name>

#define DECLARE_INTRUSIVE_COUNTER_MUTABLE(name)                                          \
   mutable intrusive_ptr_details::IntrusiveCounter _ptr_count_;                              \
   friend void intrusive_ptr_add_ref( name const * p ) { ++p->_ptr_count_.count; }                \
   friend void intrusive_ptr_release( name const * p ) { if( --p->_ptr_count_.count == 0 ) delete p; } 

#define INTRUSIVE_SMART_PTR_CONST(name) util::intrusive_ptr<name const>

namespace util
{
   struct intrusive_counter_base
   {
      virtual ~intrusive_counter_base() {}

   private:
      DECLARE_INTRUSIVE_COUNTER(intrusive_counter_base)
   };
}
