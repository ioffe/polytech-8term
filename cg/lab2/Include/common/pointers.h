#pragma once 

#include <boost\scoped_ptr.hpp>
#include <boost\scoped_array.hpp>
#include <boost\intrusive_ptr.hpp>
#include <boost\shared_ptr.hpp>
#include <boost\shared_array.hpp>
#include <boost\weak_ptr.hpp>
#include <boost\optional.hpp>
#include <boost\utility\in_place_factory.hpp>

using boost::scoped_ptr;
using boost::scoped_array;
using boost::intrusive_ptr;
using boost::shared_ptr;
using boost::shared_array;
using boost::weak_ptr;
using boost::optional;

// -------------- ::intrusive_ptr adapters ---------------
namespace intrusive_ptr_details
{
   struct intrusive_counter
   {
      intrusive_counter() : count( 0 ) {}
      intrusive_counter( intrusive_counter const & ) : count( 0 ) {} 
      intrusive_counter & operator=( intrusive_counter const & ) { return *this; }
      size_t count;
   };
}
#define DECLARE_INTRUSIVE_COUNTER_FWD(name)                                            \
   __forceinline void intrusive_ptr_add_ref( name * p );                               \
   __forceinline void intrusive_ptr_release( name * p );

#define DECLARE_INTRUSIVE_COUNTER(name)                                            \
   intrusive_ptr_details::intrusive_counter _ptr_count_;                                \
   __forceinline friend void intrusive_ptr_add_ref( name * p ) { ++p->_ptr_count_.count; }                \
   __forceinline friend void intrusive_ptr_release( name * p ) { if( --p->_ptr_count_.count == 0 ) delete p; } 

#define INTRUSIVE_SMART_PTR(name) intrusive_ptr<name>

#define DECLARE_INTRUSIVE_COUNTER_MUTABLE_FWD(name)                                      \
   void intrusive_ptr_add_ref( name const * p );                                         \
   void intrusive_ptr_release( name const * p ); 

#define DECLARE_INTRUSIVE_COUNTER_MUTABLE(name)                                          \
   mutable intrusive_ptr_details::intrusive_counter _ptr_count_;                              \
   friend void intrusive_ptr_add_ref( name const * p ) { ++p->_ptr_count_.count; }                \
   friend void intrusive_ptr_release( name const * p ) { if( --p->_ptr_count_.count == 0 ) delete p; } 

#define INTRUSIVE_SMART_PTR_CONST(name) intrusive_ptr<name const>

namespace util
{
   struct intrusive_counter_base
   {
      virtual ~intrusive_counter_base() {}

   private:
      DECLARE_INTRUSIVE_COUNTER(intrusive_counter_base)
   };
}
