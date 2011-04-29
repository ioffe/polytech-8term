#pragma once

#include <boost\shared_ptr.hpp>
#include <map>

template<class T> 
   boost::shared_ptr<T> create_singleton()
   {
      static boost::weak_ptr<T> ptr ; 
      if (! ptr.expired())
         return ptr.lock() ; 

      boost::shared_ptr<T> t ( new T ) ; 
      ptr = t ; 
      return t ; 
   }

template<class T, const size_t N > 
   boost::shared_ptr<T> create_indexed_singleton( size_t index )
   {
      static boost::weak_ptr<T> ptr[N] ; 
      if (! ptr[index].expired())
         return ptr[index].lock() ; 

      boost::shared_ptr<T> t ( new T(index) ) ; 
      ptr[index] = t ; 
      return t ; 
   }

template<class T, class N > 
   boost::shared_ptr<T> create_named_singleton( N const & name )
   {
      typedef boost::weak_ptr<T> t_ptr ; 
      typedef std::map<N, t_ptr> t_ptr_map ; 

      static t_ptr_map ptrs ; 

      t_ptr& ptr = ptrs[name] ; 

      if (! ptr.expired())
         return ptr.lock() ; 

      boost::shared_ptr<T> t ( new T(name) ) ; 
      ptr = t ; 
      return t ; 
   }
