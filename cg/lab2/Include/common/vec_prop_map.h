#pragma once

#include "boost/property_map/property_map.hpp"

template< class vec_type >
struct vec_prop_map
{
   typedef          size_t                   key_type;
   typedef typename vec_type::value_type     value_type; 

   vec_prop_map( vec_type & vec ) 
      : vec_(vec) 
   {}

   value_type & operator[]( key_type const& key )
   {
      return vec_[key];
   }
   value_type const& operator[]( key_type const& key ) const
   {
      return vec_[key];
   }

private:
   vec_type & vec_;
};

namespace boost 
{
   template< class vec_type >
   struct property_traits< vec_prop_map< vec_type > > 
   {
      typedef          size_t                   key_type;
      typedef typename vec_type::value_type     value_type; 
      typedef          value_type&              reference;
      typedef          lvalue_property_map_tag  category;
   };

   template< class vec_type >
   void put( vec_prop_map< vec_type > &map, 
      const typename vec_prop_map< vec_type >::key_type& key, 
      const typename vec_prop_map< vec_type >::value_type& val ) 
   { 
      map[key] = val;
   }

   template< class vec_type >
   typename vec_prop_map< vec_type >::value_type get( vec_prop_map< vec_type > const& map, 
      const typename vec_prop_map< vec_type >::key_type& key ) 
   { 
      return map[key];
   }
}
