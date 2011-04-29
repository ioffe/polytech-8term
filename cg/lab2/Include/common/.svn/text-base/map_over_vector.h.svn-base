#pragma once

#include <vector>
#include <utility>

namespace util
{
   template < class Key, class Value >
   struct map_over_vector
   {
      typedef std::pair< Key, Value >              value_type;
      typedef std::vector< value_type >            vector_type;
      typedef typename vector_type::iterator       iterator;
      typedef typename vector_type::const_iterator const_iterator;

      Value & operator []( Key const & key )
      {
         iterator it = find( key );
         if ( it != end() )
            return it->second;

         v_.push_back( std::make_pair( key, Value() ) );
         return v_.back().second;
      }

      iterator find( Key const & key )
      {
         iterator it = begin();
         for ( ; it != end(); ++it )
            if ( it->first == key )
               break;

         return it;
      }

      const_iterator find( Key const & key ) const
      {
         iterator it = begin();
         for ( ; it != end(); ++it )
            if ( it->first == key )
               break;

         return it;
      }

      iterator begin()              { return v_.begin(); }
      iterator end()                { return v_.end(); }

      const_iterator begin() const  { return v_.begin(); }
      const_iterator end() const    { return v_.end(); }

   private:
      std::vector< value_type > v_;
   };
}