#pragma once

#include <map>

namespace util
{
   template < class Key, class Index = int >
   struct indexer
   {
      typedef Key                         key_t;
      typedef Index                       index_t;
      typedef std::map< key_t, index_t >  map_t;

      index_t operator()( key_t const & key )
      {
         map_t::const_iterator it = map_.find( key );
         if ( it == map_.end() )
         {
            index_t idx = map_.size();
            map_.insert( it, std::make_pair( key, idx ) );
            return idx;
         }
         else
            return it->second;
      }

      size_t size() const { return map_.size(); }

   private:
      map_t map_;
   };

   template < class Key >
   struct vector_indexer
   {
      typedef Key                   key_t;
      typedef size_t                index_t;
      typedef std::vector< Key >    map_t;

      index_t operator()( key_t const & key )
      {
         map_t::iterator it = std::find( map_.begin(), map_.end(), key );
         if ( it == map_.end() )
         {
            index_t idx = map_.size();
            map_.push_back( key );
            return idx;
         }
         else
            return std::distance( map_.begin(), it );
      }

      size_t size() const { return map_.size(); }

   private:
      map_t map_;
   };
}