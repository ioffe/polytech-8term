#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include "common/util.h"

// object prototypes registry
namespace obj_reg
{
   // Collection of object prototypes that implement specific interface.
   // This class is useful when there are several objects that implement an interface 
   // and we want to somehow use their prototypes without knowledge of their specific type.
   template < class T >
      struct registry
         : private boost::noncopyable
   {
      typedef
         std::vector< T * >
         atoms_type;

      static registry & instance( )
      {
         static registry reg;
         return reg;
      }

      void put( T & t )
      { 
         atoms_.push_back( &t );
      }

      typedef typename atoms_type::iterator        iterator;
      typedef typename atoms_type::const_iterator  const_iterator;

      iterator       begin( )       { return atoms_.begin();   }
      iterator       end( )         { return atoms_.end();     }

      const_iterator begin( ) const { return atoms_.begin();   }
      const_iterator end( )   const { return atoms_.end();     }
      
   private:
      atoms_type atoms_;
   };

   template < class Derived >
      struct prototype
   {
      prototype()
      {
         static Derived instance;
      }
   };

   namespace details
   {
      template < class Base, class Derived >
         struct helper_impl
      {
         helper_impl()
         {
            static Derived instance;
            static bool registered = false;

            if ( !registered )
            {
               registered = true;

               typedef registry< Base > registry_type;
               registry_type::instance().put( instance );
            }
         }
      };
   }

   template < class Base, class Derived >
      struct helper
   {      
   protected:
      helper() { impl_; }

   private:
      static details::helper_impl< Base, Derived > impl_;
   };

   template < class Base, class Derived >
   details::helper_impl< Base, Derived > helper< Base, Derived >::impl_;
}