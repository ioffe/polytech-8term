#pragma once

#include <stack>
#include <boost/optional.hpp>
#include "common/util.h"

namespace cg 
{

   namespace algos 
   {

      namespace details
      {
         template < class point_type, class segment_type, class iterator_type >
            struct farest_vertex
         {
            farest_vertex( point_type const & a, point_type const & b )
               : a_( a )
               , b_( b )
               , dist( 0 )
            {}

            void operator( ) ( iterator_type iterator )
            {
               boost::optional< double > 
                  dist_new;               

               if ( a_ == b_ )
               {
                  dist_new.reset( distance( *iterator, a_ ) * 0.5 + distance( *iterator, b_ ) * 0.5 );
               }
               else
               {
                  dist_new.reset( distance( segment_type( a_, b_ ), *iterator ) );
               }

               if ( *dist_new >= dist )
                  iterator_res = iterator,
                  dist = *dist_new;
            }

            iterator_type result( ) const { return iterator_res; }

         private:
            point_type a_;
            point_type b_;
            double dist;
            iterator_type iterator_res;
         };

         template < class FwdIter, class Processor >
            Processor find_vertex( FwdIter p, FwdIter q, Processor proc )
         {
            for ( ; p != q; ++p )
               proc( p );

            return proc;
         }

      }

      template < class iterator_type >
      struct PointFunctor 
      {
         typedef 
            typename std::iterator_traits< iterator_type >::value_type 
            point_type ;

         point_type operator() ( iterator_type it ) const { return *it ; }
      };

      template < class iterator_type >
      struct IndexFunctor  
      {
         typedef 
            typename std::iterator_traits< iterator_type > :: difference_type
            dif_type ;

         IndexFunctor( iterator_type& begin )
            : begin_ ( begin )
         {
         }

         dif_type operator() ( iterator_type it ) const { return std::distance( begin_, it ) ; }

      private:
         iterator_type begin_  ;
      };
      
      // Douglas-Peucker polyline simplification algorithm
      template < class FwdIter, class OutIter >
         OutIter simplify_polyline_points ( FwdIter p, FwdIter q, double prec, OutIter output )
      {
         PointFunctor< FwdIter > func ;
         return simplify_polyline( p, q, prec, output, func ) ;
      }

      template < class FwdIter, class OutIter >
         OutIter simplify_polyline_indexes ( FwdIter p, FwdIter q, double prec, OutIter output )
      {
         IndexFunctor< FwdIter > func ( p ) ;
         return simplify_polyline( p, q, prec, output, func ) ;
      }

      template < class FwdIter, class OutIter, template < class InIter > class ResultFunc  >
         OutIter simplify_polyline ( FwdIter p, FwdIter q, double prec, OutIter output, ResultFunc < FwdIter > const& res_func )
      {
         if ( std::distance( p, q ) < 3 )
         {
            Assert( 0 );
            for ( ; p != q; ++p )
               *output++ = res_func(p);
         }

         typedef std::iterator_traits< FwdIter >::value_type                  point_type;
         typedef segment_t<point_type::scalar_type, point_type::dimension>    segment_type;
         typedef details::farest_vertex< point_type, segment_type, FwdIter >  fva_type;
         typedef std::pair< FwdIter, FwdIter >                                range_type;
         typedef std::stack< range_type >                                     dp_stack_type;

         boost::optional< point_type > last_point;

         dp_stack_type dp_stack;
         dp_stack.push( std::make_pair( p, q ) );
         do
         {
            range_type rg = dp_stack.top( );
            dp_stack.pop( );


            size_t nPoints = std::distance( rg.first, rg.second );
            if ( nPoints < 3 )
            {
               for ( FwdIter p = rg.first; p != rg.second; ++p )
                  if ( !( last_point && *last_point == *p ) )
                     *output++ = res_func ( p ),
                     last_point.reset( *p );

               continue;
            }

            Assert( *rg.first != *util::prev( rg.second ) );

            FwdIter r = find_vertex( util::next( rg.first ), util::prev( rg.second ), fva_type( *rg.first, *util::prev( rg.second  ) ) ).result( );
            
            if ( distance( segment_type( *rg.first, *util::prev( rg.second  ) ), *r ) > prec ) 
            {
               dp_stack.push( std::make_pair( r, rg.second ) );
               dp_stack.push( std::make_pair( rg.first, util::next( r ) ) );
            }
            else
            {
               if ( !( last_point && *last_point == *rg.first ) )
                  *output++ = res_func( rg.first ),
                  last_point.reset( *rg.first );

               if ( !( last_point && *last_point == *util::prev( rg.second  ) ) )
                  *output++ = res_func( util::prev( rg.second ) ),
                  last_point.reset( *util::prev( rg.second  ) );
            }
         }
         while ( !dp_stack.empty( ) );

         return output;
      }
   }
}
