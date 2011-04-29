#pragma once

namespace cg {
namespace polyoffset
{  

   /*    структура, которая ставит равноудаленную точку на расстрояние d от двух отрезков,
         образующих ломаную.  Описание алгоритма: эта точка может лежать лишь на трех прямых: либо это биссектриса 
         угла ломанной, либо парабола ( как множество точек, равноудаленных от прямой и точки ), либо серединный
         перпендикуляр двух крайних точек ломаной - выбор прямой зависит от d ( distance ). 
         [0, first_limit_distance] - точка лежит на биссектрисе, 
         [first_limit_distance, second_limit_distance] - точка лежит на параболе
         [second_limit_distance, INF ) - точка лежит на перпендикуляре */

   namespace details
   {
      template < typename Scalar >
         struct equidistant_point_algo
      {
         typedef              point_t< Scalar, 2 >        Point;

         // determine initial data ( angle, distance limits, etc.  )
         equidistant_point_algo( Point const & a, Point const & b, Point const & c )
            : vertex_( b )
         {          
            Point const e1 ( a - b );
            Point const e2 ( c - b );

            Assert( !eq_zero( e1 ) && !eq_zero( e2 ) );
            
            first_short_ = cg::norm_sqr( e1 ) < cg::norm_sqr( e2 );
            short_edge_  = ( first_short_ ) ? e1 : e2;
            long_edge_   = ( first_short_ ) ? e2 : e1;
            angle_       = ( first_short_ ) 
                              ? cg::angle( short_edge_, long_edge_ ) 
                              : cg::angle( long_edge_, short_edge_ );
            flat_angle_  = cg::eq( angle_, cg::pi );
           
            bs_point_n = calc_bisector( );
            
            if ( !flat_angle_ )
            {
               // determine distance limits
               first_limit_dist_ = ( Scalar )( abs( tan( angle_ / 2 ) ) * cg::norm( short_edge_ ) );
               
               Point  const rel02  = short_edge_ - long_edge_;
               Scalar const dist02 = ( Scalar ) cg::norm( rel02 );
               
               Scalar alpha = 
                  ( Scalar )( cg::pi / 2 - acos( -long_edge_ * rel02 / ( cg::norm(long_edge_) * dist02 ) ) );

               second_limit_dist_  = ( Scalar )( 0.5 * dist02 / cos( alpha ) );
            }
         }

         //returns a point distant from sections on 'dist'
         Point operator( )( Scalar dist ) const
         {  
            if ( flat_angle_ )
               return vertex_ + bs_point_n * dist ;
            
            if ( angle_ < cg::pi )
               dist = -dist;
            
            if ( dist < 0 )
               return vertex_ + bs_point_n * dist;

            if ( dist < first_limit_dist_ )
            {
               Scalar distByBisector = ( Scalar )( dist / sin( angle_ / 2 ) );
               return vertex_ + bs_point_n * distByBisector;
            }

            if ( dist < second_limit_dist_ )
            {
               Point const long_norm = cg::normalized_safe( long_edge_ );
               Point const pr_long_short ( ( long_norm * short_edge_ ) * long_norm );

               Point resVector = short_edge_- pr_long_short;

               Scalar const delta     = dist - cg::norm( resVector );
               Scalar const deltaPerp = cg::sqrt( cg::sqr( dist ) - cg::sqr( delta ) );

               resVector += delta * cg::normalized_safe( resVector )
                         +  long_norm * deltaPerp
                         +  pr_long_short;

               return resVector + vertex_;
            }

            Point const  rel02 ( short_edge_ - long_edge_ );
            Scalar const delta = ( Scalar ) cg::sqrt( cg::sqr( dist ) - cg::norm_sqr( rel02 / 2 ) );

            Point const  mid_perp( cg::normalized_safe( Point(rel02.y, -rel02.x) ) );

            Point res ( ( short_edge_ + long_edge_ ) / 2 );

            if ( angle_ > cg::pi && !first_short_ || angle_ < cg::pi && first_short_ )
               res -= mid_perp * delta;
            else
               res += mid_perp * delta;

            return res + vertex_;            
         } //end of 'operator () (double)'

      private:  
         // determine bisector vector of a angle. 
         Point calc_bisector( ) const
         {
            if ( !flat_angle_ )
            {  
               Point bis ( cg::normalized( short_edge_ ) + cg::normalized( long_edge_  ) );

               if( !cg::eq_zero( bis ) )
                  return cg::normalized( bis );
            }

            Point const dir_vec ( first_short_ ? -short_edge_ : short_edge_ );
            Point const dir_norm( -dir_vec.y, dir_vec.x );
               
            return cg::normalized_safe( dir_norm );
         }

      private:
         Point    vertex_;    
         Point    short_edge_;
         Point    long_edge_; 
         Point    bs_point_n; 
         Scalar   first_limit_dist_, second_limit_dist_;
         double   angle_;     // 0..2*pi
         bool     first_short_;
         bool     flat_angle_;
      }; //end of 'equidistant_point_algo' struct
   } 

   template< typename Scalar >
      point_t< Scalar, 2 > equidistant_point( point_t< Scalar, 2 > const & a, 
                                              point_t< Scalar, 2 > const & b, 
                                              point_t< Scalar, 2 > const & c, Scalar d )
   {
      return details::equidistant_point_algo< Scalar >( a, b, c )( d );
   }
}}
