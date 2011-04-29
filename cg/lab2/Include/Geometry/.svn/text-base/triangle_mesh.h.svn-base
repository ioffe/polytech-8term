#pragma once

#include <boost\noncopyable.hpp>

#include "geometry\triangle_3_fast.h"
#include "geometry\triangle_3_intersection.h"

namespace cg {
   // Класс содержит в себе указатели на буфферы с вершинами и индексами треугольников
   template< typename vertex_type, typename index_type >
      struct triangle_mesh
         : private boost::noncopyable
   {
      triangle_mesh( const vertex_type * vertices, index_type v_count,
                     const point_3i * indices,     index_type t_count )
         : vertices_( vertices )
         , v_count_( v_count )
         , indices_( indices )
         , t_count_( t_count )
         , normals_( t_count )
      {
         for ( size_t i = 0; i != t_count; ++i )
         {
            point_3i const & idx = indices_[ i ];

            vertex_type const & v1 = vertices_[ idx.x ];
            vertex_type const & v2 = vertices_[ idx.y ];
            vertex_type const & v3 = vertices_[ idx.z ];

            normals_[i] = cg::normalized_safe((v2 - v1) ^ (v3 - v1)) ;
         }
      }

   public:
      vertex_type normal( index_type i ) const
      {
         Assert( i < t_count_ );
         return normals_[i] ;
      }

   public:
      // triangle at i
      template< typename triangle_type >
         triangle_type triangle( index_type i ) const
      {
         Assert( i < t_count_ );
         point_3i const & idx = indices_[ i ];

         typedef typename triangle_type::point_type  point_type;
         return triangle_type( point_type( vertices_[ idx.x ] ),
                               point_type( vertices_[ idx.y ] ),
                               point_type( vertices_[ idx.z ] ) );
      }

      template< >
         triangle_3_fast< vertex_type > triangle< triangle_3_fast< vertex_type > >( index_type i ) const
      {
         Assert( i < t_count_ );
         point_3i const & idx = indices_[ i ];
         return triangle_3_fast< vertex_type >( vertices_, idx.x, idx.y, idx.z, normals_[ i ] );
      }

      template< >
         triangle_3_rti triangle< triangle_3_rti >( index_type i ) const
      {
         Assert( i < t_count_ );
         point_3i const & idx = indices_[ i ];
         
         vertex_type const & v1 = vertices_[ idx.x ];
         vertex_type const & v2 = vertices_[ idx.y ];
         vertex_type const & v3 = vertices_[ idx.z ];

         typedef triangle_t<typename vertex_type::scalar_type, 3> triangle_type  ; 

         return triangle_3_rti( v1, v3, v2, normals_[ i ] );
      }

      vertex_type vertex( index_type i ) const
      {
         Assert( i < v_count_ );
         return vertices_[ i ];
      }

      // count
      index_type   vertices_count()  const { return v_count_; }
      index_type   triangles_count() const { return t_count_; }

      // vertex_iterator
      typedef vertex_type const * vertex_iterator_type;
      vertex_iterator_type v_begin( ) const { return vertices_; }
      vertex_iterator_type v_end( )   const { return vertices_ + v_count_; }

      // triangle_iterator
      template< typename triangle_type >
         struct triangle_iterator_type
      {
         typedef std::forward_iterator_tag         iterator_category;
         typedef typename triangle_type            value_type;
         typedef typename triangle_type const &    reference;
         typedef typename triangle_type const *    pointer;
         typedef index_type                        difference_type;

         triangle_iterator_type( triangle_mesh const * mesh, index_type idx )
            : mesh_( mesh )
            , idx_ ( idx )
         { }

         // ==, !=
         friend bool operator == ( triangle_iterator_type const & a, triangle_iterator_type const & b )
         { 
            Assert( a.mesh_ == b.mesh_ ); 
            return a.idx_ == b.idx_; 
         }

         friend bool operator != ( triangle_iterator_type const & a, triangle_iterator_type const & b )
         { 
            return !( a == b ); 
         }

         // +=, +
         triangle_iterator_type & operator+=( difference_type n )  { idx_ += n; clean( ); return *this; }
         triangle_iterator_type   operator+ ( difference_type n ) const  
         { 
            triangle_iterator_type temp( *this );
            return temp += n;
         }

         // ++
         triangle_iterator_type & operator++()  { ++idx_; clean( ); return *this; }
         triangle_iterator_type   operator++( int )
         {
            triangle_iterator_type tmp ( *this );
            ++ *this;
            return tmp;
         }

         // -
         difference_type  operator-( triangle_iterator_type const & it ) const 
         { 
            Assert( mesh_ == it.mesh_ ); 
            return max( idx_, it.idx_ ) - min( idx_, it.idx_ );
         }

         // *, ->
         reference operator*( ) const { get_triangle( ); return  *triangle_; }
         pointer  operator->( ) const { get_triangle( ); return &*triangle_; }

         // index
         difference_type index() const { return idx_; }

      private:
         void get_triangle( ) const
         {
            if( !triangle_ )
               triangle_ = mesh_->triangle< triangle_type >( idx_ );
         }

         void clean( ) const  { triangle_.reset( ); }

      private:
         triangle_mesh const * mesh_;
         index_type            idx_;
         
         mutable boost::optional< triangle_type > triangle_;      
      };

      template< typename triangle_type >
         triangle_iterator_type< triangle_type > t_begin( ) const 
      { 
         return triangle_iterator_type< triangle_type >( this, 0 ); 
      }

      template< typename triangle_type >
         triangle_iterator_type< triangle_type > t_end( ) const 
      { 
         return triangle_iterator_type< triangle_type >( this, t_count_ ); 
      }

      bool empty() const
      {
         Assert( ( t_count_ == 0 ) == ( v_count == 0 ) );
         return t_count_ == 0;
      }

   //
   private:
      const vertex_type * const vertices_;
      const index_type          v_count_;

      const point_3i    * const indices_;
      const index_type          t_count_;

   private:
      std::vector< vertex_type > normals_;
   };

   // Вычисление bounding box
   template< typename vertex_type, typename index_type > 
      rectangle_3 bounding( triangle_mesh< vertex_type, index_type > const & mesh )
   {
      typedef triangle_mesh< vertex_type, index_type >::vertex_iterator_type  VIT;

      rectangle_3 rect;
      for( VIT p = mesh.v_begin( ), q = mesh.v_end( ); p != q; ++p )
         rect |= point_3( p->x, p->y, p->z ) ;

      return rect;
   }

   // Вычисление радиуса 
   template< typename vertex_type, typename index_type > 
      double radius( triangle_mesh< vertex_type, index_type > const & mesh )
   {
        typedef triangle_mesh< vertex_type, index_type > :: vertex_iterator_type  VI;
        
        double rad = 0;
        for( VI p = mesh.v_begin( ), q = mesh.v_end( ); p != q; ++p )
            cg::make_max( rad, ( double ) norm_sqr( *p ) );

        return cg::sqrt( rad );
   }
}
