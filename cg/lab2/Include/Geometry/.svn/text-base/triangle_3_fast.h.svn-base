#pragma once

#include "Geometry\triangle_3_intersection.h"

namespace cg
{

template<class Vertex = point_3>
struct triangle_3_fast
{
   typedef Vertex vertex_type;

   // конструктор
   triangle_3_fast( vertex_type const *v, int i1, int i2, int i3 ) 
      : v_ ( v  )
      , i1_( i1 )
      , i2_( i2 )
      , i3_( i3 )
   { calc_normal(); }

   triangle_3_fast( vertex_type const *v, int i1, int i2, int i3, vertex_type const &n ) 
      : v_ ( v  )
      , i1_( i1 )
      , i2_( i2 )
      , i3_( i3 )
      , n_ ( n  ) 
   { }

   // доступ к private членам
   vertex_type const& normal() const { return n_; }
   vertex_type const& v1    () const { return v_[ i1_ ]; }
   vertex_type const& v2    () const { return v_[ i2_ ]; }
   vertex_type const& v3    () const { return v_[ i3_ ]; }

   vertex_type const& operator[](size_t idx) const
   {
      Assert(idx < 3);
      return v_[(0 == idx) ? i1_ : ((1 == idx) ? i2_ : i3_)];
   }

   long checked_id    () const { return checked_id_; }

   // методы
   template< class Traits > 
      bool collinear_has_on(point_3 const &pt, Traits const &traits) const ;

private:
   // считаем нормаль и расстояние до плоскости
   void calc_normal()
   {
      n_ = cg::normalized_safe((v2() - v1()) ^ (v3() - v1()));
   }

private:
   int         i1_, i2_, i3_;  // индексы вершин
   vertex_type n_;             // нормаль к плоскости треугольника

   vertex_type const * v_;     // массив вершин

   mutable long checked_id_;
};

template<class S>
point_t< S, 3 > const& normal( triangle_3_fast< point_t< S, 3 > > const& tri )
{
   return tri.normal();
}

// проверка принадлежности точки
template<class Vertex>
   template <class Traits> 
      inline bool triangle_3_fast<Vertex> :: collinear_has_on(point_3 const &pt, Traits const &traits) const
{
   typename Traits::point_in_triangle 
      point_in_triangle = traits.point_in_triangle_object();

   if ( !eq_zero( normal().z ) )
   {
      return point_in_triangle( *this, pt, traits.xy() );
   } else
   if ( !eq_zero( normal().y ) )
   {
      return point_in_triangle( *this, pt, traits.xz() );
   } else
   {
      return point_in_triangle( *this, pt, traits.yz() );
   }
}

template<class Vertex = point_3>
   struct cartesian_for_triangle_3_fast : public cartesian<triangle_3_fast<Vertex> > {};

template<class Vertex>
inline range_2 calc_zrange( triangle_3_fast <Vertex> const &t )
{
   return range_2( t.v1().z, t.v2().z ).unite( t.v3().z );
}

} // end of namespace cg
