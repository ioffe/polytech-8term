#pragma once

#include <fstream>

#include "Iterators/null_iterator.h"
#include "Geometry/Triangulation/cgal_headers.h"
#include "Geometry/empty.h"
#include "Geometry/triangle_raster_aux.h"
#include "Geometry/cgal_predicates.h"
#include "Geometry/polygon_2_fwd.h"
#include "Geometry/DupPointsEliminator.h"
#include "Geometry/point_io.h"

struct Mesh;

namespace cg            {
namespace triangulation {

   namespace details
   {
      template < class VertexHandle >
      unsigned __stdcall dump_func(void * param )
      {
         typedef
            std::vector< std::pair< VertexHandle, VertexHandle > >
            triangulation_constraints_t;

         triangulation_constraints_t const & v =
            *reinterpret_cast< triangulation_constraints_t const * >( param );

         std::ofstream ofs( "c:\\triangulation_dump.txt" );
         ofs.precision( 20 );
         ofs.setf( std::ios::fixed );
         for ( size_t i = 0; i < v.size(); ++i )
            ofs << cg::point_2( v[i].first->point().x(), v[i].first->point().y() ) << " "
                << cg::point_2( v[i].second->point().x(), v[i].second->point().y() ) << "\n";
         ofs.flush();

         return 0;
      }

      template < class Gt, class Tds >
      struct constrained_delaunay_triangulation
         : CGAL::Triangulation_hierarchy_2< CGAL::Constrained_Delaunay_triangulation_2< Gt, Tds, CGAL::Exact_predicates_tag > >
       {
         typedef CGAL::Triangulation_hierarchy_2< CGAL::Constrained_Delaunay_triangulation_2< Gt, Tds, CGAL::Exact_predicates_tag > > base;
         //using base::insert;

         Vertex_handle insert( Point const & p )
         {
            inserted_vertices_.clear();
            
            Locate_type lt;
            int li;
            Face_handle f = locate( p, lt, li );

            if ( lt != VERTEX )
            {
               Vertex_handle h = do_insert( p, lt, f, li );
               on_new_vertex( h );
               return h;
            }
            else
               return f != NULL ? f->vertex(li) : do_insert( p, lt, f, li );
         }

         void insert( Vertex_handle p, Vertex_handle q )
         {
            inserted_vertices_.clear();

            if ( p != q )
            {
               if ( has_constraint( p, q ) )
                  return;

               log_constraint( p, q );

               __try
               {
                  insert_constraint( p, q );
               }
               __except( EXCEPTION_EXECUTE_HANDLER )
               {
                  dump_constraints_log();
                  throw_fail_exception();
               }
            }
         }

         bool has_constraint( Vertex_handle p, Vertex_handle q )
         {
            return inserted_edges_.find( std::make_pair( p, q ) ) != inserted_edges_.end();
         }

         void log_constraint( Vertex_handle p, Vertex_handle q )
         {
            inserted_edges_.insert(std::make_pair(p, q));
            inserted_edges_.insert(std::make_pair(q, p));

            constraints_log_.push_back( std::make_pair( p, q ) );
         }

         void throw_fail_exception()
         {
            throw std::runtime_error( "insert_constraint failed in the CGAL triangulation" );
         }

         void dump_constraints_log()
         {
            __if_exists (_beginthreadex)
            {
               HANDLE h = (HANDLE)_beginthreadex( 0, 0, dump_func< Vertex_handle >, &constraints_log_, 0, 0 );
               WaitForSingleObject( h, INFINITE );
               CloseHandle( h );
            }
         }

         void remove_constraint( Vertex_handle p, Vertex_handle q ) 
         {
            if ( p == q )
               return;

            Face_handle f;
            int i;

            inserted_edges_.erase(std::make_pair(p, q));
            inserted_edges_.erase(std::make_pair(q, p));

            if ( is_edge( p, q, f, i ) )
               remove_constrained_edge( f, i );
         }

         void remove_constraints()
         {
            inserted_edges_.clear();
            for( Finite_vertices_iterator it = vertices_begin( ); it != vertices_end( ); ++it )
               remove_incident_constraints( it );
         }

         Vertex_handle intersect(Face_handle f, int i, Vertex_handle vaa, Vertex_handle vbb) 
         {
            Vertex_handle  vcc, vdd;
            vcc = f->vertex(cw(i));
            vdd = f->vertex(ccw(i));

            const Point& pa = vaa->point();
            const Point& pb = vbb->point();
            const Point& pc = vcc->point();
            const Point& pd = vdd->point();

            Point pi; //creator for point is required here
            bool ok  = intersection(geom_traits(), pa, pb, pc, pd, pi, CGAL::Exact_predicates_tag() );

            Vertex_handle vi;
            if ( !ok ) {  //intersection detected but not computed
               typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
               Kernel::Exact_kernel::Intersect_2 intersect_obj;
               typedef Kernel::Exact_kernel::Segment_2 Segment_2;
               typedef Kernel::Exact_kernel::Point_2 Point_2;
               CGAL::Object result = intersect_obj(
                  Segment_2(Point_2(pa.x(), pa.y()), Point_2(pb.x(), pb.y())),
                  Segment_2(Point_2(pc.x(), pc.y()), Point_2(pd.x(), pd.y())));

               Point_2 pt;
               Segment_2 iseg;
               if (assign(pt, result))
               {
                  if (pa == pc || pa == pd)
                  {
                     vi = vaa;
                  }
                  else if (pb == pc || pb == pd)
                  {
                     vi = vbb;        
                  }
                  else
                  {
                     vi = do_insert( CGAL::Cartesian_converter<Kernel::Exact_kernel, Kernel>()(pt), f, i );
                     on_new_vertex ( vi );
                  }
               }
               else
                  CGAL_triangulation_assertion(false);
               /*int i = limit_intersection(geom_traits(), pa, pb, pc, pd, CGAL::Exact_predicates_tag() );
               switch(i){
               case 0 : vi = vaa; break;
               case 1 : vi = vbb; break;
               case 2 : vi = vcc; break;
               case 3 : vi = vdd; break; 
               }*/
            }
            else //intersection computed
            { 
               vi = do_insert( pi, f, i );
               on_new_vertex( vi );
            }

            // vi == vc or vi == vd may happen even if intersection==true
            // due to approximate construction of the intersection
            if (vi != vcc && vi != vdd) { 
               insert_constraint(vcc,vi); 
               insert_constraint(vi, vdd);
            } 
            else {
               insert_constraint(vcc,vdd);
            }
            return vi; 
         }

         std::vector< Vertex_handle > const & inserted_vertices() const
         {
            return inserted_vertices_;
         }         

      protected:
         void on_new_vertex( Vertex_handle h )
         {
            inserted_vertices_.push_back( h );
         }

         virtual Vertex_handle do_insert( Point const & pt, Locate_type lt, Face_handle f, int li )
         {
            return base::insert( pt, lt, f, li );
         }

         virtual Vertex_handle do_insert( Point const & pt, Face_handle f, int i )
         {
            remove_constraint( f->vertex(cw(i)), f->vertex(ccw(i)) );
            return virtual_insert( pt, f );
         }

      private:            
         struct Vertex_handle_pair
            : public std::pair< Vertex_handle, Vertex_handle >
         {
            Vertex_handle_pair ( std::pair< Vertex_handle, Vertex_handle > const &a )
               : std::pair< Vertex_handle, Vertex_handle > (a)
            {}
         };

         std::set< Vertex_handle_pair > inserted_edges_;
         std::vector< Vertex_handle > inserted_vertices_;
         std::vector< std::pair< Vertex_handle, Vertex_handle > > constraints_log_;
      };

      template < class Derived, class Traits >
         struct cgal_triangulation_base
      {
         typedef typename Traits::Triangulation                   triangulation;
         typedef typename Traits::scalar_type                     scalar_type;
         typedef cg::point_t< scalar_type, 2 >                    cg_point_2;
         typedef cg::point_t< scalar_type, 3 >                    cg_point_3;
         typedef cg::segment_t< scalar_type, 2 >                  cg_segment_2;
         typedef cg::segment_t< scalar_type, 3 >                  cg_segment_3;
         typedef cg::triangle_t< scalar_type, 2 >                 cg_triangle_2;
         typedef cg::triangle_t< scalar_type, 3 >                 cg_triangle_3;

         typedef typename triangulation::Point                    point;
         typedef typename triangulation::Vertex_handle            vertex_handle;
         typedef typename triangulation::Face_handle              face_handle;
         typedef typename triangulation::Finite_vertices_iterator vertices_iterator;
         typedef typename triangulation::Finite_faces_iterator    faces_iterator;
         typedef typename triangulation::Finite_edges_iterator    edges_iterator;
         typedef typename triangulation::Face_circulator          face_circulator;
         typedef typename triangulation::Face                     face;
         typedef typename triangulation::Edge_circulator          edge_circulator;
         typedef typename triangulation::Edge                     edge;
         typedef typename triangulation::Vertex_circulator        vertex_circulator;

         typedef std::list< face_handle >                         list_faces;
         typedef std::list< edge >                                list_edges;

         template < class Point, class OutIter >
            std::pair< vertex_handle, vertex_handle > insert( Point const & p, Point const & q, OutIter inserted_vertices )
         {
            vertex_handle a = self().insert( p );
            vertex_handle b = self().insert( q );

            insert( a, b, inserted_vertices );

            return std::make_pair( a, b );
         }

         template < class Point >
         std::pair< vertex_handle, vertex_handle > insert( Point const & p, Point const & q )
         {
            return insert( p, q, util::null_iterator() );
         }

         bool find_intersected_faces( vertex_handle vaa, vertex_handle vbb,
                                      list_faces & intersected_faces,
                                      list_edges & list_ab, 
                                      list_edges & list_ba,
                                      vertex_handle & vi )
         {
            return cdt().find_intersected_faces( vaa, vbb, intersected_faces, list_ab, list_ba, vi );
         }

         void insert( vertex_handle p, vertex_handle q )
         {
            insert( p, q, util::null_iterator() );
         }

#pragma warning(push)
#pragma warning(disable: 4996)

         template < class OutIter >
         void insert( vertex_handle p, vertex_handle q, OutIter inserted_vertices )
         {            
            cdt().insert( p, q );
            std::copy( cdt().inserted_vertices().begin(), cdt().inserted_vertices().end(), inserted_vertices);            
         }

#pragma warning(pop)

         int               vertices_num()    const { return cdt().number_of_vertices( ); }
         vertices_iterator vertices_begin()  const { return cdt().finite_vertices_begin(); }
         vertices_iterator vertices_end()    const { return cdt().finite_vertices_end(); }

         int               faces_num()       const { return cdt().number_of_faces(); }
         faces_iterator    faces_begin()     const { return cdt().finite_faces_begin(); }
         faces_iterator    faces_end()       const { return cdt().finite_faces_end(); }

         size_t            edges_num()       const { return cdt().number_of_edges(); }
         edges_iterator    edges_begin()     const { return cdt().finite_edges_begin(); }
         edges_iterator    edges_end()       const { return cdt().finite_edges_end(); }

         face_circulator   incident_faces   ( vertex_handle h )  const { return cdt().incident_faces   ( h ); }
         edge_circulator   incident_edges   ( vertex_handle h )  const { return cdt().incident_edges   ( h ); }
         vertex_circulator incident_vertices( vertex_handle h )  const { return cdt().incident_vertices( h ); }

         bool is_vertex( vertex_handle v ) const 
         { 
            return cdt().tds().is_vertex( v );
         }

         bool has_edge( vertex_handle p, vertex_handle q ) const 
         {
            return cdt().is_edge( p, q );
         }

         vertex_handle infinite_vertex() const
         {
            return cdt().infinite_vertex();
         }

         bool is_valid() 
         {
            return cdt().is_valid();
         }

         void clear()
         {
            cdt() = triangulation();
         }

         void remove_constraint( vertex_handle p, vertex_handle q ) 
         {
            cdt().remove_constraint( p, q );
         }

         void remove_incident_constraints( vertex_handle v )
         {
            cdt().remove_incident_constraints(v);
         }

         void remove_constraints()
         {
            cdt().remove_constraints();
         }

         bool has_incident_constraints( vertex_handle v ) const
         {
              return cdt().are_there_incident_constraints( v );
         }

         template < class T >
            bool is_infinite( T t ) const
         {
            return cdt().is_infinite( t );
         }

         bool is_constrained( vertex_handle p, vertex_handle q ) const
         {
            face_handle f;
            int i;

            return cdt().is_edge( p, q, f, i ) ? is_constrained( std::make_pair( f, i ) ) : false;
         }

         template < class T >
            bool is_constrained( T t ) const
         {
            return cdt().is_constrained( t );
         }

         void flip( face_handle f, int i )
         {
            cdt().flip( f, i );
         }

         face_handle locate( cg_point_2 const & pt ) const
         {
            triangulation::Locate_type lt; 
            int li;

            return cdt().locate( point( pt.x, pt.y ), lt, li );
         }

         vertex_handle get_vertex( cg_point_2 const & pt )
         {
            triangulation::Locate_type lt; 
            int li;

            face_handle f = cdt().locate( point( pt.x, pt.y ), lt, li );
            if ( lt == triangulation::VERTEX )
               return f != NULL ? f->vertex( li ) : self().insert( pt );
            else 
               return NULL;
         }
         

      protected:
         ~cgal_triangulation_base() {}

         triangulation &         cdt()       { return cdt_; }
         triangulation const &   cdt() const { return cdt_; }

      private:
         Derived &         self()         { return static_cast< Derived & >( *this ); }
         Derived const &   self() const   { return static_cast< Derived const & >( *this ); }

         triangulation cdt_;
      };

      template < typename Info_, typename GT, typename Vb = CGAL::Triangulation_vertex_base_with_info_2<Info_, GT> >
         struct vertex_type
           : public Vb
      {
        typedef typename Vb::Face_handle                   Face_handle;
        typedef typename Vb::Point                         Point;

        template < typename TDS2 >
        struct Rebind_TDS {
          typedef typename Vb::template Rebind_TDS<TDS2>::Other   Vb2;
          typedef vertex_type<Info_, GT, Vb2>                     Other;
        };

        vertex_type() : Vb(), z(-10000 ), index( -1 ) {}
        vertex_type(const Point & p) : Vb(p), z(-10000 ), index( -1 ) {}
        vertex_type(const Point & p, Face_handle c) : Vb(p, c), z(-10000 ), index( -1 ) {}
        vertex_type(Face_handle c) : Vb(c), z(-10000 ), index( -1 ) {}

        double z;
        int    index;
      };

      template < class Traits >
         struct traits_base
      {
         typedef typename Traits::scalar_type      scalar_type;
         typedef typename Traits::vertex_info_type vertex_info_type;
         typedef typename Traits::face_info_type   face_info_type;

         typedef typename CGAL::Filtered_kernel< CGAL::Simple_cartesian< scalar_type > > K;

         struct Vbb : vertex_type< vertex_info_type, K > {};
         struct Vb  : CGAL::Triangulation_hierarchy_vertex_base_2<Vbb>{};
         struct Fbb : CGAL::Triangulation_face_base_with_info_2< face_info_type, K > {} ;
         struct Fb  : CGAL::Constrained_triangulation_face_base_2< K, Fbb > {};
         struct TDS : CGAL::Triangulation_data_structure_2<Vb,Fb> {};

         typedef cg::point_t< scalar_type, 2 >     cg_point_2;
         typedef cg::point_t< scalar_type, 3 >     cg_point_3;
         typedef cg::segment_t< scalar_type, 2 >   cg_segment_2;
         typedef cg::segment_t< scalar_type, 3 >   cg_segment_3;

         struct K1
            : K
         {
            struct Intersect_2
            {
               typedef typename K::Point_2   Point_2;
               typedef typename K::Segment_2 Segment_2;

               cg::point_2 make_point( Point_2 const & p )
               {
                  return cg::point_2( p.x(), p.y() );
               }

               cg::segment_2 make_segment( Segment_2 const & s )
               {
                  return cg::segment_2( make_point( s[0] ), make_point( s[1] ) );
               }

               CGAL::Object operator() (Segment_2 const & p, Segment_2 const & q)
               {
                  CGAL::Object res = CGAL::intersection(p, q);

                  Point_2 pt;
                  if (CGAL::assign(pt, res))
                  {
                     cg::point_2 t(pt.x(), pt.y());

                     cg::rectangle_2 sRect( make_point( p[0] ), make_point( p[1] ) );
                     cg::rectangle_2 tRect( make_point( q[0] ), make_point( q[1] ) );

                     t = (sRect & tRect).closest_point( t );

                     return CGAL::make_object( Point_2( t.x, t.y ) );
                  }
                  else
                     return res;
               }
            };

            Intersect_2 intersect_2_object() const { return Intersect_2(); }
         };
      };

      template < class Traits >
         struct planar_triangulation_traits
            : traits_base< Traits >
      {
         typedef constrained_delaunay_triangulation<K1, TDS> Triangulation;
      };

      template < class Traits >
         struct planar_triangulation_with_height_traits
            : traits_base< Traits >
      {
         typedef cg::triangle_t< scalar_type, 3 > cg_triangle_3;

         typedef
            boost::function< scalar_type ( cg_point_2 const & ) >
            isection_handler_t;

         struct Triangulation
            : constrained_delaunay_triangulation<K1, TDS >
         {
            typedef constrained_delaunay_triangulation<K1, TDS > base;

            void set_intersection_handler( isection_handler_t h )
            {
               isect_handler_ = h;
            }

            virtual Vertex_handle do_insert( Point const & pt, Face_handle f, int i )
            {
               scalar_type z = isect_handler_( cg_point_2( pt.x(), pt.y() ) );
               remove_constraint( f->vertex(cw(i)), f->vertex(ccw(i)) );
               Vertex_handle h = virtual_insert( pt, f );
               h->z = z;
               return h;
            }

            scalar_type interpolate_height( cg_point_2 const & pt, Face_handle start = Face_handle() ) const
            {
               Locate_type lt; 
               int li;

               Face_handle loc = locate( Point( pt.x, pt.y ), lt, li, start );
               scalar_type z = interpolate_height( pt, lt, loc, li );

               Verify( _finite( z ) );
               return z;
            }

            cg_point_3 construct( Vertex_handle const & pt ) const
            {
               return cg_point_3( pt->point().x(), pt->point().y(), pt->z );
            }

            bool interpolate_in_segment( cg_point_3 a, cg_point_3 b, cg_point_2 const & pt, scalar_type & z ) const
            {
               cg::sort2( a, b );

               cg_segment_3 edge_3( a, b );
               cg_segment_2 edge_2( a, b );

               if ( eq( edge_2.P0( ), edge_2.P1( ) ) )
               {
                  if ( cg::eq( edge_2.P0(), pt ) )
                  {
                     z = edge_3.P0( ).z;
                     return true;
                  }

                  return false;
               }

               scalar_type t = edge_2( pt );
               if ( !cg::range_2( 0, 1 ).contains( t ) )
                  return false;

               z = edge_3( t ).z;
               return true;
            }

            scalar_type interpolate_height_in_face( cg_point_2 const & pt, Face_handle f ) const
            {
               cg_point_3 v0 = construct( f->vertex(0) );
               cg_point_3 v1 = construct( f->vertex(1) );
               cg_point_3 v2 = construct( f->vertex(2) );

               cg::sort3( v0, v1, v2 );

               cg_triangle_3 tri( v0, v1, v2 );
               cg::triangle_raster_aux tr( tri );
               cg::barycentric_coords bc;

               if ( tr.IsInTriangleBarycentric( pt, bc ) )
                  return cg::barycentric_interpolate_in_triangle( bc, v0.z, v1.z, v2.z );

               scalar_type z;
               AssertRelease( interpolate_in_segment( v0, v1, pt, z ) || 
                              interpolate_in_segment( v1, v2, pt, z ) || 
                              interpolate_in_segment( v2, v0, pt, z ) );
               return z;
            }

            scalar_type interpolate_height( cg_point_2 const & pt, Locate_type lt, Face_handle f, int li ) const
            {
               switch (lt)
               {
               case VERTEX:
                  if ( li == 4 || is_infinite( f->vertex( li ) ) )
                     return 0;
                  return f->vertex(li)->z;

               case EDGE:
               {
                  if ( is_infinite( f ) )
                     return 0;

                  scalar_type z;
                  AssertRelease( interpolate_in_segment( construct( f->vertex( ( li + 1 ) % 3 ) ),
                                                         construct( f->vertex( ( li + 2 ) % 3 ) ), pt, z ) );
                  return z;
               }

               case FACE:
                  if ( is_infinite( f ) )
                     return 0;

                  return interpolate_height_in_face( pt, f );

               case OUTSIDE_AFFINE_HULL:
               case OUTSIDE_CONVEX_HULL:
                  return 0;

               default:
                  Assert(0);
               }

               return 0;
            }

         private:
            isection_handler_t isect_handler_;
         };
      };
   }

   template < class Scalar = double, class VertexInfo = cg::Empty, class FaceInfo = cg::Empty >
      struct cgal_triangulation_traits
   {
      typedef Scalar       scalar_type;
      typedef VertexInfo   vertex_info_type;
      typedef FaceInfo     face_info_type;
   };

   template < class Traits = cgal_triangulation_traits<>, class TriangulationTraits = details::planar_triangulation_traits< Traits > >
      struct cgal_triangulation
         : details::cgal_triangulation_base< cgal_triangulation< Traits, TriangulationTraits >, TriangulationTraits >
         , boost::noncopyable
   {      
      typedef
         details::cgal_triangulation_base< cgal_triangulation< Traits, TriangulationTraits >, TriangulationTraits >
         base;

      typedef
         cg_point_2
         vertex_type;

      vertex_handle insert( vertex_type const & pt )
      {
         return cdt().insert( point( pt.x, pt.y ) );
      }

      void delete_vertex( vertex_handle v )
      {
         cdt().remove( v );
      }

      void delete_face( face_handle f )
      {
         cdt().delete_face( f );
      }

      using base::insert;
      using base::find_intersected_faces;

      vertex_type construct( vertex_handle v ) const
      {
         return vertex_type( v->point().x(), v->point().y() );
      }

      cg_segment_2 construct( edge const & v ) const
      {
         return cg_segment_2( construct( v.first->vertex( v.first->ccw( v.second ) ) ),
                              construct( v.first->vertex( v.first->cw( v.second ) ) ) );
      }

      cg_triangle_2 construct( face_handle f ) const
      {
         return cg_triangle_2( construct( f->vertex( 0 ) ),
                               construct( f->vertex( 1 ) ),
                               construct( f->vertex( 2 ) ) );
      }
   };

   template < class Traits = cgal_triangulation_traits<> >
      struct cgal_triangulation_with_height
         : details::cgal_triangulation_base< cgal_triangulation_with_height< Traits >,
                                             details::planar_triangulation_with_height_traits< Traits > >
         , boost::noncopyable
   {
      typedef 
         details::planar_triangulation_with_height_traits< Traits >
         trg_traits_t;

      typedef 
         details::cgal_triangulation_base< cgal_triangulation_with_height, trg_traits_t >
         base;

      typedef
         cg_point_3 
         vertex_type;

      cgal_triangulation_with_height()
      {
         cdt().set_intersection_handler( boost::bind( &cgal_triangulation_with_height::height, this, _1 ) );
      }

      void clear()
      {
         cgal_triangulation_base::clear();
         cdt().set_intersection_handler( boost::bind( &cgal_triangulation_with_height::height, this, _1 ) );
      }

      virtual ~cgal_triangulation_with_height() {}

      virtual scalar_type height( cg_point_2 const & pt )
      {
         return cdt().interpolate_height( pt );
      }

      vertex_handle insert( vertex_type const & pt )
      {
         vertex_handle h = cdt().insert( point( pt.x, pt.y ) );
         h->z = pt.z;
         return h;
      }

      using base::insert;

      vertex_type construct( vertex_handle const & pt ) const
      {
         return vertex_type( pt->point().x(), pt->point().y(), pt->z );
      }

      cg::triangle_3 construct( face_handle f ) const
      {
         return cg::triangle_3( construct( f->vertex( 0 ) ),
                                construct( f->vertex( 1 ) ),
                                construct( f->vertex( 2 ) ) );
      }

      scalar_type interpolate_height( cg_point_2 const & pt ) const
      {
         return cdt().interpolate_height( pt );
      }

      std::vector< cg_point_2 > vsection( cg_point_2 const & p, cg_point_2 const & q ) const
      {
         typedef typename triangulation::Line_face_circulator  iterator;
         typedef typename triangulation::Edge                  edge_type;

         std::vector< cg_point_2 > res( 1, cg_point_2( 0, interpolate_height( p ) ) );

         cg_segment_2 seg( p, q );

         point const pp( p.x, p.y );
         point const qq( q.x, q.y );

         for ( iterator c = cdt().line_walk( pp, qq, locate( p ) );; ++c ) {
            face_handle f = c;
            int last = res.size();

            for ( size_t i = 0; i != 3; ++i )
            {
               if ( edge_type( f, i ) < edge_type( f->neighbor( i ), f->mirror_index( i ) ) ) 
               {
                  cg_segment_3 edge( construct( f->vertex( f->cw( i ) ) ),
                                      construct( f->vertex( f->ccw( i ) ) ) );
                  cg_segment_2 edge_2d( edge.P0(), edge.P1() );

                  cg_point_2 isection;
                  if ( cg::generic_intersection( seg, edge_2d, &isection, &isection ) )
                     res.push_back( cg_point_2( seg( isection ), edge( edge_2d( isection ) ).z ) );
               }
            }

            std::sort( res.begin() + last, res.end() );

            if ( cdt().oriented_side( f, qq ) != CGAL::ON_NEGATIVE_SIDE )
               break;
         }

         res.push_back( cg_point_2( 1, interpolate_height( q ) ) );

         return res;
      }

      template < class Traits >
         friend Mesh create_mesh( cgal_triangulation_with_height< Traits > const & trg );
   };

   
   namespace details
   {
      template < class Triangulation >
         void indexate_vertices( Triangulation & trg )
      {
         size_t idx = 0;
         for ( typename Triangulation::vertices_iterator it = trg.vertices_begin(); it != trg.vertices_end(); ++it )
            it->index = idx++;
      }

      template < class Triangulation >
         void indexate_vertices( Triangulation & trg, std::vector< size_t > const &indices )
      {
         size_t idx = 0;
         for ( typename Triangulation::vertices_iterator it = trg.vertices_begin(); it != trg.vertices_end(); ++it )
            it->index = indices[idx++];
      }
   }

   template < class Triangulation, class Point >
       void get_vertices( Triangulation & trg, std::vector< Point > & vertices )
   {
      vertices.reserve( trg.vertices_num() );
      for ( typename Triangulation::vertices_iterator it = trg.vertices_begin(); it != trg.vertices_end(); ++it )
         vertices.push_back( trg.construct( it ) );
   }

   template < class Triangulation, class FaceFilter >
       void get_faces( Triangulation & trg, FaceFilter face_filter, std::vector< cg::point_3i > & faces )
   {
      faces.reserve( trg.faces_num() );
      for ( typename Triangulation::faces_iterator it = trg.faces_begin(); it != trg.faces_end(); ++it )
      {
         if ( face_filter( it ) )
         {
            cg::point_3i f;

            for ( int i = 0; i < 3; ++i )
               f[i] = it->vertex(i)->index;

            faces.push_back( f );
         }
      }
   }

   template < class Triangulation, class Point, class FaceFilter >
       void get_data( Triangulation & trg, FaceFilter face_filter, std::vector< Point > & vertices, std::vector< cg::point_3i > & faces )
   {
      details::indexate_vertices( trg );
      get_vertices( trg, vertices );
      get_faces( trg, face_filter, faces );
   }

   template < class Triangulation, class Point >
       void get_data( Triangulation & trg, std::vector< Point > & vertices, std::vector< cg::point_3i > & faces )
   {
      struct dumb_filter { template < class T > bool operator()( T const & t ) { return true; } };
      get_data( trg, dumb_filter(), vertices, faces );
   }

                                                                        

   template < class Triangulation >
   std::vector< typename Triangulation::vertex_handle > find_unconstrained_vertices( Triangulation const & trg )
   {
      typedef Triangulation trg_t;
      std::vector< typename trg_t::vertex_handle > res;

      for ( typename trg_t::vertices_iterator it = trg.vertices_begin(); it != trg.vertices_end(); ++it )
         if ( !trg.has_incident_constraints( it ) )
            res.push_back( it );

      return res;
   }

   template < class Triangulation, class OutIter >
   void remove_unconstrained_points( Triangulation & trg, OutIter out )
   {
      typedef Triangulation trg_t;
      std::vector< typename trg_t::vertex_handle > const & vertices = find_unconstrained_vertices( trg );

      for ( size_t i = 0; i != vertices.size(); ++i )
      {           
         *out++ = trg.construct( vertices[i] );
         trg.delete_vertex( vertices[i] );
      }
   }

   template < class Triangulation >
   void remove_unconstrained_points( Triangulation & trg )
   {
      remove_unconstrained_points( trg, util::null_iterator() );
   }

   template< class Triangulation >
   size_t constraints_num( Triangulation const & trg )
   {      
      return std::count_if( trg.edges_begin(), trg.edges_end(), 
         boost::bind( &Triangulation::is_constrained< typename Triangulation::edge >, &trg, _1 ) );
   }

   //namespace details
   //{
   //   template < class Triangulation, class Segment >
   //      void get_segments( Triangulation & trg, typename Triangulation::face_handle f, int i, 
   //                         Segment & edge, Segment & new_edge )
   //   {
   //      typedef typename Triangulation::vertex_handle   vertex_handle;
   //      typedef typename Triangulation::face_handle     face_handle;

   //      vertex_handle const v1 = f->vertex( ( i + 1 ) % 3 );
   //      vertex_handle const v2 = f->vertex( ( i + 2 ) % 3 );

   //      edge = Segment( trg.construct( v1 ), trg.construct( v2 ) );

   //      face_handle const n = f->neighbor( i );
   //      
   //      for ( int j = 0; j < 3; ++j )
   //         if ( n->vertex( j ) != v1 && n->vertex( j ) != v2 )
   //         {
   //            new_edge = Segment( trg.construct( f->vertex( i ) ), trg.construct( n->vertex( j ) ) );
   //            break;
   //         }
   //   }

   //   template < class Triangulation >
   //      bool need_flip( Triangulation & trg, typename Triangulation::face_handle f, int i )
   //   {
   //      cg::segment_3 s1, s2;
   //      get_segments( trg, f, i, s1, s2 );

   //      double const d1 = abs( s1.P0().z - s1.P1().z );
   //      double const d2 = abs( s2.P0().z - s2.P1().z );
   //      return d1 < 1 && d2 > d1;
   //   }

   //   inline bool intersect_inside( cg::segment_2 const & s1, cg::segment_2 const & s2 )
   //   {
   //      cg::point_2 pt;
   //      if ( cg::generic_intersection< point_2 >( s1, s2, &pt, NULL ) == cg::disjoint )
   //         return false;

   //      double const t1 = s1( pt ), t2 = s2( pt );
   //      cg::range_2 const rng( 1e-2, 1 - 1e-2 );

   //      return rng.contains( t1 ) && rng.contains( t2 );
   //   }

   //   template < class Triangulation >
   //      bool can_flip( Triangulation & trg, typename Triangulation::face_handle f, int i )
   //   {
   //      typename Triangulation::face_handle n = f->neighbor( i ); 
   //      if ( trg.is_infinite( f ) || trg.is_infinite( n ) || f->is_constrained( i ) )
   //         return false; 

   //      cg::segment_2 s1, s2;
   //      get_segments( trg, f, i, s1, s2 );

   //      return intersect_inside( s1, s2 );
   //   }
   //}

   //template < class Triangulation >
   //   void try_flip_edges( Triangulation & trg )
   //{
   //   typedef typename Triangulation::face_handle face_handle;

   //   std::queue< face_handle > faces;
   //   for ( typename Triangulation::faces_iterator it = trg.faces_begin(); it != trg.faces_end(); ++it )
   //      faces.push( face_handle( it ) );

   //   while ( !faces.empty() )
   //   {
   //      face_handle f = faces.front(); faces.pop();

   //      for ( int i = 0; i < 3; ++i )
   //         if ( details::can_flip( trg, f, i ) && details::need_flip( trg, f, i ) && !f->is_constrained( i ) )
   //         {
   //            faces.push( f );
   //            faces.push( f->neighbor( i ) );

   //            trg.flip( f, i );
   //         }
   //   }
   //}

}}

#include "Geometry/Triangulation/cgal_triangulation_adders.h"
#include "Geometry/Triangulation/cgal_triangulation_simplification.h"