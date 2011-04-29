#pragma once 

//#include "cgal_triangulation.h"
//#include "Rendering/MeshLinker.h"
//
//namespace cg            {
//namespace triangulation {
//
//   template < class Triangulation, class VertexConstructor >
//      Mesh create_mesh( Triangulation const & trg, VertexConstructor c )
//   {
//      Mesh mesh;
//
//      Mesh::VertexArray va;
//      Mesh::IndexArray ia;
//      Mesh::NormalArray na;   
//
//      va.reserve( trg.number_of_vertices() );
//      ia.reserve( 3 * trg.number_of_faces() );
//
//      typedef typename Triangulation::Vertex_handle         vertex_handle;
//      typedef typename Triangulation::Face_handle           face_handle;
//      typedef typename Triangulation::Finite_faces_iterator faces_iterator;
//
//      typedef std::map< vertex_handle, int > indices_type;
//      indices_type indices;
//      int cur_index = 0;
//
//      std::vector< cg::point_3 > normals( trg.number_of_vertices() );
//
//      for ( faces_iterator it = trg.finite_faces_begin(); it != trg.finite_faces_end(); ++it )
//      {
//         cg::point_3 const pts[] = { c( it->vertex( 0 ) ), c( it->vertex( 1 ) ), c( it->vertex( 2 ) ) };
//         cg::point_3 const & n = cg::normal( cg::triangle_3( pts[0], pts[1], pts[2] ) ); 
//
//         for ( int i = 0; i < 3; ++i )
//         {
//            int index = 0;
//            indices_type::iterator j = indices.find( it->vertex( i ) );
//            if ( j == indices.end() )
//            {
//               index = cur_index;
//               va.push_back( pts[i] );
//               indices.insert( j, std::make_pair( it->vertex( i ), cur_index++ ) );
//            }
//            else
//               index = j->second;
//
//            ia.push_back( index );
//            normals[ index ] += n;
//         }
//      }
//
//      for ( size_t i = 0; i < normals.size(); ++i )
//         na.push_back( cg::point_3f( cg::normalized( normals[i] ) ) );
//
//      mesh.setVertices( va.size(), va.empty() ? NULL : &va[0] );
//      mesh.setNormals( na.size(), na.empty() ? NULL : &na[0] );
//      mesh.setIndices( C_triangles, ia.size(), ia.empty() ? NULL : &ia[0] );
//
//      return mesh;
//   }
//
//   struct default_vertex_constructor
//   {
//      template < class VertexHandle >
//         cg::point_3 operator ()( VertexHandle v ) const { return cg::point_3( v->point().x(), v->point().y(), v->point().z() ); }
//   };
//
//   template < class Triangulation >
//      Mesh create_mesh( Triangulation const & trg )
//   {
//      return create_mesh( trg, default_vertex_constructor() );
//   }
//
//   template < class Traits >
//      Mesh create_mesh( cgal_triangulation_with_height< Traits > const & trg )
//   {
//      return create_mesh( trg.cdt() );
//   }
//
//}}