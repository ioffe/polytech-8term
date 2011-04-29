#pragma once

#include <vector>


namespace util
{
   namespace detail
   {
      template<typename index_type, typename vertex_type>
      struct vertex_info
      {
         vertex_type vertex;
         bool added;
         index_type index;

         vertex_info( vertex_type const& vertex )
            : vertex(vertex)
            , added(false)
            , index(0)
         {
         }
      };

      template<typename index_type, typename vertex_type, typename index_iterator, typename vertex_iterator>
      inline index_type reorder_vertices( index_iterator begin_index, index_iterator end_index, vertex_iterator begin_vertex, vertex_iterator end_vertex )
      {
         typedef typename vertex_info<index_type,vertex_type> vertex_info; 
         std::vector<vertex_info> vertices_info(begin_vertex, end_vertex);

         index_type vertices_written = 0;
         for (index_iterator index_iter = begin_index; index_iter != end_index; ++index_iter)
         {
            index_type & index = *index_iter;
            vertex_info & vert_info = vertices_info[index];
            if (!vert_info.added)
            {
               vert_info.index = vertices_written;
               begin_vertex[vertices_written] = vert_info.vertex;
               vert_info.added = true;
               vertices_written++;
            }
            index = vert_info.index;
         }

         return vertices_written;
      }
   }


   //
   // reorder vertices for more linear access while rendering
   //

   template<typename index_iterator, typename vertex_iterator>
   inline unsigned reorder_vertices( index_iterator begin_index, index_iterator end_index, vertex_iterator begin_vertex, vertex_iterator end_vertex )
   {
      Assert(end_vertex - begin_vertex < std::numeric_limits<index_iterator::value_type>::max());
      return detail::reorder_vertices<index_iterator::value_type,vertex_iterator::value_type>(begin_index, end_index, begin_vertex, end_vertex);
   }

   template<typename index_type, typename vertex_type>
   inline unsigned reorder_vertices( index_type * indices, unsigned indices_count, vertex_type * vertices, unsigned vertices_count )
   {
      Assert(vertices_count < std::numeric_limits<index_type>::max());
      return detail::reorder_vertices<index_type,vertex_type>(indices, indices + indices_count, vertices, vertices + vertices_count);
   }


   //
   // raw data version (for cases when vertex_type is unknown)
   //

   template<typename index_type>
   inline unsigned reorder_vertices( index_type * indices, unsigned indices_count, void * vertices, unsigned vertices_count, unsigned vertex_size )
   {
      Assert(vertices_count < std::numeric_limits<index_type>::max());
      unsigned char * vertices_data = reinterpret_cast<unsigned char *>(vertices);
      unsigned const vertices_data_size = vertices_count * vertex_size;
      std::vector<unsigned char> vertices_data_copy(vertices_data, vertices_data + vertices_data_size);
      std::vector<bool> vertex_added(vertices_count);
      std::vector<index_type> vertex_index(vertices_count);

      unsigned vertices_written = 0;
      for (unsigned i = 0; i < indices_count; i++)
      {
         index_type & index = indices[i];
         if (!vertex_added[index])
         {
            vertex_index[index] = vertices_written;
            memcpy(&vertices_data[vertices_written * vertex_size], &vertices_data_copy[index * vertex_size], vertex_size);
            vertex_added[index] = true;
            vertices_written++;
         }
         index = vertex_index[index];
      }

      return vertices_written;
   }
}
