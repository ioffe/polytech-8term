#pragma  once

namespace cg
{
   namespace rect_rasterization
   {
      template< class transform_type = aa_transform >
         struct rect_rasterization_type
      {
         rect_rasterization_type( transform_type const& tform )
            : tform_( tform )
         {
         }

         template < class processor_type >
            bool process( rectangle_2 const& rc, processor_type & proc ) const
         {
            point_2 beg = tform_.world2local(rc.lo());
            point_2 end = tform_.world2local(rc.hi());

            point_2i start = floor(beg);
            point_2i stop  = floor(end);

            point_2i idx;

            for (idx.x = start.x; idx.x <= stop.x; ++idx.x) 
            {
               for (idx.y = start.y; idx.y <= stop.y; ++idx.y) 
               {
                  if (proc(idx))
                     return true ;
               }
            }

            return false ;
         }

      private:
         transform_type tform_ ;
      };
   }

   template< class transform_type, class processor_type >
   inline bool rasterize_rect( transform_type const& tform, rectangle_2 const& rc, processor_type & proc )
   {
      return rect_rasterization::rect_rasterization_type<transform_type>(tform).process(rc, proc) ;
   }
}
