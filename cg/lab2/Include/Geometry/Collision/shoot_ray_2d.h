#pragma once

#pragma pack ( push, 1 )

namespace cg {
namespace cdt
{
   template < class Derived, class Grid, typename contour_id > // SegmentId - Может быть заменен на traits
      struct shoot_ray_2d
   {
      // Требует от Derived :
      //      void shootRayInCell ( smallcell_type, segment_2, ShootRayProcessor) const
      template < class ShootRayProcessor >
            typename ShootRayProcessor::item_id shootRay (segment_2 const &s, typename ShootRayProcessor::item_id defaultId, contour_id invisible_con_id) const
      {
         Assert ( bounding(self().grid()).contains(s) ) ;
         Processor < ShootRayProcessor, contour_id > tracer(self(), s, defaultId, invisible_con_id);

         cg::visit(const_cast<Grid&>(self().grid()), s, tracer);

         return tracer.itemFound();
      }

      template < class ShootRayProcessor >
            typename ShootRayProcessor::item_id shootRay (segment_2 const &s, typename ShootRayProcessor::item_id defaultId) const
      {
         return shootRay<ShootRayProcessor>(s, defaultId, self().defaultContourId());
      }

      // Также сюда можно запихать более информативные версии shootRay

      template< class Stream >
         void serialize( Stream & stream ) const
      {
         write( stream, char(123) ); // _dummy_
      }

   private:

      template < class ShootRayProcessor, typename contour_id >
            struct Processor : cg::grid2l_visitor_base < Grid, Processor<ShootRayProcessor, contour_id> >
      {
         typedef typename ShootRayProcessor::item_id item_id;

         template <class State>
            bool operator () (State const & state, smallcell_type const & scell)
         {
            ShootRayProcessor shoot_ray_processor ( self_, query_seg_, default_id_, invisible_con_id_ );
            segment_2 q (query_seg_(state.in_ratio), query_seg_(state.out_ratio));
            
            self_.shootRayInCell ( scell, q, shoot_ray_processor );
            itemFound_ = shoot_ray_processor.intersection();

            return shoot_ray_processor.was_intersection();
         }

         Processor (Derived const & slf, segment_2 const & s, item_id defaultId, contour_id invisible_con_id) 
            : self_ (slf), default_id_(defaultId), query_seg_(s), invisible_con_id_(invisible_con_id)
         {}

         item_id itemFound() const
         {
            return itemFound_;
         }

      private:
         Derived     const & self_;
         item_id     const   default_id_;
         segment_2   const & query_seg_;

         contour_id  const   invisible_con_id_;

         mutable item_id     itemFound_;
      };
      
   private:
      Derived const & self() const { return static_cast < Derived const & >(*this); }

      // for serialization
      char _dummy_;
   };
}}

#pragma pack ( pop )

