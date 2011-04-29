#pragma once

namespace cg        {
namespace contours  {

namespace shootraying
{
   struct SimpleContourSearchingProcessor
   {
      typedef AccumulatingEquiSegs<EmptyFilter,segment_id>   RayProcessor;

      RayProcessor & rayProcessor() { return rayProcessor_; }

      template <class Traits>
         typename Traits::contour_id 
         result (Traits const & traits, segment_2 const & ray, 
         typename Traits::contour_id defaultId) const
      {
         // выбрали отрезок, с которым пересеклись
         segment_id sid = selectFirstSegment(traits, ray);

         // в зависимости от того, с какой стороны в него воткнулись,
         // выбрали или номер контура или ON_SEA
         // а если не воткнулись, вернули defaultId
         return getContourBySegment(traits, sid, defaultId, ray.P0());
      }

      template <class Traits>
         bool result(
         Traits    const /*[in]*/ & traits, 
         segment_2 const /*[in]*/ & ray, 
         typename Traits::contour_id /*[in,out]*/ & cid,
         typename Traits::segment_id /*   [out]*/ & sid, 
         double /*[in]*/ & ratio)
      {
         if (!rayProcessor_.was_intersection())
            return false;

         // very ugly...
         ratio = ray(rayProcessor_.intersectionPoint());

         sid = selectFirstSegment(traits, ray);

         cid = getContourBySegment(traits, sid, cid, ray.P0());

         return true;
      }



   private:

      template <class Traits>
         typename Traits::segment_id 
         selectFirstSegment(Traits const & traits, segment_2 const & ray) const
      {
         return 
            selectFirstSegmentIntersected(
            traits, 
            rayProcessor_.segments_begin(),
            rayProcessor_.segments_end(),
            ray, rayProcessor_.intersectionPoint(),
            traits.defaultSegmentId()
            );
      }

      template <class Traits>
         typename Traits::contour_id 
         getContourBySegment(Traits const &traits, 
         typename Traits::segment_id sid, 
         typename Traits::contour_id defaultId,
         point_2 const & rayStart) const
      {
         return sid != traits.defaultSegmentId()
            ?   traits.getContourBySegment(sid)
            :   defaultId;
      }


   private:
      RayProcessor   rayProcessor_;
   };
}

namespace algos     {


    template <class Contours, class CDT>
        typename Contours::contour_id 
            simpleShootRay(Contours const & c, CDT const &cdt, segment_2 const & s, typename Contours::contour_id defaultId)
    {
        shootraying::SimpleContourSearchingProcessor proc;

        cdt.genericShootRay(s, proc.rayProcessor());

        return proc.result(c, s, defaultId);
    }

}}}
