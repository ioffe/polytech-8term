#pragma once

namespace cg
{
   enum frustum_type
   {
      FR_perspective = 0,
      FR_orthographic
   };

   template< typename S > class frustum_t;
   template< typename S > class frustum_perspective_t;
   template< typename S > class frustum_orthographic_t;


   typedef frustum_t<float>  frustum_f;
   typedef frustum_t<double> frustum;

   typedef frustum_perspective_t<float>  frustum_perspective_f;
   typedef frustum_perspective_t<double> frustum_perspective;

   typedef frustum_orthographic_t<float>  frustum_orthographic_f;
   typedef frustum_orthographic_t<double> frustum_orthographic;
}
