#pragma once

namespace cg
{

template< class Scalar, size_t Dim >
   struct bilinear_mapper
{
   typedef
      cg::point_t< Scalar, Dim >
      point_type;

   bilinear_mapper ( point_type const &p00, point_type const &p10,
                     point_type const &p11, point_type const &p01 )
   {
      for (size_t d = 0; d < Dim; ++d)
      {
         conv[d].x = p00[d] - p10[d] - p01[d] + p11[d];
         conv[d].y = -p00[d] + p10[d];
         conv[d].z = -p00[d] + p01[d];
         conv[d].w = p00[d];
      }
   }

   point_type operator () ( cg::point_2 const &uv ) const
   {
      cg::point_4 inp (uv.x * uv.y, uv.x, uv.y, 1);

      point_type res;
      for (size_t d = 0; d < Dim; ++d)
         res[d] = inp * conv[d];

      return res;
   }

private:
   cg::point_4 conv[Dim];
};

typedef bilinear_mapper< double, 2 > bilinear_mapper_2;
typedef bilinear_mapper< double, 3 > bilinear_mapper_3;

} // End of 'cg' namespace
