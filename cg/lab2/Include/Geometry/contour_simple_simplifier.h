#pragma once

namespace cg
{

// Input: not looped contour
template< class VertexBuffer, class ContourCont >
void simplify_contour( VertexBuffer const &vertBuffer, ContourCont &contour, double prec = 0.1 )
{
   if (contour.size() < 3)
      return;
   
   for (size_t test = 0; test < contour.size();)
   {
      size_t next = (test + 1) % contour.size();
      size_t prev = (test - 1 + contour.size()) % contour.size();

      if (cg::distance(cg::segment_2 (vertBuffer[contour[prev]],
                                      vertBuffer[contour[next]]), vertBuffer[contour[test]]) < prec)
      {
         contour.erase(contour.begin() + test);
      }
      else
      {
         test++;
      }
   }
}

// Input: not looped contour
template< class VertexBuffer, class ContourCont >
void simplify_contour_by_length( VertexBuffer const &vertBuffer, ContourCont &contour, double prec = 0.1 )
{
   if (contour.size() < 2)
      return;
   
   for (size_t test = 0; test < contour.size();)
   {
      size_t next = (test + 1) % contour.size();

      if (cg::distance( vertBuffer[contour[test]], vertBuffer[contour[next]]) < prec)
         contour.erase(contour.begin() + test);
      else
      {
         test++;
      }
   }
}

} // End of 'cg' namespace
