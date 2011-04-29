#pragma once

struct beam
{
   cg::segment_2 edge;
   cg::point_2 dirStart;
   cg::point_2 dirEnd;

   beam ()
   {
   }

   beam ( cg::segment_2 const &e, cg::point_2 const &dS, cg::point_2 const &dE )
      : edge (e), dirStart (dS), dirEnd (dE)
   {
   }
};
