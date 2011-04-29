#pragma once 
#include "Geometry\Splines\1DSplines.h"

namespace cg 
{ 
   template<class NatSpline >
   struct DefaultParameterConverter
   {
      DefaultParameterConverter( NatSpline const& natSpline )
         : spline_       (natSpline)
      {
      }

      double B2Len   (double b   ) const { return spline_.GetLength   ( b   ) ; }
      double Len2B   (double len ) const { return spline_.GetParameter( len ) ; } 
      double dBBydLen(double len ) const { return spline_.GetDtByDlen ( len ) ; }
      double Length  ()            const { return spline_.Length(); }

   private:
      NatSpline           const& spline_;
   };

//    //////////////////////////////////////////////////////////////////////////
//    // 
//    // 1D spline based on HermiteSplineManager 
//    // 
// 
//    template< class PointInfo, template < class > class GetField, class ParameterConverter >
//    struct Spline1D
//    {
//       typedef 
//          GetField < PointInfo >
//          GetPointField ;
// 
//       typedef 
//          typename GetPointField :: Type 
//          Type ;
// 
//       typedef std::vector<PointInfo>               PointInfoVec;
//       typedef cg::HermiteSplineManager<Type>       HermiteSplineManager1d;
// 
//       Spline1D( ParameterConverter const& converter )
//          : converter_(converter)
//       {}
// 
//       void SetPoints(PointInfoVec const& track)
//       {
//          const double eps = 5e-4 ; 
// 
//          values_.Clear() ; 
// 
//          for(size_t i = 0; i < track.size (); i++)
//             values_.PushBack(GetPointField::get(track[i]));
// 
//          for(size_t i = 1; i < track.size () - 1; i++)
//          {
//             double dLen = converter_.B2Len(i + 1) - converter_.B2Len(i - 1);
//             Type dValdLen  = ( GetPointField::get( track[i+1] ) - GetPointField::get( track[i-1] )) / dLen;
// 
//             double dLendtR = (converter_.B2Len(i + eps) - converter_.B2Len(i)) / eps;
//             double dLendtL = (converter_.B2Len(i) - converter_.B2Len(i - eps)) / eps;
// 
//             values_.SetLeftDirection (i, dValdLen * dLendtL); 
//             values_.SetRightDirection(i, dValdLen * dLendtR); 
//          }
//       }
// 
//       Type Interpolate (double len) const
//       {
//          Type val = values_.Interpolate(converter_.Len2B(len));
// 
//          if(len < 0) 
//             val += Derivative(0) * len;
//          if(len > converter_.Length())
//             val -= Derivative(converter_.Length()) * (converter_.Length() - len);
// 
//          return val;
//       }
// 
//       Type Derivative  (double len) const
//       {
//          return values_.Derivative(converter_.Len2B(len)) * converter_.dBBydLen(len);  
//       }
// 
//    private:
//       HermiteSplineManager1d     values_     ;
//       ParameterConverter const&  converter_  ;
//    };

   template< class PointInfo, template < class > class GetField, class AttrsProvider >
   struct Spline1D
   {
      typedef 
         GetField < PointInfo >
         GetPointField ;

      typedef 
         typename GetPointField :: Type 
         Type ;

      typedef 
         std::vector<PointInfo> 
         PointInfoVec;

      typedef 
         cg::Splines::Spline1D< Type, cg::Splines::Constructors::CatmullRom > 
         SplineCR ;

      Spline1D( ParameterConverter const& converter )
         : converter_(converter)
      {}

      void SetPoints(PointInfoVec const& track)
      {
         const double eps = 5e-4 ; 

         SplineCR::KnotPoints points ( track.size()) ;

         for( size_t i = 0; i < track.size (); i++ )
            points[i] = SplineCR::KnotPoint( converter_.B2Len( i ), GetPointField::get( track[i] )) ; 
            
         spline_.Init( points ) ;
      }

      Type Interpolate (double len) const
      {
         Type val = spline_.Interpolate( len );

         if( len < 0 ) 
            val += spline_.Derivative(0) * len;
         if( len > converter_.Length())
            val -= spline_.Derivative( converter_.Length()) * ( converter_.Length() - len );

         return val;
      }

      Type Derivative  (double len) const
      {
         return spline_.Derivative( len ) ;
      }

   private:
      SplineCR                   spline_     ;
      ParameterConverter const&  converter_  ;
   };
}