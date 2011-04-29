#pragma once

#include <common\make_str.h>
#include "geometry/primitives/point.h"
#include "Geometry/array_1d.h"

namespace cg {
namespace verification {

   enum Verificator
   {
      VF_NONE = 0,
      VF_RECURRING_POINTS = 1,
      VF_SELF_INTERSECTION = 2,
      VF_EDGE_OVERLAP_INTERNAL = 8,
      VF_EDGE_OVERLAP_EXTERNAL = 16,
      VF_EDGE_OVERLAP = VF_EDGE_OVERLAP_INTERNAL | VF_EDGE_OVERLAP_EXTERNAL,
      VF_NESTED_ORIENTATION = 32
   };

#pragma pack (push, 1)

   struct verification_result
   {
      verification_result ()
      {
      }

      verification_result ( Verificator result, size_t cntA, size_t cntB, cg::point_2 const &ref )
         : result_ (result)
         , cntA_ (cntA)
         , cntB_ (cntB)
         , ref_ (ref)
      {
      }

      Verificator verificator() const { return result_; }

      size_t contourA() const { return cntA_; }
      size_t contourB() const { return cntB_; }

      void setContourA( size_t cntA ) { cntA_ = cntA; }
      void setContourB( size_t cntB ) { cntB_ = cntB; }

      cg::point_2 const &refPoint() const { return ref_; }

      void setRefPoint( cg::point_2 const &p ) { ref_ = p; }

   private:
      Verificator result_;
      size_t cntA_, cntB_;

      cg::point_2 ref_;
   };

   struct verification_result_objects : public verification_result
   {
      verification_result_objects ()
      {
      }

      verification_result_objects ( Verificator result,
                                    size_t objA, size_t cntA,
                                    size_t objB, size_t cntB, cg::point_2 const &ref )
         : verification_result (result, cntA, cntB, ref)
         , objA_ (objA)
         , objB_ (objB)
      {
      }
                                     
      size_t objectA() const { return objA_; }
      size_t objectB() const { return objB_; }

      void setObjectA( size_t objA ) { objA_ = objA; }
      void setObjectB( size_t objB ) { objB_ = objB; }
      
   private:
      size_t objA_, objB_;
   };

   struct invalid_input_contours : std::exception, virtual boost::exception
   {
      explicit invalid_input_contours ( char const * const algo_name )
         : algorithm (algo_name)
      {
      }

      const char * what () const;

      char const *algorithm;
      cg::array_1d< verification_result > errors;      
   };

   template< class Stream >
      Stream & operator << ( Stream &stream, invalid_input_contours const &e )
   {
      stream << "Input data exception in '";/// << e.algorithm << "' algorithm:" << std::endl;
      for (size_t i = 0; i < e.errors.size(); ++i)
      {
         if (e.errors[i].verificator() & VF_RECURRING_POINTS)
         {
            stream << "recurring points found in contour #" << e.errors[i].contourA() <<
               ", reference point: (" << e.errors[i].refPoint().x << ", " <<
               e.errors[i].refPoint().y << ')' << std::endl;
         }
         else if (e.errors[i].verificator() & VF_SELF_INTERSECTION)
         {
            if (e.errors[i].verificator() & VF_EDGE_OVERLAP)
               stream << "intersection or edge overlap between contours #";
            else
               stream << "intersection between contours #";

            stream << e.errors[i].contourA() << " & #" << e.errors[i].contourB() << " found" <<
               ", reference point: (" << e.errors[i].refPoint().x << ", " <<
               e.errors[i].refPoint().y << ')' << std::endl;
         }
         else if (e.errors[i].verificator() & VF_NESTED_ORIENTATION)
         {
            stream << "contour #" << e.errors[i].contourA() << " has incorrect orientation" << std::endl;
         }
      }
      return stream;
   }

   inline const char * invalid_input_contours::what() const 
   {
      static std::string s(util::make_str() << *this);
      return s.c_str(); 
   }

   struct invalid_input_contour_objects : std::exception, virtual boost::exception
   {
      explicit invalid_input_contour_objects ( char const * const algo_name )
         : algorithm (algo_name)
      {
      }

      const char * what() const 
      {
         static std::string s = util::make_str() << *this;
         return s.c_str();
      }

      char const *algorithm;
      cg::array_1d< verification_result_objects > errors;

      template< class Stream >
         friend Stream & operator << ( Stream &stream, invalid_input_contour_objects const &e )
      {
         stream << "Input data exception in '" << e.algorithm << "' algorithm:" << std::endl;
         for (size_t i = 0; i < e.errors.size(); ++i)
         {
            if (e.errors[i].verificator() & VF_RECURRING_POINTS)
            {
               stream << "recurring points found in (object, contour) #(" <<
                  e.errors[i].objectA() << ", " << e.errors[i].contourA() << ")" <<
                  ", reference point: (" << e.errors[i].refPoint().x << ", " <<
                  e.errors[i].refPoint().y << ')' << std::endl;
            }
            else if (e.errors[i].verificator() & VF_SELF_INTERSECTION)
            {
               if (e.errors[i].verificator() & VF_EDGE_OVERLAP)
                  stream << "intersection or edge overlap between (object, contour) #(";
               else
                  stream << "intersection between (object, contour) #(";
               stream << e.errors[i].objectA() << ", " << e.errors[i].contourA() << ")" << " & #(" <<
                  e.errors[i].objectB() << ", " << e.errors[i].contourB() << ")" << " found" <<
                  ", reference point: (" << e.errors[i].refPoint().x << ", " <<
                  e.errors[i].refPoint().y << ')' << std::endl;
            }
            else if (e.errors[i].verificator() & VF_NESTED_ORIENTATION)
            {
               stream << "(object, contour) #" << e.errors[i].objectA() << ", " <<
                  e.errors[i].contourA() << ") has incorrect orientation" << std::endl;
            }
         }
         return stream;
      }
   };
   
#pragma pack(pop)

}}