#pragma once

#include <vector>
#include <map>
#include <stack>
#include <string>

#pragma warning (push)
#pragma warning (disable : 4706 4996)

#include <boost\exception.hpp>

#pragma warning (pop)

#include "verification_exceptions.h"
#include "recurring_points.h"
#include "singular_angles.h"
#include "straight_angles.h"

namespace cg
{
namespace verification
{

#define ALGORITHM(namestr) \
   static char const * const algorithm_name() { return namestr; }

#define INPUT_VERIFICATION(code) \
   BOOST_STATIC_ASSERT(((code) & cg::verification::VF_SELF_INTERSECTION) || \
                       !((code) & cg::verification::VF_EDGE_OVERLAP));      \
   static int const verification_code = code;

// Forward declaration
template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_input( VertexBuffer const &vertices,
                     ContoursIterator begin, ContoursIterator end );

// Meta-function retrieves scalar type from VertexBuffer
template< class VertexBuffer >
   struct scalar
{
   typedef typename VertexBuffer::value_type::scalar_type type;
};

// Forward declaration
template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_input( VertexBuffer const &vertices,
                     ContoursIterator begin, ContoursIterator end,
                     typename scalar< VertexBuffer >::type eps );

} // End of 'verification' namespace
} // End of 'cg' namespace

#include "self_intersection.h"
#include "nested_orientation.h"

namespace cg
{
namespace verification
{

// Converts verification result from global contour indices to local (object)
template< class ObjectsIterator >
   verification_result_objects
      convert_verification_result( ObjectsIterator begin, ObjectsIterator end,
                                   verification_result const &res )
{
   if (res.verificator() == VF_NONE)
      return res;

   verification_result_objects newRes (res.verificator());
   size_t contourA = res.contourA();
   size_t contourB = res.contourB();
   for (ObjectsIterator oIt = begin; oIt != end; ++oIt)
   {
      if (contourA == -1 && contourB == -1)
         return newRes;

      size_t objectCntCount = oIt->size();

      if (contourA != -1)
      {
         if (contourA < objectCntCount)
         {
            newRes.setContourA(contourA);
            newRes.setObjectA(oIt - begin);
            contourA = -1;
         }
         else
            contourA -= objectCntCount;
      }

      if (contourB != -1)
      {
         if (contourB < objectCntCount)
         {
            newRes.setContourB(contourB);
            newRes.setObjectB(oIt - begin);
            contourB = -1;
         }
         else
            contourB -= objectCntCount;
      }
   }
}

//
// Basic verification procedures
//

// Verification procedure checks contour input data by verification_code declared in Algorithm
template< class VertexBuffer, class ContoursIterator >
   void check_input_base( int verification_code, VertexBuffer const &vertices,
                          ContoursIterator begin, ContoursIterator end )
{
   check_input_base(verification_code, vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class VertexBuffer, class ContoursIterator >
   void check_input_base( int verification_code, VertexBuffer const &vertices,
                          ContoursIterator begin, ContoursIterator end,
                          typename scalar< VertexBuffer >::type eps )
{
   typedef typename scalar< VertexBuffer >::type scalar_type;
   typedef cg::point_t< scalar_type, 2 > point_type;

   invalid_input_contours result ("Unknown");

   std::vector< verification_result > tmp;

   if (verification_code & VF_RECURRING_POINTS)
   {
      std::vector< std::pair< size_t, point_type > > res;
      check_recurring_points(vertices, begin, end, std::back_inserter(res), eps);

      for (size_t i = 0; i < res.size(); ++i)
         tmp.push_back(verification_result (VF_RECURRING_POINTS, res[i].first, static_cast< size_t >( -1 ), res[i].second));
   }

   if (verification_code & VF_SELF_INTERSECTION)
   {
      int edgeOverlapBit = verification_code & VF_EDGE_OVERLAP;
      int otype = OT_NONE;

      if (edgeOverlapBit != 0)
      {
         if (edgeOverlapBit & VF_EDGE_OVERLAP_EXTERNAL)
            otype |= OT_EXTERNAL;
         if (edgeOverlapBit & VF_EDGE_OVERLAP_INTERNAL)
            otype |= OT_INTERNAL;
      }

      std::vector< contours_intersection > res;
      check_self_intersection(vertices, begin, end, std::back_inserter(res), otype, eps);

      for (size_t i = 0; i < res.size(); ++i)
         tmp.push_back(verification_result (Verificator (VF_SELF_INTERSECTION | edgeOverlapBit),
                                                      res[i].cntA, res[i].cntB, res[i].refPoint));
   }

   if (verification_code & VF_NESTED_ORIENTATION)
   {
      std::vector< size_t > res;
      check_nested_orientation(vertices, begin, end, std::back_inserter(res), eps);

      for (size_t i = 0; i < res.size(); ++i)
      {
         tmp.push_back(verification_result (VF_NESTED_ORIENTATION,
            res[i], static_cast< size_t >( -1 ), vertices[(*(begin + res[i]))[0]]));
      }
   }

   if (!tmp.empty())
   {
      result.errors = cg::array_1d< verification_result > (tmp.begin(), tmp.end());
      throw result;
   }
}

// Verification procedure checks and try to correct contour input data
// by verification_code declared in Algorithm
template< class VertexBuffer, class ContoursIterator >
   void check_n_correct_input_base( int verification_code, VertexBuffer const &vertices,
                                    ContoursIterator begin, ContoursIterator end )
{
   check_n_correct_input_base(verification_code, vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class VertexBuffer, class ContoursIterator >
   void check_n_correct_input_base( int verification_code, VertexBuffer const &vertices,
                                    ContoursIterator begin, ContoursIterator end,
                                    typename scalar< VertexBuffer >::type eps )
{
   typedef typename scalar< VertexBuffer >::type scalar_type;
   typedef cg::point_t< scalar_type, 2 > point_type;

   invalid_input_contours result ("Unknown");

   std::vector< verification_result > tmp;

   if (verification_code & VF_RECURRING_POINTS)
   {
      std::vector< std::pair< size_t, point_type > > res;
      check_n_correct_recurring_points(vertices, begin, end, std::back_inserter(res), eps);

      for (size_t i = 0; i < res.size(); ++i)
         tmp.push_back(verification_result (VF_RECURRING_POINTS, res[i].first, -1, res[i].second));
   }
   
   if (verification_code & VF_SELF_INTERSECTION)
   {
      int edgeOverlapBit = verification_code & VF_EDGE_OVERLAP;
      int otype = OT_NONE;

      if (edgeOverlapBit != 0)
      {
         if (edgeOverlapBit & VF_EDGE_OVERLAP_EXTERNAL)
            otype |= OT_EXTERNAL;
         if (edgeOverlapBit & VF_EDGE_OVERLAP_INTERNAL)
            otype |= OT_INTERNAL;
      }

      std::vector< intersection > res;
      check_self_intersection(vertices, begin, end, std::back_inserter(res), otype, eps);

      for (size_t i = 0; i < res.size(); ++i)
         tmp.push_back(verification_result (Verificator (VF_SELF_INTERSECTION | edgeOverlapBit),
                                                         res[i].cntA, res[i].cntB, res[i].refPoint));
   }

   if (verification_code & VF_NESTED_ORIENTATION)
   {
      std::vector< size_t > res;
      check_n_correct_nested_orientation(vertices, begin, end, std::back_inserter(res), eps);

      for (size_t i = 0; i < res.size(); ++i)
      {
         tmp.push_back(verification_result (VF_NESTED_ORIENTATION,
            res[i], -1, vertices[(*(begin + res[i]))[0]]));
         
      }
   }

   if (!tmp.empty())
   {
      result.errors = cg::array_1d< verification_result > (tmp.begin(), tmp.end());
      throw result;
   }
}

// Verification procedure checks objects with contour input data by verification_code declared in Algorithm
template< class VertexBuffer, class ObjectsIterator >
   void check_input_objects_base( int verification_code, VertexBuffer const &vertices,
                                  ObjectsIterator begin, ObjectsIterator end )
{
   check_input_objects_base(verification_code, vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class VertexBuffer, class ObjectsIterator >
   void check_input_objects_base( int verification_code, VertexBuffer const &vertices,
                                  ObjectsIterator begin, ObjectsIterator end,
                                  typename scalar< VertexBuffer >::type eps )
{
   typedef ObjectsIterator::value_type::const_iterator ContoursIterator;

   std::vector< ContoursIterator::value_type > input;
   input.reserve(end - begin);

   for (ObjectsIterator oIt = begin; oIt != end; ++oIt)
      input.insert(input.end(), oIt->begin(), oIt->end());

   std::vector< std::pair< size_t, size_t > > cnts2obj;
   cnts2obj.reserve(input.size());
   size_t objIdx = 0;
   for (ObjectsIterator oIt = begin; oIt != end; ++oIt, ++objIdx)
   {
      size_t cntIdx = 0;
      for (ContoursIterator cIt = oIt->begin(); cIt != oIt->end(); ++cIt, ++cntIdx)
         cnts2obj.push_back(std::make_pair(objIdx, cntIdx));
   }

   try
   {
      check_input_base(verification_code, vertices, input.begin(), input.end(), eps);
   }
   catch (invalid_input_contours &result)
   {
      invalid_input_contour_objects newRes (result.algorithm);
      std::vector< verification_result_objects > tmp;
      tmp.reserve(result.errors.size());
      for (size_t e = 0; e < result.errors.size(); ++e)
      {
         tmp.push_back(verification_result_objects (result.errors[e].verificator(),
            cnts2obj[result.errors[e].contourA()].first, cnts2obj[result.errors[e].contourA()].second,
            cnts2obj[result.errors[e].contourB()].first, cnts2obj[result.errors[e].contourB()].second,
            result.errors[e].refPoint()));
      }

      if (!tmp.empty())
         newRes.errors = cg::array_1d< verification_result_objects > (tmp.begin(), tmp.end());

      throw newRes;
   }
}

// Verification procedure checks and try to correct objects with contour input data
// by verification_code declared in Algorithm
template< class VertexBuffer, class ObjectsIterator >
   void check_n_correct_input_objects_base( int verification_code, VertexBuffer const &vertices,
                                            ObjectsIterator begin, ObjectsIterator end )
{
   check_n_correct_input_objects_base(verification_code, vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class VertexBuffer, class ObjectsIterator >
   void check_n_correct_input_objects_base( int verification_code, VertexBuffer const &vertices,
                                            ObjectsIterator begin, ObjectsIterator end,
                                            typename scalar< VertexBuffer >::type eps )
{
   typedef ObjectsIterator::value_type::const_iterator ContoursIterator;

   std::vector< ContoursIterator::value_type > input;
   input.reserve(end - begin);

   for (ObjectsIterator oIt = begin; oIt != end; ++oIt)
      input.insert(input.end(), oIt->begin(), oIt->end());

   std::vector< std::pair< size_t, size_t > > cnts2obj;
   cnts2obj.reserve(input.size());
   size_t objIdx = 0;
   for (ObjectsIterator oIt = begin; oIt != end; ++oIt, ++objIdx)
   {
      size_t cntIdx = 0;
      for (ContoursIterator cIt = oIt->begin(); cIt != oIt->end(); ++cIt, ++cntIdx)
         cnts2obj.push_back(std::make_pair(objIdx, cntIdx));
   }

   try
   {
      check_n_correct_input_base(verification_code, vertices, input.begin(), input.end(), eps);
   }
   catch (invalid_input_contours &result)
   {
      invalid_input_contour_objects newRes (result.algorithm);
      std::vector< verification_result_objects > tmp;
      tmp.reserve(result.errors.size());
      for (size_t e = 0; e < result.errors.size(); ++e)
      {
         tmp.push_back(verification_result_objects (result.errors[e].verificator(),
            cnts2obj[result.errors[e].contourA()].first, cnts2obj[result.errors[e].contourA()].second,
            cnts2obj[result.errors[e].contourB()].first, cnts2obj[result.errors[e].contourB()].second,
            result.errors[e].refPoint()));
      }

      if (!tmp.empty())
         newRes.errors = cg::array_1d< verification_result_objects > (tmp.begin(), tmp.end());

      throw newRes;
   }
}

//
// Verify algorithm helpers with static verification code
//

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_input( VertexBuffer const &vertices,
                     ContoursIterator begin, ContoursIterator end )
{
   check_input< Algorithm >(vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_input( VertexBuffer const &vertices,
                     ContoursIterator begin, ContoursIterator end,
                     typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_input_base(Algorithm::verification_code, vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_n_correct_input( VertexBuffer const &vertices,
                               ContoursIterator begin, ContoursIterator end )
{
   check_n_correct_input< Algorithm >(vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_n_correct_input( VertexBuffer const &vertices,
                               ContoursIterator begin, ContoursIterator end,
                               typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_n_correct_input_base(Algorithm::verification_code, vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_input_objects( VertexBuffer const &vertices,
                             ObjectsIterator begin, ObjectsIterator end )
{
   check_input_objects< Algorithm >(vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_input_objects( VertexBuffer const &vertices,
                             ObjectsIterator begin, ObjectsIterator end,
                             typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_input_objects_base(Algorithm::verification_code, vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_n_correct_input_objects( VertexBuffer const &vertices,
                                       ObjectsIterator begin, ObjectsIterator end )
{
   check_n_correct_input_objects< Algorithm >(vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_n_correct_input_objects( VertexBuffer const &vertices,
                                       ObjectsIterator begin, ObjectsIterator end,
                                       typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_n_correct_input_objects_base(Algorithm::verification_code, vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

//
// Verify algorithm helpers with parametrized verification code
//

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_input( typename Algorithm::Parameters const &params,
                     VertexBuffer const &vertices,
                     ContoursIterator begin, ContoursIterator end )
{
   check_input< Algorithm, VertexBuffer, ContoursIterator >(params,
      vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_input( typename Algorithm::Parameters const &params,
                     VertexBuffer const &vertices,
                     ContoursIterator begin, ContoursIterator end,
                     typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_input_base(Algorithm::verification_code(params), vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_n_correct_input( typename Algorithm::Parameters const &params,
                               VertexBuffer const &vertices,
                               ContoursIterator begin, ContoursIterator end )
{
   check_n_correct_input< Algorithm >(params, vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ContoursIterator >
   void check_n_correct_input( typename Algorithm::Parameters const &params,
                               VertexBuffer const &vertices,
                               ContoursIterator begin, ContoursIterator end,
                               typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_n_correct_input(Algorithm::verification_code(params), vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_input_objects( typename Algorithm::Parameters const &params,
                             VertexBuffer const &vertices,
                             ObjectsIterator begin, ObjectsIterator end )
{
   check_input_objects< Algorithm >(params, vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_input_objects( typename Algorithm::Parameters const &params,
                             VertexBuffer const &vertices,
                             ObjectsIterator begin, ObjectsIterator end,
                             typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_input_objects(Algorithm::verification_code(params), vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_n_correct_input_objects( typename Algorithm::Parameters const &params,
                                       VertexBuffer const &vertices,
                                       ObjectsIterator begin, ObjectsIterator end )
{
   check_n_correct_input_objects< Algorithm >(params, vertices, begin, end, cg::epsilon< scalar< VertexBuffer >::type >());
}

template< class Algorithm, class VertexBuffer, class ObjectsIterator >
   void check_n_correct_input_objects( typename Algorithm::Parameters const &params,
                                       VertexBuffer const &vertices,
                                       ObjectsIterator begin, ObjectsIterator end,
                                       typename scalar< VertexBuffer >::type eps )
{
   try
   {
      check_n_correct_input_objects(Algorithm::verification_code(params), vertices, begin, end, eps);
   }
   catch (invalid_input_contours &result)
   {
      result.algorithm = Algorithm::algorithm_name();
      throw;
   }
}

} // End of 'verification' namespace
} // End of 'cg' namespace
