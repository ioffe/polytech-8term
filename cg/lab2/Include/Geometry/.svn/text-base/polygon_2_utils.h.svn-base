#pragma once

//#define DEBUG_POLYOPS 

#include <limits>

#include <boost/exception.hpp>

#include "contours/readytouse/contours_set_com.h"
#include "Viewer/viewer_dc.h"
#include "contours/readytouse/contour_data.h"

#include "geometry/polygon_2.h"
#include "geometry/polygon_2_io.h"
#include "Geometry/StraightSkeleton/offset.h"
#include "Geometry\PSLG\pslg_overlay.h"
#include "Geometry\cgal\polygon_algorithm.h"
#include "geometry\cgal\std_traits.h"

#include "Iterators/null_iterator.h"
#include <boost/lexical_cast.hpp>
#include "common/PerfCounter.h"

#include "common/FileUtils.h"

namespace cg
{
   typedef cg::dcel::DCEL< double > DCEL_type;

   template < class Traits > 
      cg::polygon_2 create_polygon( contours::IContoursSetIterator< Traits > & iter )
   {
      cg::polygon_2 res;
      contours::IContoursSetIterator< Traits >::const_contour_data cnt;

      while ( iter.Next( cnt ) )
      {
         std::vector< cg::point_2 > pts( cnt.begin(), cnt.end() );
         res.add_contour( pts.rbegin(), pts.rend() );
      }

      return res;
   }

   // Input polygon must not have self-intersections (and probably no recurring points),
   // otherwise it may throw cg::verification::nested_orientation_invalid_input_exception().
   inline cg::polygon_2 fix_orientation( cg::polygon_2 const & p, bool *isModifiedPtr = 0 )
   { 
      typedef std::vector< cg::point_2 > contour_t;
      typedef std::vector< std::vector< size_t > > index_buffer_t;

      contour_t vertices;
      index_buffer_t indices;

      indexate_polygon( p, vertices, indices );

      util::conting_null_iterator outIt;
      verification::check_n_correct_nested_orientation(
         vertices, indices.begin(), indices.end(), outIt );

      if ( isModifiedPtr )
         *isModifiedPtr = ( outIt.count() != 0 );

      return cg::polygon_2( vertices, indices.begin(), indices.end() );
   }

   inline cg::polygon_2 fix_recurring_points( cg::polygon_2 const & p, double eps = cg::epsilon< double >(), bool *isModifiedPtr = 0 )
   {
      typedef std::vector< cg::point_2 > contour_t;
      typedef std::vector< std::vector< size_t > > index_buffer_t;

      contour_t vertices;
      index_buffer_t indices;

      indexate_polygon( p, vertices, indices );

      util::conting_null_iterator outIt;
      verification::check_n_correct_recurring_points(
         vertices, indices.begin(), indices.end(), outIt, eps );

      if ( isModifiedPtr )
         *isModifiedPtr = ( outIt.count() != 0 );

      return cg::polygon_2( vertices, indices.begin(), indices.end() );
   }

   inline cg::polygon_2 fix_singular_angles( cg::polygon_2 const & p, double eps = cg::epsilon< double >(), bool *isModifiedPtr = 0 )
   {
      typedef std::vector< cg::point_2 > contour_t;
      typedef std::vector< std::vector< size_t > > index_buffer_t;

      contour_t vertices;
      index_buffer_t indices;

      indexate_polygon( p, vertices, indices );

      util::conting_null_iterator outIt;
      verification::check_n_correct_singular_angles(
         vertices, indices.begin(), indices.end(), outIt, eps );

      if ( isModifiedPtr )
         *isModifiedPtr = ( outIt.count() != 0 );

      return cg::polygon_2( vertices, indices.begin(), indices.end() );
   }

   inline cg::polygon_2 fix_straight_angles( cg::polygon_2 const & p, double eps = cg::epsilon< double >(), bool *isModifiedPtr = 0 )
   {
      typedef std::vector< cg::point_2 > contour_t;
      typedef std::vector< std::vector< size_t > > index_buffer_t;

      contour_t vertices;
      index_buffer_t indices;

      indexate_polygon( p, vertices, indices );

      util::conting_null_iterator outIt;
      verification::check_n_correct_straight_angles(
         vertices, indices.begin(), indices.end(), outIt, eps );

      if ( isModifiedPtr )
         *isModifiedPtr = ( outIt.count() != 0 );

      return cg::polygon_2( vertices, indices.begin(), indices.end() );
   }

   inline cg::polygon_2 fix_empty_subcontours( cg::polygon_2 const & p, bool *isModifiedPtr = 0 )
   {
      polygon_2 result;
      bool isPolygonModified(false);
      for (polygon_2::const_iterator cIt = p.begin(); cIt!= p.end(); ++cIt)
      {
         polygon_2::value_type const &c = *cIt;
         if (c.size() < 3)
            isPolygonModified = true;
         else
            result.add_contour(c.begin(), c.end(), c.hole());
      }

      if ( isModifiedPtr )
         *isModifiedPtr = isPolygonModified;

      return result;
   }

   namespace details
   {
      inline cg::polygon_2 skeleton_offset_impl( cg::polygon_2 const & p, offset_params const & params, double eps, bool ignore_holes )
      {
         if ( !p.size() )
            return cg::polygon_2();

         Verify( is_finite( p ) );

         cg::polygon_2 const & poly = fix_recurring_points( p, eps );

         typedef std::vector< cg::point_2 >  contour_t;
         typedef std::vector< size_t >       indices_t;
         typedef std::vector< indices_t >    multi_indices_t;

         contour_t points, offseted_points;
         multi_indices_t multicontour, offseted_indices;
         cg::indexate_polygon( poly, points, multicontour );

         typedef cg::StraightSkeletonGenerator< contour_t, multi_indices_t::iterator > skeleton_t;
         skeleton_t skeleton( points, multicontour.begin(), multicontour.end(), params );
         cg::skeleton_result( skeleton, offseted_points, std::back_inserter( offseted_indices ), ignore_holes );

         cg::polygon_2 res;
         for ( size_t i = 0; i != offseted_indices.size(); ++i )
         {
            contour_t cur;
            for ( size_t j = 0; j != offseted_indices[i].size(); ++j )
               cur.push_back( offseted_points[ offseted_indices[i][j] ] );

            if ( params.outside )
               res.add_contour( cur.rbegin(), cur.rend() );
            else
               res.add_contour( cur.begin(), cur.end() );
         }

         return res;
      }
   }

   inline cg::polygon_2 skeleton_offset( cg::polygon_2 const & p, offset_params const & params )
   {
      return details::skeleton_offset_impl( p, params, epsilon< double >(), true );
   }

   inline cg::polygon_2 skeleton_offset( cg::polygon_2 const & p, double dist )
   {
      return details::skeleton_offset_impl( p, offset_params( fabs( dist ), dist > 0, true ), epsilon< double >(), true );
   }

   inline cg::polygon_2 calc_skeleton( cg::polygon_2 const & p )
   {
      return details::skeleton_offset_impl( p, cg::offset_params( std::numeric_limits< double >::max(), false, true ), epsilon< double >(), false );
   }

   inline void check_poly( cg::polygon_2 const & p )
   {
      typedef std::vector< cg::point_2 > contour_t;
      typedef std::vector< std::vector< size_t > > index_buffer_t;

      contour_t vertices;
      index_buffer_t indices;

      indexate_polygon( p, vertices, indices );

      verification::check_input< cg::StraightSkeletonGenerator< contour_t, index_buffer_t > >(
         vertices, indices.begin(), indices.end() );
   }

   inline bool check_self_intersection( cg::polygon_2 const & p )
   {
      typedef std::vector< cg::point_2 > contour_t;
      typedef std::vector< std::vector< size_t > > index_buffer_t;

      contour_t vertices;
      index_buffer_t indices;

      indexate_polygon( p, vertices, indices );

      std::vector< verification::contours_intersection > intersections;
      verification::check_self_intersection( vertices, indices.begin(), indices.end(), 
         std::back_inserter( intersections ) );
      
      return intersections.empty();
   }

   inline void draw_polygon( viewer::viewer_dc & dc, cg::polygon_2 const & poly, float width = 1.f )
   {
      for ( size_t i = 0; i < poly.size(); ++i )
         dc.drawLineLoop( poly[i].begin(), poly[i].end(), width );
   }

   MIDL_INTERFACE("D99CC96F-BD67-49c2-9B8D-D6779539A34A")
   IPolygonIterator: cg::contours::IContoursSetIterator<> {};

   _COM_SMARTPTR_TYPEDEF( IPolygonIterator, __uuidof( IPolygonIterator ) );

   template < class Polygon >
      struct ATL_NO_VTABLE PolygonIterator
         : CComObjectRootEx< CComSingleThreadModel >
         , IPolygonIterator
   {
      BEGIN_COM_MAP( PolygonIterator )
         COM_INTERFACE_ENTRY( IPolygonIterator )
      END_COM_MAP( )

      void Init( Polygon const & poly )
      {
         cur_ = 0;
         poly_ = &poly;
      }

      bool Next( const_contour_data & data )
      {
         if ( cur_ >= poly_->size() )
            return false;

         data = const_contour_data( (*poly_)[cur_].size(), &(*poly_)[cur_][0], attr_, 0 );
         cur_++;
         return true;
      }

   private:
      Polygon const * poly_;
      size_t cur_;
      cg::Empty attr_;
   };

   template< class DCEL >
      void convertDCELToPolygon( polygon_2 &res, DCEL const &dcel )
   {
      cg::dcel::convertDCELToContour(res, dcel);
      for (size_t c = 0; c < res.size(); ++c)
         res[c].updateType();
   }

   namespace details
   {
      static cg::BooleanOp getOperation( ClippingOperation op )
      {
         switch ( op ) 
         {
         case Difference:
            return cg::BO_DIFFERENCE;
         case ExclusiveOR:
            return cg::BO_XOR;
         case Union:
            return cg::BO_UNION;
         case Intersection:
            return cg::BO_INTERSECTION;
         }

         return cg::BO_UNION;
      };
   }

   // TODO: make exception info typed
   typedef boost::error_info<struct clipping_error_tag, std::string>     clipping_error;
   typedef boost::error_info<struct conv_to_dcel_error_tag, std::string> conv_to_dcel_error;

   struct CFlagsOptRaw 
   {
      CFlagsOptRaw& operator = (ContourFlags const & cf) 
      {
         ex_ = cf;
         return *this;
      }

      friend bool has_0(CFlagsOptRaw const & x)
      {
         return x.ex_[0] != 0;
      }

      friend bool has_N(CFlagsOptRaw const & x, size_t N)
      {
         return x.ex_[N] != 0;
      }

      template <class F>
      friend void for_all_set_greater_0(CFlagsOptRaw const & x, F f)
      {
         for (size_t i = 1; i != x.ex_.size(); ++i)
         {
            if (x.ex_[i])
               f(i);
         }
      }


   private:
      ContourFlags   ex_;
   };

   struct CFlagsOpt
   {
      CFlagsOpt& operator = (std::vector<size_t> const & cf)
      {
         cf_ = cf;
         return *this;
      }

      CFlagsOpt& operator = (ContourFlags const & cf) 
      {
         cf_.clear();

         for (size_t i = 0; i != cf.size(); ++i)
         {
            if (cf[i])
               cf_.push_back(i);
         }

         return *this;
      }

      friend bool has_0(CFlagsOpt const & x)
      {
         return !x.cf_.empty() && x.cf_.front() == 0;
      }

      friend bool has_N(CFlagsOpt const & x, size_t N)
      {
         return std::binary_search(x.cf_.begin(), x.cf_.end(), N);
      }

      template <class F>
         friend void for_all_set_greater_0(CFlagsOpt const & x, F f)
      {
         std::vector<size_t>::const_iterator it = x.cf_.begin();

         if (it != x.cf_.end())
         {
            if (*it == 0)
               ++it;


            for (; it != x.cf_.end(); ++it)
            {
               f(*it);
            }
         }
      }


   private:
      std::vector<size_t>  cf_;
   };

   struct CFlagsPtrOpt
   {
      CFlagsPtrOpt& operator = (std::vector<size_t> const * cf)
      {
         cf_ = cf;
         return *this;
      }

      friend bool has_0(CFlagsPtrOpt const & x)
      {
         return !x.cf_->empty() && x.cf_->front() == 0;
      }

      friend bool has_N(CFlagsPtrOpt const & x, size_t N)
      {
         return std::binary_search(x.cf_->begin(), x.cf_->end(), N);
      }

      template <class F>
      friend void for_all_set_greater_0(CFlagsPtrOpt const & x, F f)
      {
         std::vector<size_t>::const_iterator it = x.cf_->begin();

         if (it != x.cf_->end())
         {
            if (*it == 0)
               ++it;


            for (; it != x.cf_->end(); ++it)
            {
               f(*it);
            }
         }
      }


   private:
      std::vector<size_t>  const *cf_;
   };

   struct ManyContourData
   {
      typedef CFlagsPtrOpt setin_type;

      setin_type      setIn;
      int             dcelId;

      ManyContourData(int id) : dcelId(id) {}

      friend void SetIn(ManyContourData & data, std::vector<size_t> const * s)
      {
         data.setIn = s;
      }

   };
#ifdef DEBUG_POLYOPS
   template <class PolygonIterator>
      void dump(PolygonIterator begin, PolygonIterator end, polygon_2 const & clipper)
      {
         static int i = 0;

         std::string dir_name;

         do 
         {
            dir_name = "d:\\crashes\\" + boost::lexical_cast<std::string>(i);
            
            if (CreateDirectoryA(dir_name.c_str(),NULL) != 0)
               break;

            ++i;

            if(ERROR_ALREADY_EXISTS !=GetLastError())
               throw std::exception("don't know what's the problem");

         } while (true);

         dir_name += "\\";

         std::ofstream((dir_name + "0.pl").c_str()) << clipper;

         int n = 1;

         for (PolygonIterator it = begin; it != end; ++it, ++n)
         {
            std::ofstream((dir_name + boost::lexical_cast<std::string>(n) + ".pl").c_str()) << *it;
         }

      }
#endif

      inline cg::polygon_2 clip( ClippingOperation op,
         polygon_2 const &sub, polygon_2 const &clip );

      template <class Traits, class PolygonIterator>
      void many_by_one_base(PolygonIterator begin, PolygonIterator end, polygon_2 const & clipper, BooleanOp boop)
      {
         typedef cg::dcel::DCEL<double, cg::Empty, ManyContourData>  DCEL_type;

         std::list<DCEL_type> dcels;

         PerfCounter pf;

#ifdef DEBUG_POLYOPS
         double t1, t2, t3;
#endif
         dcels.push_back(DCEL_type());
         cg::dcel::convertContourToDCEL(dcels.back(), clipper, -1);

         // Reflection: input polygon index -> index in `dcels' vector.
         std::vector<size_t>  inputPolyIdxToDCELIdx;

         size_t polyIdx(0);
         for (PolygonIterator it = begin; it != end; ++it, ++polyIdx)
         {
            bool empty;
            if (it->size() > 0)
            {
               dcels.push_back(DCEL_type());

               empty = !cg::dcel::convertContourToDCEL(dcels.back(), *it, dcels.size() - 1);
               if (empty)
               {
                  dcels.pop_back();
               }
            }
            else
            {
               empty = true;
            }

            if (!empty)
            {
               inputPolyIdxToDCELIdx.push_back(dcels.size() - 1);
            }
            else
            {
               // If input polygon is empty it will not be placed in `dcels' 
               // so it has no index in input polygons.
               inputPolyIdxToDCELIdx.push_back(static_cast<size_t>(-1));
            }
         }
#ifdef DEBUG_POLYOPS
         t1 = pf.time(); pf.restart();
#endif

         DCEL_type res;

         try
         {
            cg::pslg::overlay(res, dcels.begin(), dcels.end(), boop);

#ifdef DEBUG_POLYOPS
            t2 = pf.time(); pf.restart();
#endif
         }                 
         catch (boost::exception & ex)
         {
            std::stringstream s;

            s << "Polygon_2 clipping error" << std::endl;
            s << "clip polygon dump:" << std::endl;
            s << clipper;

            for (PolygonIterator it = begin; it != end; ++it)
            {
               s << "sub polygon dump:" << std::endl;
               s << *it;
            }

            ex << clipping_error(s.str());
#ifdef DEBUG_POLYOPS
            dump(begin, end, clipper);
#endif

            throw;
         }
         catch (...)
         {
#ifdef DEBUG_POLYOPS
            dump(begin, end, clipper);
#endif
            throw;
         }

         try 
         {
            DCEL_type::ContoursToSegments c2s;
            res.buildContoursToSegments(c2s, Traits(-1));

            size_t polyIdx(0);
            for (PolygonIterator it = begin; it != end; ++it, ++polyIdx)
            {
               cg::polygon_2 &poly = *it;
               poly.clear();

               if (inputPolyIdxToDCELIdx[polyIdx] != static_cast<size_t>(-1))
               {
                  Traits traits(inputPolyIdxToDCELIdx[polyIdx]);
                  if (!c2s[inputPolyIdxToDCELIdx[polyIdx]].empty())
                     cg::dcel::convertNthDcelToPolygon(inputPolyIdxToDCELIdx[polyIdx], poly, res, c2s[inputPolyIdxToDCELIdx[polyIdx]], traits);
               }
               else
               {
                  // Currently implemented only intersection and difference
                  // boolean operations in both of them input empty contour
                  // clipped with any other contour still be empty.
                  // If other operations will be implemented there should be
                  // code for handling BoolOp(empty_polygon, clipper).
               }
            }

#ifdef DEBUG_POLYOPS
            t3 = pf.time(); pf.restart();
            std::ofstream("d:\\cut.log") << "t1 = " << t1 << " t2 = " << t2 << " t3 = " << t3;
#endif
            return;
         }
         catch (boost::exception & ex)
         {
            std::stringstream s;

            s << "Convert DCEL to Polygons crushed after clipping operation" << std::endl;
            s << "clip polygon dump:" << std::endl;
            s << clipper;

            for (PolygonIterator it = begin; it != end; ++it)
            {
               s << "sub polygon dump:" << std::endl;
               s << *it;
            }

            ex << conv_to_dcel_error(s.str());
#ifdef DEBUG_POLYOPS
            dump(begin, end, clipper);
#endif
            // Known exception in mass polygons clipper.
            // Pass it and at the end of this function slow per-polygon 
            // clipping will be done.
         }
         catch (...)
         {
#ifdef DEBUG_POLYOPS
            dump(begin, end, clipper);
#endif
            throw;
         }

         // Slow per-polygon clipping (for cases when mass clipping failed).
         for (PolygonIterator it = begin; it != end; ++it)
         {
            *it = clip(boop == BO_INTERSECTION ? Intersection : Difference, *it, clipper);
         }
      }


   template <class PolygonIterator>
      void cut_many_by_one(PolygonIterator begin, PolygonIterator end, polygon_2 const & clipper)
      {
         many_by_one_base<cg::dcel::details::CutManyByOneTraits>(begin, end, clipper, BO_IGNORE);
      }

   template <class PolygonIterator>
      void intersect_many_with_one(PolygonIterator begin, PolygonIterator end, polygon_2 const & clipper)
      {
         many_by_one_base<cg::dcel::details::IntersectManyByOneTraits>(begin, end, clipper, BO_IGNORE);
      }

   inline void cut_many_by_one(std::list<polygon_2> & inp, polygon_2 const & clipper)
   {
      cut_many_by_one(inp.begin(), inp.end(), clipper);
   }
     
   inline void subdivide(polygon_2 & pl, int N)
   {
      std::list<std::vector<cg::point_2> >   pts;
      
      for (polygon_2::const_iterator c_it = pl.begin(); c_it != pl.end(); ++c_it)
      {
         pts.push_back(std::vector<cg::point_2>());
         pts.back().push_back(c_it->front());

         for (polygon_2::value_type::const_iterator it = c_it->begin(); it != c_it->end() - 1; ++it)
         {
            cg::Lerp<double, cg::point_2> L(0, 1, it[0], it[1]);

            for (int i = 0; i != N; ++i)
            {
               pts.back().push_back(L(double(i + 1) / N));
            }
         }
      }

      pl = polygon_2(pts);
   }

   inline bool subdivide(const char * filename, int N)
   {
      polygon_2 pl;

      {
         std::ifstream in(filename);
         if (!in)
            return false;

         in >> pl;
      }

      subdivide(pl, N);

      {
         std::ofstream out(filename);

         out << pl;
      }
      return true;
   }

   inline void subdivide_clipping()
   {
      int N = 2;

      subdivide("d:\\dump\\0.pl", N);

      for (int i = 1; ; ++i)
      {
         if (!subdivide(("d:\\dump\\" + boost::lexical_cast<std::string>(i) + ".pl").c_str(), N))
            break;
      }
   }

   inline void test_clipping()
   {
      //subdivide_clipping();


      polygon_2 clipper;
      {
         std::ifstream ifs("d:\\dump\\0.pl");
         ifs >> clipper;
      }

      std::list<polygon_2> pls;

      for (int i = 1; ; ++i)
      {
         std::ifstream in(("d:\\dump\\" + boost::lexical_cast<std::string>(i) + ".pl").c_str());
         if (!in)
            break;

         pls.push_back(polygon_2());
         in >> pls.back();
      }

      cut_many_by_one(pls.begin(), pls.end(), clipper);
   }

   inline void clip_simple_base( ClippingOperation op,
                                 polygon_2 const &sub, polygon_2 const &clip, polygon_2 & res )
   {
      DCEL_type a, b;
      cg::dcel::convertContourToDCEL(a, sub);
      cg::dcel::convertContourToDCEL(b, clip);

      try
      {
         cg::pslg::overlay(a, a, b, details::getOperation(op));
      }                 
      catch (boost::exception & ex)
      {
         std::stringstream s;

         s << "Polygon_2 clipping error" << std::endl;
         s << "sub polygon dump:" << std::endl;
         s << sub;
         s << "clip polygon dump:" << std::endl;
         s << clip;

         ex << clipping_error(s.str());

         throw;
      }

      res.clear();
      try
      {
         convertDCELToPolygon(res, a);
      }
      catch (boost::exception & ex)
      {
         std::stringstream s;

         s << "Convert DCEL to Polygon crushed after clipping operation" << std::endl;
         s << "sub polygon dump:" << std::endl;
         s << sub;
         s << "clip polygon dump:" << std::endl;
         s << clip;

         ex << conv_to_dcel_error(s.str());

         throw;
      }
   }

   inline void clip_simple_base_subd( ClippingOperation op, polygon_2 &sub, polygon_2 &clip, polygon_2 & res )
   {
      DCEL_type a, b;
      cg::dcel::convertContourToDCEL(a, sub);
      cg::dcel::convertContourToDCEL(b, clip);

      DCEL_type resDCEL;
      try
      {
         cg::pslg::overlay_subd(resDCEL, a, b, details::getOperation(op));
      }                 
      catch (boost::exception & ex)
      {
         std::stringstream s;
         
         s << "Polygon_2 clipping error" << std::endl;
         s << "sub polygon dump:" << std::endl;
         s << sub;
         s << "clip polygon dump:" << std::endl;
         s << clip;

         ex << clipping_error(s.str());

         throw;
      }

      res.clear();
      try
      {
         convertDCELToPolygon(res, resDCEL);
         sub.clear();
         convertDCELToPolygon(sub, a);
         clip.clear();
         convertDCELToPolygon(clip, b);
      }
      catch (boost::exception & ex)
      {
         std::stringstream s;

         s << "Convert DCEL to Polygon crushed after clipping operation" << std::endl;
         s << "sub polygon dump:" << std::endl;
         s << sub;
         s << "clip polygon dump:" << std::endl;
         s << clip;

         ex << conv_to_dcel_error(s.str());

         throw;
      }
   }

   inline void check_clip_poly_pair( polygon_2 const &sub, polygon_2 const &clip )
   {
      std::vector< cg::point_2 > vertices;
      std::vector< std::vector< size_t > > indices;
      indexate_polygon(sub, vertices, indices);

      cg::verification::check_input< cg::PSLGOverlayProcessor< DCEL_type > >(vertices,
         indices.begin(), indices.end());

      vertices.clear();
      indices.clear();
      indexate_polygon(clip, vertices, indices);

      cg::verification::check_input< cg::PSLGOverlayProcessor< DCEL_type > >(vertices,
         indices.begin(), indices.end());
   }

   inline void clip( ClippingOperation op, polygon_2 const &sub, polygon_2 const &clip, polygon_2 &res )
   {
      try
      {
         clip_simple_base(op, sub, clip, res);
      }
      catch (...)
      {
         check_clip_poly_pair(sub, clip);
         throw;
      }
   }

   inline void clip_subd( ClippingOperation op, polygon_2 &sub, polygon_2 &clip, polygon_2 &res )
   {
      try
      {
         clip_simple_base_subd(op, sub, clip, res);
      }
      catch (...)
      {
         check_clip_poly_pair(sub, clip);
         throw;
      }
   }

   template< class PolyFwdIterator >
      void clip_base( ClippingOperation op, PolyFwdIterator begin, PolyFwdIterator end, polygon_2 &res )
   {
      std::vector< DCEL_type > input;
      for (PolyFwdIterator pIt = begin; pIt != end; ++pIt)
      {
         input.push_back(DCEL_type ());
         cg::dcel::convertContourToDCEL(input.back(), *pIt);
      }

      DCEL_type resDCEL;
      try
      {
         cg::pslg::overlay(resDCEL, input.begin(), input.end(), details::getOperation(op));
      }
      catch (boost::exception & e)
      {
         std::stringstream s;

         s << "Polygon_2 clipping error" << std::endl;
         size_t polyIdx = 0;
         for (PolyFwdIterator pIt = begin; pIt != end; ++pIt, ++polyIdx)
            s << "polygon " << polyIdx << " dump:" << std::endl << *pIt;

         e << clipping_error(s.str());
         throw;
      }

      convertDCELToPolygon(res, resDCEL);
   }

   template< class PolyFwdIterator >
      void clip_base_subd( ClippingOperation op, PolyFwdIterator begin, PolyFwdIterator end, polygon_2 &res )
   {
      std::vector< DCEL_type > input;
      for (PolyFwdIterator pIt = begin; pIt != end; ++pIt)
      {
         input.push_back(DCEL_type ());
         cg::dcel::convertContourToDCEL(input.back(), *pIt);
      }

      DCEL_type resDCEL;
      try
      {
         cg::pslg::overlay_subd(resDCEL, input.begin(), input.end(), details::getOperation(op));
      }
      catch (boost::exception & ex)
      {
         std::stringstream s;

         s << "Polygon_2 clipping error" << std::endl;
         size_t polyIdx = 0;
         for (PolyFwdIterator pIt = begin; pIt != end; ++pIt, ++polyIdx)
            s << "polygon " << polyIdx << " dump:" << std::endl << *pIt;

         ex << clipping_error(s.str());

         throw;
      }

      convertDCELToPolygon(res, resDCEL);
      std::vector< DCEL_type >::iterator inp_it = input.begin();
      for (PolyFwdIterator pIt = begin; pIt != end; ++pIt, ++inp_it)
      {
         pIt->clear();
         convertDCELToPolygon(*pIt, *inp_it);
      }
   }

   template< class PolyFwdIterator >
      void check_clip_poly( PolyFwdIterator begin, PolyFwdIterator end )
   {
      std::vector< cg::point_2 > vertices;
      std::vector< std::vector< size_t > > indices;

      for (PolyFwdIterator pIt = begin; pIt != end; ++pIt)
      {
         vertices.clear();
         indices.clear();

         indexate_polygon(*pIt, vertices, indices);

         cg::verification::check_input< cg::PSLGOverlayProcessor< DCEL_type > >(vertices,
            indices.begin(), indices.end());
      }
   }

   template< class PolyFwdIterator >
      void clip( ClippingOperation op, PolyFwdIterator begin, PolyFwdIterator end, polygon_2 & res )
   {
      res.clear();
      if (begin == end)
         return;

      try
      {
         clip_base(op, begin, end, res);
      }
      catch (...)
      {
         check_clip_poly(begin, end);
         throw;
      }
   }

   template< class PolyFwdIterator >
      void clip_subd( ClippingOperation op, PolyFwdIterator begin, PolyFwdIterator end, polygon_2 & res )
   {
      res.clear();
      if (begin == end)
         return;

      try
      {
         clip_base_subd(op, begin, end, res);
      }
      catch (...)
      {
         check_clip_poly(begin, end);
         throw;
      }
   }

   inline cg::polygon_2 clip( ClippingOperation op,
                              polygon_2 const &sub, polygon_2 const &clip )
   {
      cg::polygon_2 res;
      cg::clip( op, sub, clip, res );
      return res;
   }

   inline cg::polygon_2 clip_subd( ClippingOperation op, polygon_2 &sub, polygon_2 &clip )
   {
      cg::polygon_2 res;
      cg::clip_subd( op, sub, clip, res );
      return res;
   }

   template< class PolyFwdIterator >
      cg::polygon_2 clip( ClippingOperation op,
                          PolyFwdIterator begin, PolyFwdIterator end )
   {
      cg::polygon_2 res;
      cg::clip( op, begin, end, res );
      return res;
   }

   template< class PolyFwdIterator >
      cg::polygon_2 clip_subd( ClippingOperation op,
                               PolyFwdIterator begin, PolyFwdIterator end )
   {
      cg::polygon_2 res;
      cg::clip_subd( op, begin, end, res );
      return res;
   }

   namespace subdivide_poly_details
   {
      struct contour_attr
      {
         contour_attr( bool hole, size_t idx )
            : hole( hole )
            , idx( idx )
         {}

         bool hole;
         size_t idx;
      };
   }

   struct subdivide_poly_failure_exception {};

   // Require that input subcontours have valid orientation.
   template < class OutIter >
      void subdivide_poly( cg::polygon_2 const & poly, OutIter output )
   {
      using namespace cg::contours;

      typedef gen::nested_contours_set< contours_set_traits< double, subdivide_poly_details::contour_attr > > contours_t;
      contours_t collision;

      std::map< size_t, std::vector< size_t > >  contours_res;

      for ( size_t l = 0; l != poly.size( ); ++l )
      {
         std::vector< point_2 > pts( poly[l].begin( ), poly[l].end( ) );
         if ( poly[l].hole( ) )
            collision.add_contour( pts.rbegin( ), pts.rend( ), subdivide_poly_details::contour_attr( poly[l].hole( ), l ) );
         else
            collision.add_contour( pts.begin( ), pts.end( ), subdivide_poly_details::contour_attr( poly[l].hole( ), l ) );

         if ( !poly[l].hole( ) )
            contours_res[l].push_back( l );
      }

      collision.process();

      for ( contours_t::contours_iterator it = collision.contours_begin(); it != collision.contours_end(); ++it )
      {
         subdivide_poly_details::contour_attr const & attr = collision.attr( *it );

         if ( attr.hole )
         {
            contour_type const contourType = collision.containing_contour( *it );
            if ( contourType )
            {
               size_t const idx = collision.attr( contourType ).idx;

               if ( contours_res.find( idx ) != contours_res.end( ) )
                  contours_res[ idx ].push_back( attr.idx );
               else
                  throw boost::enable_error_info( subdivide_poly_failure_exception( ) );
            }
            else
               throw boost::enable_error_info( subdivide_poly_failure_exception( ) );
         }
      }

      for ( std::map< size_t, std::vector< size_t > >::const_iterator it = contours_res.begin( ); it != contours_res.end( ); ++it )
      {
         cg::polygon_2 res;
         for ( size_t l = 0; l != it->second.size( ); ++l )
            res.add_contour( poly[it->second[l]].begin( ), poly[it->second[l]].end( ), poly[it->second[l]].hole( ) );

         *output++ = res;
      }
   }

   inline bool has_intersection( polygon_2 const & poly, line_2 const & ray, point_2 * result = NULL )
   {
      double min_t = std::numeric_limits< double >::max();
      for (size_t c = 0; c < poly.size(); c++)
         for (size_t v = 0; v < poly[c].size(); v++)
         {
            size_t v_next = (v + 1) % poly[c].size();
            segment_2 seg(poly[c][v], poly[c][v_next]);
            point_2 out;
            if (has_intersection(ray, line_2(seg.P0(), seg.P1(), line::by_points), out))
            {
               double cur_t = ray(out);
               if (cur_t > 0 && cur_t < min_t && seg(out) >= 0 && seg(out) <= 1)
               {
                  min_t = cur_t;
                  if (result)
                     *result = out;
               }
            }
         }
      return min_t < std::numeric_limits< double >::max();
   }

   inline double distance( point_2 const & p, polygon_2 const & poly )
   {
      double dist = std::numeric_limits< double >::max();
      for (size_t c = 0; c < poly.size(); c++)
         for (size_t v = 0; v < poly[c].size(); v++)
         {
            double cur_dist = cg::distance(p, segment_2(poly[c][v], poly[c][(v + 1) % poly[c].size()]));
            if (cur_dist < dist)
               dist = cur_dist;
         }
      return dist;
   }

   inline double distance_skel( point_2 const & p, polygon_2 const & poly )
   {
      std::pair<size_t, size_t> closest((size_t)-1, (size_t)-1);
      double dist = std::numeric_limits< double >::max();
      for (size_t c = 0; c < poly.size(); c++)
         for (size_t v = 0; v < poly[c].size(); v++)
         {
            segment_2 cur_seg(poly[c][v], poly[c][(v + 1) % poly[c].size()]);
            double cur_dist = cg::distance(p, cur_seg);
            if (cg::eq(cur_dist, dist))
            {
               segment_2 prev_seg(poly[c][(v + (poly[c].size() - 1)) % poly[c].size()], poly[c][v]);
               if (cg::eq(cg::distance(p, prev_seg), cur_dist))
               {
                  if (cg::angle(p - poly[c][v], -cg::direction(prev_seg)) <
                      cg::angle(cg::direction(cur_seg), p - poly[c][v]))
                     cur_dist = cg::distance(p, line_2(prev_seg.P0(), prev_seg.P1(), cg::line::by_points));
                  else
                     cur_dist = cg::distance(p, line_2(cur_seg.P0(), cur_seg.P1(), cg::line::by_points));
               }
            }

            if (cur_dist < dist)
            {
               dist = cur_dist;
               closest = std::make_pair(c, v);
            }            
         }

      return dist;
   }
}
