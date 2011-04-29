#pragma once

#include "Geometry/grid2L.h"
#include "Geometry/Grid2L/subdiv.h"
#include "Contours/misc/smallcell.h"

#include "Geometry/cgal_predicates.h"

namespace cg {
namespace details {

//////////////////////////////////////////////////////////////////////////
template< class segment_id_type >
   struct add_processor
{
   typedef add_processor BigCellProcessor;
   typedef add_processor SmallCellProcessor;

   add_processor( segment_id_type seg_id )
      : seg_id_ (seg_id)
   {
   }

   template< class state, class small_cell_type >
      bool operator () ( state const &, small_cell_type &cell )
   {
      cell.to_process().push_back(seg_id_);
      return false;
   }

   BigCellProcessor & processgrid( ) { return *this; }

   template< class State, class bigcell_type >
      SmallCellProcessor &processbigcell( State const &, bigcell_type const & )
   {
      return *this;
   }

private:
   segment_id_type seg_id_;
};

//////////////////////////////////////////////////////////////////////////
template< class segment_id_type >
   struct remove_processor
{
   typedef remove_processor BigCellProcessor;
   typedef remove_processor SmallCellProcessor;

   remove_processor( segment_id_type seg_id )
      : seg_id_ (seg_id)
   {
   }

   template< class state, class small_cell_type >
      bool operator () ( state const &, small_cell_type &cell )
   {
      small_cell_type::segments_type::iterator place =
         std::find(cell.processed().begin(), cell.processed().end(), seg_id_);

      if (place != cell.processed().end())
         cell.processed().erase(place);

      place = std::find(cell.to_process().begin(), cell.to_process().end(), seg_id_);

      if (place != cell.to_process().end())
         cell.to_process().erase(place);

      return false;
   }

   BigCellProcessor & processgrid( ) { return *this; }

   template< class State, class bigcell_type >
      SmallCellProcessor &processbigcell( State const &, bigcell_type const & )
   {
      return *this;
   }

private:
   segment_id_type seg_id_;
};

inline cg::point_2 const &min( cg::point_2 const &a, cg::point_2 const &b )
{
   if (a > b)
      return b;
   return a;
}

inline cg::point_2 const &max( cg::point_2 const &a, cg::point_2 const &b )
{
   if (a > b)
      return a;
   return b;
}

inline bool operator < ( cg::segment_2 const &a, cg::segment_2 const &b )
{
   cg::segment_2 am (min(a.P0(), a.P1()), max(a.P0(), a.P1()));
   cg::segment_2 bm (min(b.P0(), b.P1()), max(b.P0(), b.P1()));
   return am.P0() < bm.P0() || (am.P0() == bm.P0() && am.P1() < bm.P1());
}

//////////////////////////////////////////////////////////////////////////
template< class Host, class segments_type, class intr_type, class out_iter >
   struct intersect_processor
{
   typedef typename segments_type::value_type segment_type;
   typedef typename segment_type::point_type  point_type;
   typedef typename point_type::scalar_type   scalar_type;

   typedef intersect_processor BigCellProcessor;
   typedef intersect_processor SmallCellProcessor;

   intersect_processor( Host &host, segments_type const & segments, out_iter output, scalar_type eps )
      : host_ (host)
      , segments_ (segments)
      , output_ (output)
      , eps_ (eps)
   {
   }

   template< class state, class small_cell_type >
      bool operator () ( state const &st, small_cell_type &cell )
   {
      typedef typename small_cell_type::segment_id_type segment_id_type;

      cell.processed().reserve(cell.processed().size() + cell.to_process().size());
      for (int i = cell.to_process().size() - 1; i >= 0; --i)
      {
         segment_id_type idA = cell.to_process()[i];
         cell.to_process().pop_back();

         for (size_t j = 0; j < cell.processed().size(); ++j)
         {
            segment_id_type idB = cell.processed()[j];

            segment_type::input_segment_type segA = segments_.at(idA).seg;
            segment_type::input_segment_type segB = segments_.at(idB).seg;
            if (!(segA.P0() < segA.P1()))
               segA = segment_type::input_segment_type (segA.P1(), segA.P0());
            if (!(segB.P0() < segB.P1()))
               segB = segment_type::input_segment_type (segB.P1(), segB.P0());

            point_type ipA, ipB;
            cg::intersection_type ires =
               cg::robust_isect_segments(segA, segB, ipA, ipB);

            if ((ires == cg::intersect || ires == cg::overlap) &&
                (cg::distance(ipA, segA) > eps_ || cg::distance(ipA, segB) > eps_) ||
                (ires == cg::overlap &&
                (cg::distance(ipB, segA) > eps_ || cg::distance(ipB, segB) > eps_)))
            {
               ires = cg::exact_isect_segments(segA, segB, ipA, ipB);
            }

            if (ires != cg::disjoint)
            {
               intr_type intr (idA, idB, ipA, ires);
               *output_++ = intr;

               bool a_vertex_intersection = ipA == segA.P0() || ipA == segA.P1();
               bool b_vertex_intersection = ipA == segB.P0() || ipA == segB.P1();

               if (a_vertex_intersection && b_vertex_intersection && ires == cg::overlap)
               {
                  ipA = ipB;
                  a_vertex_intersection = ipA == segA.P0() || ipA == segA.P1();
                  b_vertex_intersection = ipA == segB.P0() || ipA == segB.P1();
               }

               if (a_vertex_intersection && b_vertex_intersection &&
                   (ires != cg::overlap || segA == segB))
               {
                  if (host_.vertex_intersections_)
                     host_.yield_intersection(intr);
                  continue;
               }

               host_.yield_intersection(intr);
               if (!a_vertex_intersection)
                  subdiv_segment(idA, ipA);
               else
                  cell.to_process().push_back(idA);

               if (!b_vertex_intersection)
                  subdiv_segment(idB, ipA);

               return operator ()(st, cell);
            }
         }

         cell.processed().push_back(idA);
      }

      return false;
   }

   template< class segment_id_type >
      void subdiv_segment( segment_id_type segId, point_type const &p )
   {
      // Remove from grid segments to subdivide
      details::remove_processor< segment_id_type > rem_proc (segId);
      cg::visit(*host_.grid_, host_.holder_[segId].seg, rem_proc);

      // Subdivide
      host_.holder_.push_back(segment_type (
         segment_type::input_segment_type (host_.holder_[segId].seg.P0(), p),
         host_.holder_[segId].root_ancestor));

      host_.holder_.push_back(segment_type (
         segment_type::input_segment_type (p, host_.holder_[segId].seg.P1()),
         host_.holder_[segId].root_ancestor));

      host_.holder_[segId].left = host_.holder_.size() - 2;
      host_.holder_[segId].right = host_.holder_.size() - 1;

      {
         details::add_processor< segment_id_type > add_proc (host_.holder_.size() - 1);
         cg::visit(*host_.grid_, host_.holder_[host_.holder_.size() - 1].seg, add_proc);
      }
      {
         details::add_processor< segment_id_type > add_proc (host_.holder_.size() - 2);
         cg::visit(*host_.grid_, host_.holder_[host_.holder_.size() - 2].seg, add_proc);
      }
   }

   BigCellProcessor & processgrid( ) { return *this; }

   template< class State, class bigcell_type >
      SmallCellProcessor &processbigcell( State const &, bigcell_type const & )
   {
      return *this;
   }

private:
   Host &host_;
   segments_type const &segments_;
   out_iter output_;
   scalar_type eps_;
};

//////////////////////////////////////////////////////////////////////////
template< class segment_id_type >
   struct segments_holder
{
   typedef std::vector< segment_id_type > segments_type;
   typedef segment_id_type segment_id_type;

   void add_segment( segment_id_type it )
   {
     to_process_.push_back(it);
   }

   segments_type const &processed() const { return processed_; }
   segments_type       &processed()       { return processed_; }

   segments_type const &to_process() const { return to_process_; }
   segments_type       &to_process()       { return to_process_; }

private:
   segments_type processed_;
   segments_type to_process_;
};

//////////////////////////////////////////////////////////////////////////
template< class segments_type, class grid_type, class segment_id_type >
   struct segments_traits
{
   typedef typename segments_type::const_iterator iterator_type;
   typedef typename segments_type::value_type     segment_type;

   typedef typename segment_type::input_segment_type input_segment_type;

   segments_traits( segments_type const &segments )
      : segments_ (segments)
   {
   }

   segment_id_type begin() const { return 0; }
   segment_id_type end()   const { return (segment_id_type)segments_.size(); }

   segment_id_type distance( segment_id_type a, segment_id_type b ) const
   {
      return abs((int)a - (int)b);
   }

   input_segment_type const &getItem( segment_id_type it  ) const
   {
      return segments_[it].seg;
   }

   cg::rectangle_2 getBbox( segment_id_type it ) const
   {
      return cg::rectangle_2(segments_[it].seg.P0(), segments_[it].seg.P1());
   }

private:
   segments_type const &segments_;
};

} // End of 'details' namespace

//////////////////////////////////////////////////////////////////////////
template< class segment_id_type, class point_type >
   struct segments_intersection
{
   segments_intersection( segment_id_type idA, segment_id_type idB,
                          point_type const &p, intersection_type type )
      : idA_ (std::min(idA, idB))
      , idB_ (std::max(idA, idB))
      , p_ (p)
      , type_( type )
   {}

   segment_id_type idA() const { return idA_; }
   segment_id_type idB() const { return idB_; }

   point_type const &p() const { return p_; }
   intersection_type type() const { return type_; }

   friend bool operator < ( segments_intersection const &a, segments_intersection const &b )
   {
      return a.idA() < b.idA() || (a.idA() == b.idA() &&
             a.idB() < b.idB()) || (a.idA() == b.idA() && a.idB() == b.idB() &&
             a.p() < b.p());
   }

   friend bool operator == ( segments_intersection const &a, segments_intersection const &b )
   {
      return !(a < b) && !(b < a);
   }

private:
   segment_id_type idA_;
   segment_id_type idB_;

   point_type p_;
   intersection_type type_;
};

struct EmptyFilter
{
   template <class T> bool operator () (T const &, T const &) const { return true; }
};

//////////////////////////////////////////////////////////////////////////
template< class input_segment_type, class input_segments_type = std::vector< input_segment_type >, class EdgeFilter = EmptyFilter >
   struct robust_segments_intersector
      : public std::vector< segments_intersection< size_t, typename input_segment_type::point_type > >
{
public:
   typedef typename input_segment_type::scalar_type scalar_type;
   typedef typename input_segment_type::point_type  point_type;

   typedef size_t segment_id_type;

   typedef
      std::vector< segments_intersection< segment_id_type, point_type > >
      base_type;

public:
   struct segment_type
   {
      typedef point_type point_type;
      typedef input_segment_type input_segment_type;

      input_segment_type seg;
      segment_id_type root_ancestor;
      std::vector< std::pair< segment_id_type, bool > > equal_segs; // (id + collinearity)[]
      segment_id_type left, right;

      bool operator == ( segment_type const &other )
      {
         return seg == other.seg;
      }

      bool operator < ( segment_type const &other )
      {
         return details::operator < (seg, other.seg);
      }

      segment_type ( input_segment_type const &inp_seg, segment_id_type id )
         : seg (inp_seg)
         , root_ancestor (id)
         , left (static_cast< segment_id_type >(-1))
         , right (static_cast< segment_id_type >(-1))
      {
      }
   };

   typedef std::vector< segment_type > segments_type;

   typedef
      details::intersect_processor
      <
         robust_segments_intersector, segments_type,
         value_type, std::back_insert_iterator< base_type >
      >
      inter_proc;

   friend struct inter_proc;

public:
   robust_segments_intersector( input_segments_type const &segments,
                                bool vertex_intersections = false, scalar_type eps = 1e-5, EdgeFilter const & edge_filter = EdgeFilter() )
      : vertex_intersections_ (vertex_intersections)
   {
      for (input_segments_type::const_iterator it = segments.begin(); it != segments.end(); ++it)
         holder_.push_back(segment_type (*it, holder_.size()));
      segments_ = &holder_;

      unite_equal_segments();
      find_intersections(eps);
   }

   template< typename FwdIter >
      robust_segments_intersector( FwdIter p, FwdIter q,
                                   bool vertex_intersections = false, scalar_type eps = 1e-5, EdgeFilter const & edge_filter = EdgeFilter()  )
                                   :   edge_filter_ (edge_filter)
   {
      for (FwdIter it = p; it != q; ++it)
         holder_.push_back(segment_type (*it, holder_.size()));
      segments_ = &holder_;

      unite_equal_segments();
      find_intersections(vertex_intersections, eps);
   }

   segments_type const &holder() const
   {
      return holder_;
   }

   std::pair< size_t, bool > input2holder( size_t idx ) const
   {
      std::map< size_t, std::pair< size_t, bool > >::const_iterator it = input2holder_.find(idx);
      Verify(it != input2holder_.end());
      return it->second;
   }

private:
   void unite_equal_segments()
   {
      std::sort(holder_.begin(), holder_.end());

      for (size_t i = 0; i < holder_.size(); ++i)
      {
         size_t j = i + 1;
         while (j < holder_.size() && holder_[j] == holder_[i])
         {
            holder_[i].equal_segs.push_back(std::make_pair(holder_[j].root_ancestor,
               holder_[i].seg.P0() == holder_[j].seg.P0()));

            ++j;
         }
         i = j - 1;
      }

      holder_.erase(std::unique(holder_.begin(), holder_.end()), holder_.end());

      for (size_t i = 0; i < holder_.size(); ++i)
      {
         input2holder_[holder_[i].root_ancestor] = std::make_pair(i, true);
         for (size_t j = 0; j < holder_[i].equal_segs.size(); ++j)
            input2holder_[holder_[i].equal_segs[j].first] = std::make_pair(i, holder_[i].equal_segs[j].second);
      }
   }

   void create_grid()
   {
      typedef
         details::segments_traits< segments_type, grid_type, segment_id_type >
         segments_traits_type;

      segments_traits_type traits (*segments_);

      cg::AABB bb (traits.begin(), traits.end(), traits);
      if (!bb.empty())
         bb.inflate(1.);

      cg::Grid2LSubdiv subdiv (bb, 30, 5, 250);
      grid_.reset(new cg::Grid2LInitializer< grid_type > (traits.begin(), traits.end(),
                                                          traits, subdiv));
   }

   void yield_intersection( value_type const &intr )
   {
      segment_type const &aseg = holder_[intr.idA()];
      segment_type const &bseg = holder_[intr.idB()];

      segment_id_type idA = aseg.root_ancestor;
      segment_id_type idB = bseg.root_ancestor;

      if (edge_filter_(idA, idB))
         push_back(value_type (idA, idB, intr.p(), cg::intersect));

      for (size_t i = 0; i < bseg.equal_segs.size(); ++i)
         if (edge_filter_(idA, bseg.equal_segs[i].first))
            push_back(value_type (idA, bseg.equal_segs[i].first, intr.p(), cg::intersect));

      for (size_t i = 0; i < aseg.equal_segs.size(); ++i)
      {
         if (edge_filter_(idB, aseg.equal_segs[i].first))
            push_back(value_type (idB, aseg.equal_segs[i].first, intr.p(), cg::intersect));

         for (size_t j = 0; j < bseg.equal_segs.size(); ++j)
            if (edge_filter_(aseg.equal_segs[i].first, bseg.equal_segs[j].first))
               push_back(value_type (aseg.equal_segs[i].first, bseg.equal_segs[j].first, intr.p(), cg::intersect));
      }
   }

   void find_intersections( scalar_type eps )
   {
      create_grid();

      size_t num_intersected = static_cast< size_t >(-1);

      size_t iteration = 0;
      while (num_intersected != 0)
      {
         base_type intersections;
         inter_proc proc (*this, *segments_, std::back_inserter(intersections), eps);
         cg::visit_every_cell(*grid_, proc);

         vertex_intersections_ = false;
         num_intersected = intersections.size();

         ++iteration;
      }

      // Clean dupes
      std::sort(begin(), end());
      erase(std::unique(begin(), end()), end());
   }

private:
   typedef details::segments_holder< segment_id_type > small_cell_type;
   typedef cg::Grid2L< small_cell_type >               grid_type;

private:
   bool vertex_intersections_;
   segments_type const *segments_;
   EdgeFilter  edge_filter_;

   segments_type holder_;
   std::auto_ptr< grid_type > grid_;

   std::map< size_t, std::pair< size_t, bool > > input2holder_;
};

} // End of 'cg' namespace
