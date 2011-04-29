#pragma once

#pragma warning (push)
#pragma warning (disable : 4706)

#include <boost/exception.hpp>

#pragma warning (pop)

namespace cg
{

namespace dcel
{

// Iterator for additional data initialization in addContour method
template< class AddVertexData >
   struct EmptyDataIterator
{
   typedef AddVertexData value_type;

   value_type const &operator * () const
   {
      return addData_;
   }

   value_type const *operator -> () const
   {
      return (&**this);
   }

   EmptyDataIterator &operator ++ ()
   {
      return (*this);
   }

   EmptyDataIterator operator ++ ( int )
   {
      iterator temp = *this;
      ++*this;
      return temp;
   }

private:
   AddVertexData addData_;
};

typedef EmptyDataIterator< cg::Empty > NoVertexData;

namespace details
{

//
// Iteration base class for cycled iteration of edges
//

template< class EdgeContainer, class IterationTraits >
   class base_edge_const_iterator
{
public:
   typedef typename EdgeContainer::value_type value_type;
   typedef IterationTraits traits_type;

   base_edge_const_iterator (IterationTraits const & traits = IterationTraits())
      : traits_(traits), edges_ (NULL), idx_ (static_cast< size_t >( -1 )), startIdx_ (static_cast< size_t >( -1 )), end_ (false)
   {
   }

   base_edge_const_iterator (EdgeContainer const *edges, size_t idx = -1, bool end = false, IterationTraits const & traits = IterationTraits())
      : traits_(traits), edges_ ((EdgeContainer *)edges), idx_ (idx), startIdx_ (idx), end_ (end)
   {
   }

   value_type const &operator * () const
   {
      return (*edges_)[idx_];
   }

   value_type const *operator -> () const
   {
      return (&**this);
   }

   base_edge_const_iterator &operator ++ ()
   {
      if (end_)
         end_ = false;

      idx_ = traits_.next(*edges_, idx_);
      if (idx_ == startIdx_ && !end_)
         end_ = true;

      return (*this);
   }

   base_edge_const_iterator operator ++ ( int )
   {
      exiting_edge_const_iterator temp = *this;
      ++*this;
      return temp;
   }

   base_edge_const_iterator &operator -- ()
   {
      if (end_)
         end_ = false;

      idx_ = traits_.prev(*edges_, idx_);
      if (idx_ == startIdx_ && !end_)
         end_ = true;

      return (*this);
   }

   base_edge_const_iterator operator -- ( int )
   {
      exiting_edge_const_iterator temp = *this;
      --*this;
      return temp;
   }

   bool operator == ( base_edge_const_iterator const &right ) const
   {
      return edges_ == right.edges_ && idx_ == right.idx_ &&
         startIdx_ == right.startIdx_ && end_ == right.end_;
   }

   bool operator != ( base_edge_const_iterator const &right ) const
   {
      return !(*this == right);
   }

   size_t index() const
   {
      return idx_;
   }

protected:
   size_t idx_;

   size_t startIdx_;
   bool end_;

   EdgeContainer *edges_;
   IterationTraits traits_;
};

template< class EdgeContainer, class IterationTraits >
   class base_edge_iterator : public base_edge_const_iterator< EdgeContainer, IterationTraits >
{
public:
   typedef typename EdgeContainer::value_type value_type;
   typedef base_edge_const_iterator< EdgeContainer, IterationTraits > Base;

   base_edge_iterator (IterationTraits const & traits = IterationTraits())
      :  Base(traits)
   {
   }

   base_edge_iterator (EdgeContainer *edges, size_t idx = -1, bool end = false, IterationTraits const & traits = IterationTraits() )
      : Base (edges, idx, end, traits)
   {
   }

   value_type &operator * () const
   {
      return ((value_type &)**(Base *)this);
   }

   value_type *operator -> () const
   {
      return (&**this);
   }

   base_edge_iterator &operator ++ ()
   {
      if (end_)
         end_ = false;

      idx_ = traits_.next(*edges_, idx_);
      if (idx_ == startIdx_ && !end_)
         end_ = true;

      return (*this);
   }

   base_edge_iterator operator ++ ( int )
   {
      iterator temp = *this;
      ++*this;
      return temp;
   }

   base_edge_iterator &operator -- ()
   {
      if (end_)
         end_ = false;

      idx_ = traits_.prev(*edges_, idx_);
      if (idx_ == startIdx_ && !end_)
         end_ = true;

      return (*this);
   }

   base_edge_iterator operator -- ( int )
   {
      iterator temp = *this;
      --*this;
      return temp;
   }
};


//
// Exiting edges iteration
//

struct ExitingEdgesIterationTraits
{
   template< class EdgeContainer >
      static size_t next( EdgeContainer const &edges, size_t idx )
   {
      return edges[edges[idx].prevEdge].twinEdge;
   }

   template< class EdgeContainer >
      static size_t prev( EdgeContainer const &edges, size_t idx )
   {
      return edges[edges[idx].twinEdge].nextEdge;
   }
};

//
// Entering edges iteration
//

struct EnteringEdgesIterationTraits
{
   template< class EdgeContainer >
      static size_t next( EdgeContainer const &edges, size_t idx )
   {
      return edges[edges[idx].twinEdge].prevEdge;
   }

   template< class EdgeContainer >
      static size_t prev( EdgeContainer const &edges, size_t idx )
   {
      return edges[edges[idx].nextEdge].twinEdge;
   }
};

//
// Cycle edges iteration
//

struct CycleEdgesIterationTraits
{
   template< class EdgeContainer >
      static size_t next( EdgeContainer const &edges, size_t idx )
   {
      return edges[idx].nextEdge;
   }

   template< class EdgeContainer >
      static size_t prev( EdgeContainer const &edges, size_t idx )
   {
      return edges[idx].prevEdge;
   }
};

template <char In>
struct ManyByOneTraits
{
   ManyByOneTraits(int dcelId) : dcelId(dcelId) {}

   template <class EdgeContainer>
   bool is_mine(EdgeContainer const & edges, size_t idx) const
   {
      typename EdgeContainer::value_type::data_type::setin_type const & setIn = edges[idx].data.setIn;
      int dcId = edges[idx].data.dcelId;
      return (dcId == dcelId || dcId == -1) && has_0(setIn) == In && has_N(setIn, dcelId);
   }

   template <class Contour2Segment>
      struct Inserter  
      {
         Inserter(size_t idx, Contour2Segment & contours2segment)
            :  idx(idx), contours2segment(contours2segment)
         {}

         void operator () (size_t N) 
         {
            contours2segment[N].insert(idx);
         }

      private:
         size_t                idx;
         Contour2Segment &     contours2segment;
      };

   template <class EdgeContainer, class Contour2Segment>
   static void qualify(EdgeContainer const & edges, size_t idx, Contour2Segment & contours2segments)
   {
      typename EdgeContainer::value_type::data_type::setin_type const & setIn = edges[idx].data.setIn;

      if (edges[idx].data.dcelId == -1)
      {
         if (has_0(setIn) == In)
            for_all_set_greater_0(setIn, Inserter<Contour2Segment>(idx, contours2segments));
      }
      else
      {
         int N = edges[idx].data.dcelId;

         if (has_0(setIn) == In && has_N(setIn, N))
         {
            contours2segments[N].insert(idx);
         }
      }
   }

private:
   int dcelId;
};


typedef ManyByOneTraits<false> CutManyByOneTraits;
typedef ManyByOneTraits<true>  IntersectManyByOneTraits;

template <class Traits>
   struct FilteredCycleEdgesIterationTraits
{
   FilteredCycleEdgesIterationTraits(Traits const & traits) : traits_(traits) {}

   typedef ExitingEdgesIterationTraits Impl;

   template< class EdgeContainer >
      size_t next( EdgeContainer const &edges, size_t idx ) const 
   {
      size_t const start = edges[idx].twinEdge;

      for (size_t idx = Impl::prev(edges, start); idx != start; 
                  idx = Impl::prev(edges, idx))
      {
         if (traits_.is_mine(edges, idx))
            return idx;
      }

      return error();
   }

      struct no_suitable_exiting_edges_found : boost::exception {};

private:

   bool error() const
   {
      throw no_suitable_exiting_edges_found();
   }

private:
   Traits traits_;
};

//////////////////////////////////////////////////////////////////////////

//
// Cycles iteration
//

template< class EdgeContainer, class const_cycled_edge_iterator >
   class cycle_const_iterator
{
public:
   struct EdgeIteratorPair
   {
      const_cycled_edge_iterator begin, end;

      EdgeIteratorPair ()
      {
      }

      EdgeIteratorPair ( const_cycled_edge_iterator b, const_cycled_edge_iterator  e )
         : begin (b), end (e)
      {
      }
   };
   typedef EdgeIteratorPair value_type;

   cycle_const_iterator ()
      : edges_ (NULL), idx_ (static_cast< size_t >( -1 )), ignoreHoles_ (true)
   {
   }

   cycle_const_iterator ( EdgeContainer const *edges, bool ignoreHoles = true, bool end = false )
      : edges_ ((EdgeContainer *)edges), idx_ (end ? -1 : edges_->head()), ignoreHoles_ (ignoreHoles)
      , curIters_ (const_cycled_edge_iterator (edges, idx_, idx_ == -1),
                   const_cycled_edge_iterator (edges, idx_, true))
   {
      if (curIters_.begin != curIters_.end)
      {
         if (ignoreHoles_ && curIters_.begin->hole)
         {
            operator ++ ();         
         }
         else
            addCurCycleToIteratedEdges();
      }
   }

   value_type &operator * () const
   {
      return (value_type &)curIters_;
   }

   value_type *operator -> () const
   {
      return (&**this);
   }

   cycle_const_iterator &operator ++ ()
   {
      EdgesSet::const_iterator place;
      do 
      {
         idx_ = edges_->next(idx_);
         place = iteratedEdges_.end();
         if (idx_ != -1)
            place = iteratedEdges_.find(idx_);

      } while (idx_ != -1 && (place != iteratedEdges_.end() || ignoreHoles_ && (*edges_)[idx_].hole));

      curIters_ = EdgeIteratorPair (const_cycled_edge_iterator (edges_, idx_, idx_ == -1),
                                    const_cycled_edge_iterator (edges_, idx_, true));

      addCurCycleToIteratedEdges();

      if (idx_ == -1)
         iteratedEdges_.clear();

      return (*this);
   }

   cycle_const_iterator operator ++ ( int )
   {
      exiting_edge_const_iterator temp = *this;
      ++*this;
      return temp;
   }

   bool operator == ( cycle_const_iterator const &right ) const
   {
      return edges_ == right.edges_ && idx_ == right.idx_ && ignoreHoles_ == right.ignoreHoles_ &&
             iteratedEdges_.size() == right.iteratedEdges_.size();
   }

   bool operator != ( cycle_const_iterator const &right ) const
   {
      return !(*this == right);
   }

   void skip( size_t edgeIdx )
   {
      iteratedEdges_.insert(edgeIdx);
   }

   size_t index() const
   {
      return idx_;
   }

protected:
   EdgeContainer *edges_;
   size_t idx_;
   EdgeIteratorPair curIters_;
   bool ignoreHoles_;

   typedef std::set< size_t > EdgesSet;
   EdgesSet iteratedEdges_;

private:
   void addCurCycleToIteratedEdges()
   {
      for (const_cycled_edge_iterator it = curIters_.begin; it != curIters_.end; ++it)
      {
         size_t edgeIdx = &(*it) - &(*edges_)[0];
         if (edgeIdx >= edges_->containerSize())
            throw boost::enable_error_info(corrupted_exception ());

         std::pair< EdgesSet::iterator, bool > pr = iteratedEdges_.insert(edgeIdx);
         if (!pr.second)
            throw boost::enable_error_info(corrupted_exception ());
      }
   }
};

template< class EdgeContainer, class const_cycled_edge_iterator >
class cycle_const_iterator_ex
{
public:
   struct EdgeIteratorPair
   {
      const_cycled_edge_iterator begin, end;

      EdgeIteratorPair ()
      {
      }

      EdgeIteratorPair ( const_cycled_edge_iterator b, const_cycled_edge_iterator  e )
         : begin (b), end (e)
      {
      }
   };
   typedef EdgeIteratorPair value_type;

   cycle_const_iterator_ex ()
      : edges_ (NULL), idx_ (static_cast< size_t >( -1 )), ignoreHoles_ (true)
   {
   }

   cycle_const_iterator_ex ( EdgeContainer const *edges, bool ignoreHoles, bool end, 
      std::set<size_t> const & my_edges,
      boost::function<const_cycled_edge_iterator (EdgeContainer const *, size_t, bool)> factory
      )
      : edges_ ((EdgeContainer *)edges), idx_ (end ? -1 : *my_edges.begin()), ignoreHoles_ (ignoreHoles)
      , curIters_ (factory(edges, idx_, idx_ == -1), factory(edges, idx_, true)), factory_(factory), my_edges_(my_edges)
   {
      if (curIters_.begin != curIters_.end)
      {
         //if (ignoreHoles_ && curIters_.begin->hole)
         {
            operator ++ ();         
         }
         //else
         //   addCurCycleToIteratedEdges();
      }
   }

   value_type &operator * () const
   {
      return (value_type &)curIters_;
   }

   value_type *operator -> () const
   {
      return (&**this);
   }

   cycle_const_iterator_ex &operator ++ ()
   {
      EdgesSet::const_iterator place;

      if (!my_edges_.empty())
      {
         idx_ = *my_edges_.begin();
      }
      else
         idx_ = static_cast<size_t>(-1);

      curIters_ = EdgeIteratorPair (factory_(edges_, idx_, idx_ == -1),
         factory_(edges_, idx_, true));

      addCurCycleToIteratedEdges();

      if (idx_ == -1)
         iteratedEdges_.clear();

      return (*this);
   }

   cycle_const_iterator_ex operator ++ ( int )
   {
      exiting_edge_const_iterator temp = *this;
      ++*this;
      return temp;
   }

   bool operator == ( cycle_const_iterator_ex const &right ) const
   {
      return edges_ == right.edges_ && idx_ == right.idx_ && ignoreHoles_ == right.ignoreHoles_ &&
         iteratedEdges_.size() == right.iteratedEdges_.size();
   }

   bool operator != ( cycle_const_iterator_ex const &right ) const
   {
      return !(*this == right);
   }

   void skip( size_t edgeIdx )
   {
      iteratedEdges_.insert(edgeIdx);
   }

   size_t index() const
   {
      return idx_;
   }

protected:
   EdgeContainer *edges_;
   std::set<size_t> my_edges_;
   boost::function<const_cycled_edge_iterator (EdgeContainer const *, size_t, bool)>   factory_;
   size_t idx_;
   EdgeIteratorPair curIters_;
   bool ignoreHoles_;

   typedef std::set< size_t > EdgesSet;
   EdgesSet iteratedEdges_;

private:
   void addCurCycleToIteratedEdges()
   {
      for (const_cycled_edge_iterator it = curIters_.begin; it != curIters_.end; ++it)
      {
         size_t edgeIdx = &(*it) - &(*edges_)[0];
         if (edgeIdx >= edges_->containerSize())
            throw boost::enable_error_info(corrupted_exception ());

         std::pair< EdgesSet::iterator, bool > pr = iteratedEdges_.insert(edgeIdx);

         std::set<size_t>::iterator eit = my_edges_.find(edgeIdx);

         if (eit == my_edges_.end())
            throw boost::enable_error_info(corrupted_exception());

         my_edges_.erase(eit);
         if (!pr.second)
            throw boost::enable_error_info(corrupted_exception ());
      }
   }
};

template< class EdgeContainer, class cycled_edge_iterator, class const_cycled_edge_iterator >
   class cycle_iterator : public cycle_const_iterator< EdgeContainer, const_cycled_edge_iterator >
{
public:
   typedef cycle_const_iterator< EdgeContainer, const_cycled_edge_iterator > Base;
   struct EdgeIteratorPair
   {
      cycled_edge_iterator begin, end;

      EdgeIteratorPair ()
      {
      }

      EdgeIteratorPair ( cycled_edge_iterator b, cycled_edge_iterator e )
         : begin (b), end (e)
      {
      }
   };
   typedef EdgeIteratorPair value_type;

   cycle_iterator ()
   {
   }

   cycle_iterator ( EdgeContainer *edges, bool ignoreHoles = true, bool end = false )
      : Base (edges, ignoreHoles, end)
      , curIters_ (cycled_edge_iterator (edges_, idx_, idx_ == -1),
                   cycled_edge_iterator (edges_, idx_, true))
   {
   }

   value_type &operator * () const
   {
      return (value_type &)curIters_;
   }

   value_type *operator -> () const
   {
      return (&**this);
   }

   cycle_iterator &operator ++ ()
   {
      Base::operator ++ ();

      curIters_ = EdgeIteratorPair (cycled_edge_iterator (edges_, idx_, idx_ == -1),
                                    cycled_edge_iterator (edges_, idx_, true));

      return (*this);
   }

   cycle_iterator operator ++ ( int )
   {
      iterator temp = *this;
      ++*this;
      return temp;
   }

private:
   EdgeIteratorPair curIters_;
};

} // End of 'details' namespace
} // End of 'dcel' namespace
} // End of 'cg' namespace
