#pragma once

#include "geometry\cgal\enum.h"
#include "geometry\cgal\polygon_algorithm.h"
#include "geometry\cgal\std_traits.h"

#include "primitives\segment.h"
#include "segment_2_intersection.h"

namespace cg
{
    template <class Iterator, class Value>
	    struct input_iterator_impl 
		    : std::iterator<std::input_iterator_tag, Value>
    {
	    Iterator & operator ++ ()   { return derived().increment(); }
	    Iterator   operator ++ (int){ T temp(derived()); ++*this; return temp; }

	    friend bool operator != (
		    input_iterator_impl const &lhs,
		    input_iterator_impl const &rhs
		    )
	    {
		    return  
			    std::iterator<std::input_iterator_tag, Value>(lhs) !=
			    std::iterator<std::input_iterator_tag, Value>(rhs);
	    }

    protected:
	    Iterator & derived() 
	    { return *static_cast<Iterator*>(this); }

	    Iterator const & derived() const 
	    { return *static_cast<Iterator const *>(this); }
    };

    template 
    <
	    class Base,			// underlying iterator
	    class Iterator,		// concrete iterator derived from this iterator
	    class Value		// value_type for Iterator
    >
	    struct input_iterator_wrapper : input_iterator_impl<Iterator, Value>
    {
	    typedef Base base_iterator;

	    typedef input_iterator_wrapper Wrapper;

	    input_iterator_wrapper() {}

	    input_iterator_wrapper(base_iterator const &b) : p_(b) {}

	    Iterator& increment() { ++p_; return derived(); }

	    friend bool operator != (
		    input_iterator_wrapper const &a, 
		    input_iterator_wrapper const & b
		    )
	    { 
		    return a.p_ != b.p_; 
	    }
	    friend bool operator == (Wrapper a, Wrapper b){ return a.p_ == b.p_; }

    protected:
	    base_iterator       & base()       { return p_; }
	    base_iterator const & base() const { return p_; }
    private:
	    base_iterator	p_;
    };



    // тег, указывающий, что аргумент после вызова функции обнулится
    struct swapping {};

    struct polyline 
    {

	    typedef point_2 vertex;

	    // как перекрываются концы в массиве вершин, 
	    // который подает в конструктор пользователь
	    enum Overlapping { 
		    none,	// полигон не замкнут
		    closed, // первая точка совпадает с последней
		    first_edge   // первый сегмент совпадает с последним
	    };
        
        polyline() {}

        void swap(polyline & other) { v_.swap(other.v_); }
	    
	    polyline(std::vector<vertex> const &v, Overlapping overlapping)
		    :	v_(v)
	    {	make_complete(overlapping);	}

	    polyline(std::vector<vertex> &v, Overlapping overlapping, swapping)
	    {	std::swap(v_,v); make_complete(overlapping);	}
	    
	    polyline(polyline const &other) : v_(other.v_) {}

        polyline(polyline &other, swapping) { std::swap(other.v_,v_); }

	    template <class FwdIter>
		    polyline(FwdIter p, FwdIter q, Overlapping overlapping)
		    :	v_(p,q)
	    {	make_complete(overlapping); }

	    friend bool operator == (polyline const &a, polyline const &b) 
	    {	return a.v_ == b.v_;	}
		    

	    // requires: i in [0, size() + 2)
	    vertex const & operator[] (size_t i) const 
	    {	return v_.at(i); }

        // requires: i in [0, size() + 2)
        vertex      & operator[] (size_t i)       
        {	return v_.at(i); }


	    // количество вершин полилинии
	    size_t size() const 
	    {	return v_.size() - 2; }

	    bool empty() const 
	    {	return v_.empty(); }

	    // проверяет, что вершины должным образом замкнуты
	    void assert_constructed() const {
		    Assert(v_[0] == v_[size() + 0]);
		    Assert(v_[1] == v_[size() + 1]);
	    }

	    // --------------- доступ к вершинам. следует обращаться через vertices()
	    // Todo: можно попробовавть выделить класс Vertices
	    typedef 
		    std::vector<vertex>::iterator 
		    iterator;

	    typedef 
		    std::vector<vertex>::const_iterator 
		    const_iterator;

	    iterator  begin() { return v_.begin();	   }
	    iterator  end()   { return v_.end() - 2;   }
	    polyline& vertices() { return *this; }

	    const_iterator  begin() const { return v_.begin();	   }
	    const_iterator  end()   const { return v_.end() - 2;   }
	    polyline const& vertices() const { return *this; }
	    // --------------- доступ к вершинам.

	    typedef       iterator	     vertex_iterator;
	    typedef const_iterator const_vertex_iterator;

	    struct Edges {

		    Edges(polyline const &v) : v_(v) {}

		    struct const_iterator 
			    :	input_iterator_wrapper<const_vertex_iterator, const_iterator, segment_2> 
		    {
			    typedef input_iterator_wrapper<const_vertex_iterator, const_iterator, segment_2> Base;

			    const_iterator() {}
		    
			    const_iterator(const_vertex_iterator p) : Wrapper(p) {}

			    segment_2 operator * () const { return segment_2(base()[0],base()[1]); }

			    bool eq(const_iterator const &other) const
			    {	
				    return base() == other.base();
			    }

			    friend bool operator != (
				    const_iterator const &lhs,
				    const_iterator const &rhs
				    )
			    {
				    return lhs.base() != rhs.base();
			    }
		    };

		    const_iterator begin() const { return v_.vertices().begin(); }
		    const_iterator end  () const { return v_.vertices().end  (); }

	    private:
		    polyline const	&v_;
	    };

	    Edges const edges() const { return Edges(*this); }

    private:
	    std::vector<vertex>		v_;

	    // дополнить массив точками
	    void make_complete( Overlapping overlapping ) 
       {
		    switch (overlapping) 
          {
		    case closed:		
			    v_.push_back(v_[0]); 
		    case first_edge:	
			    v_.push_back(v_[1]);
			    break;
		    }
	    }
    };

    inline rectangle_2 bounding(polyline const &p) 
    {
	    return rectangle_2::bounding(&*p.vertices().begin(), &*p.vertices().end());
    }

    template <typename FwdIter>
        double distance (FwdIter first, FwdIter beyond, point_2 const & pt, bool is_closed = true)
    {
        double bestd = std::numeric_limits<double>::max();

        for (FwdIter v = first; v != beyond - 1; ++v) 
        {
            make_min(bestd, distance(segment_2(v[0], v[1]), pt));
        }

        if ( is_closed )
           make_min(bestd, distance(segment_2(first[0], beyond[-1]), pt) );

        return bestd;
    }


    // Distance - функционал, который оценивает расстояние до отрезка, 
    // концы которого заданы парой итераторов
    // double operator () (FwdIter edge_0, FwdIter edge_1)
    // TODO: На самом деле маразм играть на перегрузке Distance - point_2
    template <class FwdIter, class Distance>
        double distance(FwdIter first, FwdIter beyond, Distance const & distance)
    {
        double bestd = std::numeric_limits<double>::max();

        FwdIter p =   first;

        for (FwdIter q = ++first; q != beyond; ++p, ++q)
        {
            make_min(bestd, distance(p, q));
        }

        make_min(bestd, distance(first, p));

        return bestd;
    }

    template <class FwdIter, class Distance>
        double distance(FwdIter first, FwdIter beyond, Distance const & distance, point_2 & point)
    {
        double bestd = std::numeric_limits<double>::max();

        point_2 pt(0, 0);

        FwdIter p =   first;

        for (FwdIter q = ++first; q != beyond; ++p, ++q)
        {
            make_min ( bestd, distance(p, q, &pt), point, pt );
        }

        make_min ( bestd, distance(first, p, &pt), point, pt );

        return bestd;
    }

    inline double distance(polyline const &p, point_2 const &pt)
    {
        return distance(p.begin(), p.end() + 1, pt);
    }


    inline bool contains(std::vector<point_2> const &p, point_2 const &pt)
    {
        return 
            bounded_side_2(p.begin(), p.end() - 1, pt, StdTraits<point_2>()) 
            == ON_BOUNDED_SIDE;
    }

    inline bool contains(polyline const &p, point_2 const &pt)
    {
        return
            bounded_side_2(p.begin(), p.end() + 1, pt, StdTraits<point_2>())
            == ON_BOUNDED_SIDE;
    }

    template <class FwdIter, class Point>
        bool contains(FwdIter first, FwdIter again_first, Point const & pt)
    {
        Assert(*first == *again_first);

        return 
            bounded_side_2(first, again_first, pt, StdTraits<Point>())
            == ON_BOUNDED_SIDE;
    }

		template <class FwdIter>
			double signed_area(FwdIter first, FwdIter beyond)
		{
         if ( first == beyond )
            return 0;

			double acc = 0;
			for (FwdIter v = first; v != beyond - 1; ++v)
				acc += v[0] ^ v[1];

			acc += beyond[-1] ^ first[0];

			return acc * .5;
		}

    inline double area(polyline const &p) 
    {
        return cg::abs(signed_area(p.begin(), p.end() + 1));
    }

    template <class FwdIter>
        double area(FwdIter first, FwdIter again_first)
    {
        return cg::abs(signed_area(first, again_first));
    }
}
