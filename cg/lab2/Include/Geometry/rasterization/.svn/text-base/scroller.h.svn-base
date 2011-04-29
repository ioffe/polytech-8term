#pragma once

#include <boost\function.hpp>

#include <geometry\array_2d.h>

namespace cg
{

struct default_scroller_factory
{
   template<class S, class I>
      static void reset ( S&, point_2i const&, boost::scoped_ptr<I>& item ) 
      {
         item.reset(); 
      }

   template<class S, class T>
      static void reset ( S&, point_2i const&, T& item )
      {
         item = T() ; 
      }

   template<class S, class T>
      static void reset ( S&, point_2i const&, boost::optional<T>& item )
      {
         item.reset(); 
      }
} ; 

template<class W, class F = default_scroller_factory >
   struct scroller
   {
      typedef typename W::value_type value_type;

      scroller ( point_2i const& origin, W& window ) 
         : window_      (window)  
         , extents_     (window.extents())
         , rect_        (rectangle_by_extents(origin, window.extents()) )    
      { 
      }

      rectangle_2i const& rect () const 
      {
         return rect_ ; 
      }

      value_type const& at ( point_2i const & pos ) const
      {
         ensure ( pos ) ; 
         return window_[idx(pos)] ; 
      }

      value_type& at ( point_2i const & pos ) 
      {
         ensure ( pos ) ; 
         return window_[idx(pos)] ; 
      }

      value_type const& operator[] ( point_2i const & pos ) const
      {
         Assert ( rect_.contains(pos) ) ; 
         return window_[idx(pos)] ; 
      }

      value_type& operator[] ( point_2i const & pos ) 
      {
         Assert ( rect_.contains(pos) ) ; 
         return window_[idx(pos)] ; 
      }

      bool ensure ( point_2i const& pos ) 
      {
         if ( !rect_.contains(pos) ) 
         {
            set_rect ( offset( rect_, pos - rect_.closest_point(pos)) ); 
            return true ; 
         }
         return false ; 
      }

      bool ensure ( rectangle_2i const& rect ) 
      {
         if ( !rect_.contains(rect) ) 
         {
            set_rect ( offset( rect_, delta(rect_, rect) ) ) ; 
            return true ; 
         }
         return false ; 
      }

      bool set_center ( point_2i const& center )
      {
         if ( rect_.center() != center ) 
         {
            set_rect ( offset(rect_, center - rect_.center()) ) ; 
            return true ; 
         }
         return false ; 
      }

      void reset () 
      {
         reset(rect_); 
      }

   private:
      static range_2i subtract ( range_2i const& a, range_2i const& b )
      {
         Assert ( a.contains(b) ) ; 
         return b.hi() < a.hi() ? range_2i (b.hi() + 1, a.hi()) : 
                b.lo() > a.lo() ? range_2i (a.lo(), b.lo() - 1 ) :
                range_2f();
      }

      static rectangle_2i offset ( rectangle_2i const& rect, point_2i const& delta ) 
      {
         return rectangle_2i(rect).offset( delta ) ; 
      }

      static int delta ( range_2i const& a, range_2i const& b )
      {
         Assert ( a.size() >= b.size() ) ; 
         if ( b.lo() < a.lo() ) 
            return b.lo() - a.lo() ; 
         if ( b.hi() > a.hi() ) 
            return b.hi() - a.hi() ; 
         return 0 ; 
      }

      static point_2i delta ( rectangle_2i const& a, rectangle_2i const& b )
      {
         return point_2i ( delta(a.x, b.x), delta(a.y, b.y) ) ; 
      }

      point_2i idx ( point_2i const& p ) const
      {
         point_2i pp = p % extents_ ; 

         if ( pp.x < 0 ) pp.x += extents_.x; 
         if ( pp.y < 0 ) pp.y += extents_.y; 
         return pp ; 
      }

   private:
      rectangle_2i     rect_ ; 
      point_2i         extents_ ; 

      W&               window_ ;  

      void set_rect ( rectangle_2i const& rect )
      {
         Assert ( rect != rect_ ) ; 

         rectangle_2i intersection = rect_ & rect ; 

         if ( intersection.empty() ) 
            reset ( rect ); 
         else
         {
            range_2i xr = subtract ( rect.x, intersection.x ) ; 
            range_2i yr = subtract ( rect.y, intersection.y ) ; 

            if ( !xr.empty () ) 
               reset ( rectangle_2i(xr, rect.y) ); 

            if ( !yr.empty () ) 
               reset ( rectangle_2i(intersection.x, yr) ); 
         }

         rect_ = rect ; 
      }

      void reset ( rectangle_2i const& rect ) 
      {
         for ( rectangle_2i::iterator i = rect ; i ; ++ i ) 
            F::reset ( *this, *i, window_[idx(*i)] ) ; 
      }
   } ; 

template<class W>
   struct managed_scroller : scroller<W, managed_scroller<W> > 
   {
      typedef typename W::value_type value_type;
      typedef boost::function<void ( point_2i const& pos, value_type& item ) > reset_function_t ;

      managed_scroller ( point_2i const& origin, W& window, reset_function_t reset )
         : scroller ( origin, window ) 
         , reset_ ( reset ) 
      {
         scroller::reset() ; 
      }

   private:
      reset_function_t reset_ ; 

      typedef scroller<W, managed_scroller> scroller ; 
      friend scroller ; 

      static void reset(scroller& self, point_2i const& pos, value_type& item )
      {
         static_cast<managed_scroller&>(self).reset_(pos, item) ; 
      }
   } ; 

template<class T>
   struct scrolling_array_window 
   {
      cg::array_2d<T> storage ; 

      scrolling_array_window ( point_2i const& extents ) : storage(extents) {}

   } ; 

template<class T, class F = default_scroller_factory>
   struct scrolling_array : private scrolling_array_window<T>, scroller<cg::array_2d<T>, F>
   {
      scrolling_array ( point_2i const& origin, point_2i const& extents )
         : window   (extents) 
         , scroller (origin, storage) 
      {
      }

      scrolling_array ( rectangle_2i const& rect )
         : window   (extents_by_size(rect.size())) 
         , scroller (rect.lo(), storage) 
      {
      }

      mem_usage get_mem_usage () const 
      {
         return ::get_mem_usage(storage) ; 
      }

   private:
      typedef scrolling_array_window<T> window ; 
      typedef scroller<cg::array_2d<T>,F> scroller ; 
   } ; 

template<class T>
   struct managed_scrolling_array : private scrolling_array_window<T>, managed_scroller<cg::array_2d<T> >
   {
      managed_scrolling_array ( point_2i const& origin, point_2i const& extents, reset_function_t reset )
         : window   (extents) 
         , scroller (origin, storage, reset) 
      {
      }

      managed_scrolling_array ( rectangle_2i const& rect, reset_function_t reset )
         : window   (extents_by_size(rect.size())) 
         , scroller (rect.lo(), storage, reset) 
      {
      }

      mem_usage get_mem_usage () const 
      {
         return ::get_mem_usage(storage) ; 
      }

   private:
      typedef scrolling_array_window<T> window ; 
      typedef managed_scroller<cg::array_2d<T>> scroller ; 
   } ; 

}
