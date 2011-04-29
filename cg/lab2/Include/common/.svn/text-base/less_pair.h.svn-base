#pragma once

struct less_pair
{
   template<class F, class S> bool operator() ( std::pair<F,S> const& p1, std::pair<F,S> const& p2 ) const
   {
      return p1.first < p2.first ; 
   }
   template<class F, class S> bool operator() ( std::pair<F,S> const& p1, F const& time ) const
   {
      return p1.first < time ; 
   }
   template<class F, class S> bool operator() ( F const& time, std::pair<F,S> const& p2 ) const
   {
      return time < p2.first ; 
   }
} ; 
