#pragma once

#include "simplify_polyline.h"
#include "common/util.h"
#include <boost/iterator/iterator_adaptor.hpp>

namespace cg 
{

   namespace algos 
   {
      namespace details
      {
         template<class Iterator>
         struct proxy_iterator_t 
            :  boost::iterator_adaptor<proxy_iterator_t<Iterator>, Iterator, boost::use_default, boost::bidirectional_traversal_tag>
         {
            proxy_iterator_t( Iterator p, Iterator q, Iterator it )
               : proxy_iterator_t::iterator_adaptor_(it)
               , p_ (p), q_(q)
            {}

            proxy_iterator_t()
            {}

         private:
            friend class boost::iterator_core_access;

            void increment()
            { 
               ++base_reference();
               if (base() == q_)
                  base_reference() = p_;
            }

            void decrement()
            { 
               if (base() == p_)
                  base_reference() = q_;
               
               --base_reference();
            }

         private:
            Iterator p_;
            Iterator q_;
         };

         template<class Iterator>
         struct cycled_iterator_t 
            :  boost::iterator_adaptor<cycled_iterator_t<Iterator>, Iterator, boost::use_default, boost::bidirectional_traversal_tag>
         {
            cycled_iterator_t( Iterator p, bool end )
               : cycled_iterator_t::iterator_adaptor_( p )
               , p_ (p), end_(end)
            {}

            cycled_iterator_t() {}

         private:
            friend class boost::iterator_core_access;

            void increment()
            {
               ++base_reference();

               if (base() == p_ && end_ == false)
                  end_ = true;
            }

            void decrement()
            {
               if (base() == p_ && end_ == true)
                  end_ = false;

               --base_reference();
            }

            bool equal(cycled_iterator_t<Iterator> const& other) const
            {
               return base() == other.base() && end_ == other.end_;
            }

         private:
            Iterator p_;
            bool end_;
         };

         template<class OutIterator>
         struct output_base_iterator_t
            : std::iterator<std::output_iterator_tag, void, void, void, void>
         {
            output_base_iterator_t( OutIterator & out ) 
               : out_( out )
            {}

            template<class BaseIterator>
            output_base_iterator_t & operator=( BaseIterator const& it )
            {
               *out_++ = it.base();
               return *this;
            }
            output_base_iterator_t & operator*() { return *this; }
            output_base_iterator_t & operator++() { return *this; }
            output_base_iterator_t & operator++(int) { return *this; }

         private:
            OutIterator out_;
         };


         template<class OutIterator, class ResultFunc>
         struct out_iterator_filter_first_t
            : std::iterator<std::output_iterator_tag, void, void, void, void>
         {
            out_iterator_filter_first_t( OutIterator & out, ResultFunc const& base_res_func ) 
               : out_( out )
               , base_res_func_( base_res_func ) 
               , first_( true )
            {}

            template<class BaseIterator>
            out_iterator_filter_first_t & operator=( BaseIterator const& it )
            {
               if (!first_)
                  *out_++ = base_res_func_(it);
               else
                  first_ = false;

               return *this;
            }
            out_iterator_filter_first_t & operator*() { return *this; }
            out_iterator_filter_first_t & operator++() { return *this; }
            out_iterator_filter_first_t & operator++(int) { return *this; }

         private:
            ResultFunc base_res_func_;
            OutIterator out_;
            bool first_;
         };

         template<class Iterator>
         struct iterator_result_func_t
         {
            Iterator const& operator()( Iterator const& it ) const { return it; }
         };
      }

      template < class BiDirIter, class OutIter >
      OutIter simplify_contour_points ( BiDirIter p, BiDirIter q, double prec, OutIter output )
      {
         PointFunctor< BiDirIter > func ;
         return simplify_contour( p, q, prec, output, func ) ;
      }

      template < class BiDirIter, class OutIter >
      OutIter simplify_contour_indexes ( BiDirIter p, BiDirIter q, double prec, OutIter output )
      {
         IndexFunctor< BiDirIter > func ( p ) ;
         return simplify_contour( p, q, prec, output, func ) ;
      }


      template < class BiDirIter, class OutIter, template < class InIter > class ResultFunc  >
      OutIter simplify_contour ( BiDirIter p, BiDirIter q, double prec, OutIter output, ResultFunc < BiDirIter > const& res_func )
      {
         // разбиваем контур на две полилинии по точкам, имеющим макс и мин координату x 
         // (вообще чтобы честно необходимо посчитать диаметр и разбить по диаметрально противоположным точкам)
         // затем упрощаем отдельно верхнюю и нижнюю цепи

         if (std::distance(p, q) < 3)
         {
           for (BiDirIter it = p; it != q; ++it)
             *output++ = res_func(it);
           return output;
         }

         if (*p == *util::prev(q))
            q--;

         BiDirIter min = p, max = util::prev(q);

         for (BiDirIter it = p; it != q; ++it)
         {
            if (it->x < min->x)
               min = it;
            if (it->x > max->x)
               max = it;
         }

         typedef details::proxy_iterator_t<BiDirIter> proxy_iterator_type;
         typedef details::cycled_iterator_t<proxy_iterator_type> cycled_iterator_type;


         typedef details::out_iterator_filter_first_t<OutIter, ResultFunc<BiDirIter>> out_iterator_filter_first_type;
         typedef details::output_base_iterator_t<out_iterator_filter_first_type> output_base1_iterator_type;
         typedef details::output_base_iterator_t<output_base1_iterator_type>     output_base2_iterator_type;

         proxy_iterator_type min_proxy(p, q, min);
         proxy_iterator_type max_proxy(p, q, max);

         out_iterator_filter_first_type out_proxy(output, res_func);

         if (min_proxy == util::next(max_proxy))
         {
            output_base2_iterator_type out((output_base1_iterator_type)(out_proxy));
            simplify_polyline(cycled_iterator_type(min_proxy, false), cycled_iterator_type(min_proxy, true), prec, out, details::iterator_result_func_t<cycled_iterator_type>());
            *output++ = res_func(min_proxy.base());
         }
         else if (max_proxy == util::next(min_proxy))
         {
            output_base2_iterator_type out((output_base1_iterator_type)(out_proxy));
            simplify_polyline(cycled_iterator_type(max_proxy, false), cycled_iterator_type(max_proxy, true), prec, out, details::iterator_result_func_t<cycled_iterator_type>());
            *output++ = res_func(max_proxy.base());
         }
         else
         {
            output_base1_iterator_type out(out_proxy);
            simplify_polyline(min_proxy, util::next(max_proxy), prec, out, details::iterator_result_func_t<proxy_iterator_type>());
            simplify_polyline(max_proxy, util::next(min_proxy), prec, out, details::iterator_result_func_t<proxy_iterator_type>());
         }

         return output;
      }

   }
}
