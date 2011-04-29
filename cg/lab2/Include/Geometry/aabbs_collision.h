#pragma once

namespace cg
{
   namespace details
   {
      template< typename rectangle_type, typename index_type >
      struct aabb_rasterization_grid 
         : private boost::noncopyable
      {
         // Тип ячейки в сетке
         struct cell_type
         {
            cell_type() {}

            typedef  std::vector< index_type > indices_type;

            indices_type  indices;
            range_2       zrange;
         };

         // Тип сетки
         typedef Grid1L< cell_type > grid_type;

      private:
         struct rasterization_processor_type
         {
            rasterization_processor_type ( index_type idx, range_2 const & zrange )  
               : idx_( idx )
               , zrange_( zrange )
            {}

            template< class State >
            bool operator() ( State const & state, cell_type & cell )
            {
               cell.indices.push_back ( idx_ );
               cell.zrange |= zrange_ ;

               return false;
            }

         private:
            index_type  idx_;
            range_2     zrange_;
         };

      public:
         aabb_rasterization_grid( rectangle_2 bb, point_2i const & ext, rectangle_type const* aabbs, size_t count )
         {
            bb.inflate(cg::epsilon<float>()) ;
            point_2 const unit = bb.size() / ext ;
            grid_ = new grid_type(grid_params(bb.xy(), unit, ext)) ;

            for (size_t i = 0; i < count; ++i)
               visit(*grid_, aabbs[i], rasterization_processor_type(i, aabbs[i][2])) ;
         }

      public:
         grid_type const & grid() const { return *grid_; }

      private:
         m_ptr< grid_type > grid_;
      };

   } // end of namespace details

   // Параметры для растеризации
   struct rasterized_aabb_params
   {
      rasterized_aabb_params( int minAabbs, point_2i const & ext )
         : minAabbs_( minAabbs )
         , ext_( ext )
      {}

      size_t            min_aabbs() const { return minAabbs_; } 
      point_2i const &  extents()       const { return ext_; }

   private:
      size_t   minAabbs_;
      point_2i ext_;            
   };

   template< typename rectangle_type, typename index_type >
   struct rasterized_aabbs
   {
   private:
      typedef typename details::aabb_rasterization_grid< rectangle_type, index_type >  raster_type;

   public:
      typedef typename raster_type :: grid_type                            grid_type;
      typedef std::vector< char >                                          cache_type;

   public:
      rasterized_aabbs( rasterized_aabb_params const & params, rectangle_3 const * bb = NULL )
         : params_      ( params )
         , aabbsCount_  ( 0 )
         , aabbs_       ( NULL )
      { 
         if (bb)
            bbMax_.reset(*bb) ;
      }

      void clean_cache() const
      {
         if (!cache_.empty())
            memset(&cache_[0], 0, cache_.size()) ;
      }

      void set( rectangle_type const* aabbs, size_t aabbsCount )
      {
         aabbsCount_ = aabbsCount ;
         aabbs_ = aabbs ;

         bb_ = rectangle_3() ;
         for (size_t i = 0 ;i < aabbsCount_; ++i)
            bb_ |= aabbs_[i] ;
         if ( bbMax_ )
            bb_ &= *bbMax_ ;

         if (aabbsCount_ >= params_.min_aabbs())
         {
            raster_.reset(new raster_type(bb_, params_.extents(), aabbs_, aabbsCount_)) ;
            cache_.resize(aabbsCount_);
         }
         else
            raster_.reset() ;
      }

      void clear()
      {
         aabbsCount_ = 0 ;
         raster_.reset() ;
         aabbs_ = NULL ;
      }

      cache_type           &  cache() const { return cache_; }
      grid_type      const *  grid()  const { return raster_ ? &raster_->grid() : 0; } 
      rectangle_3    const &  bb()    const { return bb_; }

      rectangle_type const*   aabbs()       const { return aabbs_ ; }
      size_t                  aabbs_count() const { return aabbsCount_ ; }

   private: 
      rasterized_aabb_params const         params_ ;
      rectangle_type const*                aabbs_ ;
      size_t                               aabbsCount_ ;
      boost::shared_ptr< raster_type >     raster_;

      boost::optional< rectangle_3 >       bbMax_;
      rectangle_3                          bb_;
      mutable cache_type                   cache_;
   };

   namespace details
   {
      template< typename rectangle_type, typename index_type, typename processor_type >
      struct visit_aabb_processor
      {
         typedef typename rasterized_aabbs< rectangle_type, index_type > rasterized_aabbs_type ;

         visit_aabb_processor ( rectangle_t<typename rectangle_type::scalar_t, rectangle_type::dimension> const &aabb, 
            rasterized_aabbs_type const &aabbs, processor_type & proc )
            : proc_  ( proc )
            , aabbs_ ( aabbs )
            , zrange_( aabb[2] )
         {
            aabbs_.clean_cache() ;
         }

      private:
         typedef typename rasterized_aabbs_type::grid_type::cell_type cell_type;
         typedef typename rasterized_aabbs_type::cache_type           cache_type;

      public:
         template< class State > 
         bool operator() ( State const & state, cell_type const & cell )
         {
            typedef typename cell_type::indices_type::const_iterator CII;

            if ( !has_intersection( zrange_, cell.zrange ) )
               return false;

            cache_type      &cache = aabbs_.cache();

            for ( CII p = cell.indices.begin(), q = cell.indices.end(); p != q; ++p )
            {
               if ( cache[*p] )
                  continue;

               cache[*p] = true;

               if (process(*p))
                  return true ;
            }

            return false;
         }

         bool process( index_type i )
         {
            return proc_(i) ;
         }

      private:
         processor_type &             proc_ ;
         rasterized_aabbs_type const &aabbs_ ;
         cg::range_2                  zrange_ ;
      };
   }

   template< typename rectangle_type, typename index_type, typename processor_type >
   void visit_rasterized_aabbs( /*cg::rectangle_t<typename rectangle_type::scalar_type, rectangle_type::dimension>*/ rectangle_3 const& rect, rasterized_aabbs< rectangle_type, index_type > const &aabbs, processor_type & proc )
   {
      typedef details::visit_aabb_processor< rectangle_type, index_type, processor_type > visit_aabb_processor_type ;
      visit_aabb_processor_type visit_proc(rect, aabbs, proc) ;

      if (aabbs.grid())
      {
         visit(*aabbs.grid(), rect, visit_proc) ;
      }
      else
      {
         // Process all triangles
         for (size_t i = 0; i < aabbs.aabbs_count(); ++i)
         {
            visit_proc.process(i) ;  
         }
      }
   }

}