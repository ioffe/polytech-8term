namespace cg            {
namespace skeleton      {
namespace shootraying   {

   template < class Derived, class Grid >
      struct FindAllIntersections
   {
      template < class OutIter >
         size_t intersectSegment( cg::segment_2 seg, OutIter out ) const
      {
         IntersectionsProcessor proc( self(), seg );
         Grid const & grid = self().getCdt().grid();
         cg::visit( grid, seg, proc );

         proc.sort_isections();
         std::copy( proc.isections().begin(), proc.isections().end(), out );

         return proc.isections().size();
      }

   private:
      Derived const & self() const { return static_cast< Derived const & >( *this ); }

      struct IntersectionsProcessor
         : cg::grid2l_visitor_base< Grid, IntersectionsProcessor >
      {
         typedef std::vector< contours::contour_id > isections_type;

         IntersectionsProcessor( Derived const & cdt, cg::segment_2 const & seg ) 
            : seg_( seg )
            , cdt_( cdt )
         {}

         template < class State >
            bool operator ()( State &, smallcell_type &scell )
         {
            for ( smallcell_type::Segments::const_iterator it = scell.segments().begin();
               it != scell.segments().end(); ++it )
            {
               cg::segment_2 const & seg = cdt_.getSegment( *it );
               cg::intersection_type pt_type = cg::generic_intersection( seg_, seg );

               if ( pt_type == cg::intersect )
               {
                  isections_.push_back( cdt_.getContourBySegment( *it ) );
               }
            }

            return false;
         }

         isections_type const & isections()
         {
            return isections_;
         }

         void sort_isections() 
         {
            std::sort( isections_.begin(), isections_.end() );
            isections_.erase( std::unique( isections_.begin(), isections_.end() ), isections_.end() );
         }

      private:
         cg::segment_2     seg_;
         isections_type    isections_;
         Derived const &   cdt_;
      };
   };

}}}
