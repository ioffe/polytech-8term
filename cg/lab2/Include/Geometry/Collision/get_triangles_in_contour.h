#pragma   once

#include "Geometry/Triangulation/cgal_triangulation.h"

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct get_triangles_in_contour
        {
            typedef typename Traits::face_id face_id;

            void getTrianglesInContour( point_2 const *contour, DWORD contour_size, 
                                        cg::range_2 const* zrange, std::vector<face_id> &triangles ) const
            {
                typedef cg::triangulation::cgal_triangulation<> trg_type;

                trg_type trg;
                add_contour( trg, contour, contour + contour_size );

                std::vector<char> used( self().traits().getFaceNum(), 0 );

                Processor processor( triangles, used, zrange );
                for ( trg_type::faces_iterator it = trg.faces_begin(); it != trg.faces_end(); ++it )
                    visit( grid(), trg.construct( it ), processor );
            }

            void getTrianglesInRect( rectangle_2 const& rect, range_2 const* zrange, std::vector<face_id> &triangles ) const
            {
               std::vector<char> used( self().traits().getFaceNum() );

               Processor processor( triangles, used, zrange );
               visit( grid(), rect, processor );
            }

        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }
            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            // ToDo: Вообще говоря, клиповать поверхность полигоном лучше
            //       не в c3d, а структуре данных, которая поддерживает 
            //       быстрый переход к соседнему треугольнику.
            struct Processor : grid2l_visitor_base<Grid, Processor>
            { 
                Processor (std::vector<face_id> &triangles, std::vector<char> & used, range_2 const * zrange)
                    : triangles_    ( triangles )
                    , used_         ( used      )
                    , zrange_       ( zrange    )
                {}

                typedef Processor SideProcessor;

                SideProcessor & side_processor(int, int) {
                    return *this;
                }

                template <class State>
                    bool operator () (State &, typename Grid::smallcell_type const & column)
                {
                    if (column && (zrange_ == 0 || has_intersection(*zrange_, column->zrange())))
                    {
                        // добавляем все треугольники, попавшие в эту ячейку, в результат
                        // при этом надо избежать дублирования треугольников
                        for (unsigned i = 0, size = column->faces().size(); i != size; ++i)
                        {
                            face_id tr_index = column->faces()[i];
                            Assert( tr_index >= 0 && size_t(tr_index) < used_.size() );

                            if ( !used_[tr_index] )
                            {
                                triangles_.push_back( tr_index );
                                used_[tr_index] = 1;
                            }
                        }
                    }
                    return false;
                }

            private:
                std::vector<face_id>  & triangles_;
                std::vector<char>     & used_;
                range_2         const * zrange_;
            };

       };
    }
}