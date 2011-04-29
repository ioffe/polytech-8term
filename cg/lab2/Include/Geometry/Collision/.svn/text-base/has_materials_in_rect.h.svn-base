#pragma   once

#include "dataview/material_handles.h"

namespace cg
{
    namespace cdt
    {
        template <class Grid, class Traits, class Derived>
            struct has_materials_in_rect
        {
            typedef typename Traits::face_id face_id;
            typedef std::vector< materials::texture_handle > materials_type;

            bool hasMaterials(rectangle_2 const &rect, materials_type mat) const
            {
                sort( mat.begin(), mat.end() );

                Processor processor(self().traits(), mat);
                visit(grid(), rect, processor);

                return processor.result();
            }

        private:
            Derived const & self () const { return static_cast<Derived const &>(*this); }
            Grid & grid() const { return const_cast<Grid&>(self().grid()); }

            struct Processor
               : grid2l_visitor_base<Grid, Processor>
            { 
                Processor (Traits const & traits, materials_type const & mat)
                    : traits_( traits )
                    , mat_( mat )
                    , res_( false )
                {}

                template < class State >
                    bool operator () (State &, typename Grid::smallcell_type const & column)
                {
                    if ( column )
                    {
                        for ( DWORD i=0; i < column->faces().size(); i++ )
                        {
                            face_id tr_index = column->faces()[i];
                            if ( std::find( mat_.begin(), mat_.end(), traits_.getMaterial( tr_index ) ) != mat_.end() )
                            {
                                res_ = true;
                                return true;
                            }
                        }
                    }
                    return false;
                }

                bool result() const { return res_; }

            private:
                Traits const &         traits_;
                materials_type const & mat_;
                bool                   res_;
            };
        };
    }
}