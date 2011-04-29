#pragma once

#include "contours\shootray\processor\processor_functions.h"
#include "contours\shootray\state\state.h"

#include "Geometry/grid2L.h"

namespace cg
{
    namespace qualifying_contours
    {
        template <class FwdIter>
            static FwdIter prev (FwdIter p) 
        {
            return --p;
        }

        template <class FwdIter>
            static FwdIter next (FwdIter p) 
        {
            return ++p;
        }

        template <class Traits, class Grid, class Reporter>
            struct BoundsAreSeparated
                :   grid2l_visitor_base < Grid, BoundsAreSeparated<Traits, Grid, Reporter> >
        {
            typedef typename smallcell_type::Segments::const_iterator  SCI;
            typedef typename Traits::contour_id   contour_id;
            typedef typename Traits::segment_id   segment_id;

            BoundsAreSeparated (Traits const & traits, Reporter & reporter) 
                : traits_ (traits), reporter_ (reporter) {}

            template <class State>
                bool operator () (State const &, smallcell_type const & scell) const
            {
                checkCell(scell.segments().begin(), scell.segments().end());
                return false;
            }

        private:

            template <class FwdIter>
                void checkCell (FwdIter p, FwdIter q) const
            {
                if (p != q)
                {
                    for (; p != prev(q); ++p)
                    {
//                        segment_2 const & sA = traits_.getSegment(*p);
                        segment_2 sA = traits_.getSegment(*p);
                        contour_id cA = traits_.getContourBySegment(*p);

                        for (FwdIter r = next(p); r != q; ++r)
                        {
//                            segment_2 const & sB = traits_.getSegment(*r);
                            segment_2 sB = traits_.getSegment(*r);
                            contour_id cB = traits_.getContourBySegment(*r);

                            point_2 intersect1;
                            point_2 intersect2;

                            cg::intersection_type intersect_tp = 
                                cg::generic_intersection(sA, sB, &intersect1, &intersect2);

                            if ( intersect_tp == cg::disjoint )
                                continue;
                            if ( intersect_tp == cg::intersect ) 
                            {
                                // два отрезка пересеклись
                                if ( cg::prs::point_inside_segment ( sA, intersect1 ) &&
                                        cg::prs::point_inside_segment ( sB, intersect1 ) ) 
                                {
                                    reporter_.OnContoursIntersect(cA, cB, sA, sB, intersect1);
                                    continue;
                                }

                                // один отрезок крайней точкой лежит на другом
                                if ( cg::prs::point_inside_segment ( sA, intersect1 ) )
                                {
                                    // sB лежит точкой на отрезке sA
                                    // на отрезке sA (p) лежит sB (r) 
                                    checkIntersectionPS ( p, r, intersect1, 
                                        cg::prs::is_prev_point ( sB, intersect1 ) ); 
                                    continue;
                                }

                                if ( cg::prs::point_inside_segment ( sB, intersect1 ) )
                                {
                                    // sA лежит точкой на отрезке sB
                                    // на отрезке sB (r) лежит sA (p) 
                                    checkIntersectionPS ( r, p, intersect1, 
                                        cg::prs::is_prev_point ( sA, intersect1 ) ); 
                                    continue;
                                }

                                // два отрезка пересекаются по точке
                                if ( cA != cB ) {
                                    checkIntersectionPP ( p, r, intersect1,
                                        cg::prs::is_prev_point ( sA, intersect1 ),
                                        cg::prs::is_prev_point ( sB, intersect1 ) );
                                }

                                continue;
                            }

                            // два отрезка пересекаются так, что образуется новый отрезок
                            // если эти отрезки сонаправлены, то контуры пересекаются...
                            // а условие неравенства контуров 
                            if ( intersect_tp == cg::overlap 
                                && ( cA == cB 
                                    || ( cA != cB && cg::prs::is_codirectional ( sA, sB ) 
                                        && (   cg::prs::point_inside_segment ( sA, intersect1 ) 
                                            || cg::prs::point_inside_segment ( sA, intersect2 ) 
                                            ) 
                                        ) 
                                    ) 
                                ) 
                            {
                                reporter_.OnContoursOverlap ( 
                                    cA, cB, sA, sB, intersect1, intersect2 );
                                continue;
                            }
                        }
                    }
                }
            }

            template <class FwdIter>
                void checkIntersectionPS ( FwdIter segA, FwdIter segB, point_2 intersect, bool prev ) const
                // на segA лежит крайней точкой segB
            {
                segment_id segA_id = *segA;
                segment_id segB_id = *segB;

                segment_2 const & segA_s = traits_.getSegment(segA_id);
                segment_2 const & segB_s = traits_.getSegment(segB_id);

                point_2 segB_p = prev ? segB_s.P1() : segB_s.P0();

                // теперь если конец segB лежит с внешней стороны отрезка segA,
                // то все в порядке
                if  ( cg::prs::is_outside ( segB_p, segA_s ) ) 
                {
                    return;
                }

                reporter_.OnContoursIntersect( traits_.getContourBySegment ( segA_id ), 
                    traits_.getContourBySegment ( segB_id ), 
                    segA_s, segB_s, intersect);
            }

            template <class FwdIter>
                void checkIntersectionPP ( FwdIter segA, FwdIter segB, point_2 intersect, bool prevA, bool prevB ) const
            {
                segment_id segA_id  = *segA;
                segment_id segB_id  = *segB;
                segment_id segNA_id = traits_.getAdjacentSegment ( segA_id, prevA );
                segment_id segNB_id = traits_.getAdjacentSegment ( segB_id, prevB );

                segment_2 const & segA_s  = traits_.getSegment ( segA_id );
                segment_2 const & segB_s  = traits_.getSegment ( segB_id );
                segment_2 const & segNA_s = traits_.getSegment ( segNA_id );
                segment_2 const & segNB_s = traits_.getSegment ( segNB_id );

                point_2 segA_p  = prevA ? segA_s.P1()  : segA_s.P0();
                point_2 segB_p  = prevB ? segB_s.P1()  : segB_s.P0();
                point_2 segNA_p = prevA ? segNA_s.P0() : segNA_s.P1();
                point_2 segNB_p = prevB ? segNB_s.P0() : segNB_s.P1();

                // случаи:
                //  1. один контур содержит другой
                //  2. контуры не пересекаются
                //  3. контуры частично пересекаются
                //  если не 2, то 1 или 3 !!! Логика!!! :-)
                //  а 2 -- это когда обе пары точек лежат снаружи от соотв. контуров !!!
                //  т.о. кидать ассерт, когда нарушается это условие!

                bool term2 = true;

                term2 = term2 && ( cg::prs::is_outside ( segA_p,  segB_s  ) || cg::prs::is_outside ( segA_p,  segNB_s ) );
                term2 = term2 && ( cg::prs::is_outside ( segNA_p, segB_s  ) || cg::prs::is_outside ( segNA_p, segNB_s ) );
                term2 = term2 && ( cg::prs::is_outside ( segB_p,  segA_s  ) || cg::prs::is_outside ( segB_p,  segNA_s ) );
                term2 = term2 && ( cg::prs::is_outside ( segNB_p, segA_s  ) || cg::prs::is_outside ( segNB_p, segNA_s ) );

                if (!term2)
                {
                    reporter_.OnContoursIntersect( traits_.getContourBySegment ( segA_id ), 
                                                    traits_.getContourBySegment ( segB_id ), 
                                                    segA_s, segB_s, intersect);
                }
            }


        private:
            Traits const & traits_;
            Reporter     & reporter_;
        };

        template <class FwdIter, class Traits, class Reporter>
            void checkContoursDontNested (Traits const & traits, 
                FwdIter p, FwdIter q, Reporter & reporter) 
        {
            // for every contour
            for (; p != q; ++p)
            {
                checkContourIsntNested(traits, traits.pointsBegin(p), reporter);
            }
        }

        template <class FwdIter, class Traits, class Reporter>
            void checkContourIsntNested(Traits const & traits, FwdIter p, Reporter & reporter) 
        {
            typedef typename Traits::segment_id segment_id;

            // shoot ray for the "min" point
            point_2   from = traits.getPoint(p); //from.x -= 100 * epsilon;
            point_2   to   = point_2(traits.grid().origin().x, traits.getPoint(p).y);

            // бростиь луч и посмотреть, с кем и как он пересечется
            segment_2 ray (from, to);
            segment_id segId = traits.shootRay < ShootRay2DSimpleProcessor < Traits, cg::State < segment_id > > > (ray, traits.defaultSegmentId(), traits.getContourByPoint(p));

            if (segId == traits.defaultSegmentId()) 
                return;

            if ( cg::prs::is_outside( ray.P0(), traits.getSegment(segId) ) )
                return;

            reporter.OnContoursAreNested(
                traits.getContourByPoint(p), 
                traits.getContourBySegment(segId));
        }
        
    }

    // Проверяет, что границы контуров не пересекаются
    // Контура растеризованы в грид
    template <class Traits, class Grid, class Reporter>
        void checkContoursDontIntersect (
            Traits const & traits, Grid const & grid, Reporter & reporter)
    {
        qualifying_contours::BoundsAreSeparated <Traits, Grid, Reporter> 
            processor(traits, reporter);

        visit_every_cell(const_cast<Grid&>(grid), processor);

        qualifying_contours::checkContoursDontNested(
            traits, traits.contoursBegin(), traits.contoursEnd(), reporter);
    }

}