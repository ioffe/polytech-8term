#pragma once

#include "classify_pt_on_rect.h"



namespace cg
{
    // polygon is supposed to be clockwise
    template <class Traits>
        struct PolyByRectClipper
    {
        typedef typename Traits::comparer_type   Cmp;
        typedef typename Traits::point_id        point_id;

        // ћы копируем в себ€ точки. ћожно было бы этого не делать
        typedef std::vector<point_id>   Line;   // это одна ломанна€. как правило, она незамкнута€
        typedef std::list  <Line>       Lines;

        void addLine (Traits const & traits, point_id const & pt0, point_id const & pt1);

        void newLine()
        {
            lines_.push_back(Line());
        }

        void add(point_id const & pt)
        {
            lines_.back().push_back(pt);
        }

        bool closeFirstAndLastLines();

        template <class OutputIterator>
            void extractInnerPolyline (OutputIterator out); 

        bool empty() const { return lines_.empty(); }

        Lines const & lines() const { return lines_; }

        template <class FwdIter>
            PolyByRectClipper (FwdIter first, FwdIter beyond)
        {
            for (FwdIter p = first; p != beyond; ++p)
            {
                lines_.push_back(Line());
                std::copy(p->begin(), p->end(), std::back_inserter(lines_.back()));
            }
        }

        PolyByRectClipper () {}

        PolyByRectClipper const & segments() const { return *this; }
        PolyByRectClipper       & segments()       { return *this; }

        // итераци€ по всем отрезкам
        template <class Func>
            void for_each(Func & f) const
        {
            for (Lines::const_iterator l_it = lines_.begin(); l_it != lines_.end(); ++l_it)
            {
                Line::const_iterator p = l_it->begin();

                do
                {
                    point_id p_0 = *p;

                    if (++p == l_it->end())
                        break;

                    point_id p_1 = *p;

                    f(p_0, p_1);

                } while (true);
            }
        }

        template <class OutputIterator>
            void extractPolyline (Traits const & traits, rectangle_2 const & rc, OutputIterator);

        template <class OutputIterator>
            void extractPolylineGeneric(Traits const & traits, rectangle_2 const & rc, OutputIterator out)
        {
            if (sole(traits, rc)) 
            {
                    extractInnerPolyline(out);
            }
            else
            {
                if (!lines_.empty())
                {
                    extractPolyline(traits, rc, out);
                }
            }
        }

        bool sole(Traits const & traits, rectangle_2 const & rc) const 
        {
            return 
                lines_.size() == 1 && traits.comparer(rc).is_undefined(lines_.front().front());
        }

    private:
        bool is_new_line (Traits const & traits, point_id const & pt0) const;

        typename Lines::iterator getNextEnteringLine(Cmp const & cmp, point_id const & exitPt)
        {
            pt_vs_rect::location A = cmp.classify(exitPt);

            bool orgPassed = false;
            Lines::iterator best = findAnySuitableLine(cmp, exitPt, &orgPassed);

            for (typename Lines::iterator p = lines_.begin(); p != lines_.end(); ++p)
            {
                if (!orgPassed)
                    if (!cmp(exitPt, A, p->front()))
                        continue;

                if (!cmp(best->front(), p->front()))
                    best = p;
            }

            return best;
        }


        typename Lines::iterator findAnySuitableLine(Cmp const & cmp, point_id const & exitPt, bool * orgPassed)
        {
            pt_vs_rect::location A = cmp.classify(exitPt);

            for (typename Lines::iterator p = lines_.begin(); p != lines_.end(); ++p)
            {
                if (cmp(exitPt, A, p->front()))
                    return p;
            }

            *orgPassed = true;
            return lines_.begin();
        }

    private:    
        Lines   lines_;
    };

    //------------------------------------------------------  adding a new line

    template <class Traits>
        void PolyByRectClipper<Traits>::addLine(Traits const & traits, point_id const & pt0, point_id const & pt1)
    {
        if (is_new_line(traits, pt0))
        {
            lines_.push_back(Line());
            lines_.back().push_back(pt0);
        }

        lines_.back().push_back(pt1);
    }

    template <class Traits>
        bool PolyByRectClipper<Traits>::is_new_line(Traits const & traits, point_id const & pt) const
    {
        return lines_.empty() || !traits.compare(lines_.back().back(), pt);
    }

    //------------------------------------------------------- working with undefined starting point

    template <class Traits>
        bool PolyByRectClipper<Traits>::closeFirstAndLastLines()
    {
        // Assert(is_undefined(lines_.front().front()));
        if (lines_.size() > 1)
        {
            std::copy(
                lines_.front().begin(), 
                lines_.front().end(), 
                std::back_inserter(lines_.back()));
            lines_.erase(lines_.begin());
            return false;
        }
        return true;
    }

    template <class Traits>
        template <class OutputIterator>
            void PolyByRectClipper<Traits>::extractInnerPolyline(OutputIterator out)
    {
        // Assert(is_undefined(lines_.front().front()));
        Assert(lines_.size() == 1);

        std::copy(lines_.front().begin(), lines_.front().end(), out);

        lines_.erase(lines_.begin());
   }

   //---------------------------------------------------------- extracting lines

    template <class Traits>
        template <class OutputIterator>
            void PolyByRectClipper<Traits>::extractPolyline(Traits const & traits, cg::rectangle_2 const & bound, OutputIterator out)
    {
        if (lines_.empty()) return;

        Cmp  cmp = traits.comparer(bound);

        typename Lines::iterator first = lines_.begin();
        typename Lines::iterator cur = first;

        do 
        {
            std::copy(cur->begin(), cur->end(), out);

            point_id  exitPt  = cur->back();

            typename Lines::iterator next = getNextEnteringLine(cmp, exitPt);

            pt_vs_rect::location exit  = cmp.classify(exitPt);
            pt_vs_rect::location entry = cmp.classify(next->front());

            if (exit == entry && cmp(next->front(), entry, exitPt))
            {
                *out++ = cmp.getExitCorner(exit);
                ++exit;
            }

            for (; exit != entry; ++exit)
            {
                *out++ = cmp.getExitCorner(exit);
            }

            if (first != cur)
                lines_.erase(cur);

            cur = next;

        } while (first != cur);

        lines_.erase(first);
    }

}