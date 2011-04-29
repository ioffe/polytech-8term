#pragma   once

namespace cg
{
    namespace segment_rasterization_2
    {
        // Растеризация отрезка вдоль оси координат, но не приндалежит одной клетке
        template <class Policy>
            bool rasterize_aa_segment(Policy t)
        {
            state::inside  inside;
            state::slave   slave;

            if (t.process(inside, slave))
                return true;

            while (t.slave_cond_B())
                if (t.process(slave, slave))
                    return true;

            return t.process(slave, inside);
        }

        // Растеризация отрезка, который в одной клетке :-))
        template <class Policy>
            bool rasterize_cell_segment(Policy t)
        {
            state::inside inside;

            return t.process(inside, inside);
        }
    }
}