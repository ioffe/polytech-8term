#pragma once

namespace cg
{
    template<class Traits>
        struct circle_rasterizator
    {
        typedef typename Traits::queue_elem_type queue_elem_type;

        template <class Processor>
            static bool process(point_2 const &o, double r, 
                                Processor const &out, Traits &traits)
        {
            bool res = false;
            traits.init();

            traits.queue_push(o, o);

            while(!traits.queue_empty())
            {
                queue_elem_type idx_d = traits.queue_pop();

                if (idx_d.dist() > r)
                    return res;

                res = true;

                if (out(idx_d, idx_d.dist()))
                    return true;

                for(int i = -1; i <= 1; i++)
                    for(int j = -1; j <= 1; j++)
                        traits.queue_push(idx_d + point_2i(i, j), o);
            }

            return res;
        }
    };

    template <class Processor, class Traits>
        inline bool rasterize_circle(point_2 const &o, double r, Processor const &out, Traits &traits)
    {
        return circle_rasterizator<Traits>::process(o, r, out, traits);
    }
}
