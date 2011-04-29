#pragma once

#include <map>

#include "common/safe_bool.h"
#include "rasterization\segment_raster_iterator.h"
#include "triangle_raster_aux.h"
#include "primitives\range.h"

namespace cg 
{
    namespace triangle_rasterization
    {
            // каждой клеточке сопоставляется диапазон 
            // заметаемых границей треугольника высот
        typedef std::map<point_2i, range_2>     HeightMap;

            // каждой ординате сопоставляется 
            // левая и првая граница треугольника
            // На самом деле частный случай массива, в котором индексация не с нуля
        struct Sides  
        {
            Sides() {}

            Sides(int base, size_t sz) : base_(base), data_(sz) {}

            void setup(int base, size_t sz) {
                base_ = base; data_.resize(sz);
            }

            range_2i & get (int idx) {
                return data_.at(idx - base_);
            }

            int min() const { return base_; }
            int max() const { return base_ + (int)data_.size(); }

            bool is_valid(int idx) const {
                int dy = idx - base_;
                return 0 <= dy && size_t(dy) < data_.size();
            }
        
            void intersect_with_cell(point_2i const &)
            {
            
            }

        private:
            std::vector<range_2i>   data_;
            int                     base_;
        };

        struct SidesCreator
        {
            SidesCreator(Sides & sides, point_2i const & clipping)
                :   sides_ (sides), clipping_ (clipping)
            {}

            bool operator () (point_2i idx, double, double)
            {
                if (0 <= idx.y && idx.y < clipping_.y)
                {
                    sides_.get(idx.y).unite(idx.x);
                }

                return false;
            }

        private:
            Sides          & sides_;
            point_2i const & clipping_;
        };


        struct AddSideToRaster
        {
            AddSideToRaster (HeightMap & heightmap, Sides & sides,  double h0, double h1)
                :   heightmap_(heightmap), sides_(sides), h0_(h0), h1_(h1)
            {}

            bool operator () (point_2i const &idx, double in_ratio, double out_ratio)
            {
                Assert(sides_.is_valid(idx.y));
                //  раздвигаем левую и правую стороны
                sides_.get(idx.y).unite(idx.x);
                //  обновляем heightmap
                double z0 = clamp(0., 1., h0_, h1_)(in_ratio);
                heightmap_[idx].unite(z0);
                double z1 = clamp(0., 1., h0_, h1_)(out_ratio);
                heightmap_[idx].unite(z1);
                return false;
            }

        private:
            HeightMap   & heightmap_;
            Sides       & sides_;
            double const  h0_, h1_;
        };
    }

    inline segment_2 edge(triangle_3 const &t, int v0, int v1) {
        return segment_2(t[v0], t[v1]);
    }


    struct triangle3_rasterization_iterator
    {
        typedef triangle_rasterization::HeightMap       HeightMap;
        typedef triangle_rasterization::Sides           Sides;
    
        triangle3_rasterization_iterator( triangle_3 const &triangle )
            : triangle_aux_ ( triangle )
        {
            triangle_2 tr2 = triangle_2( triangle );

            point_2i v1 = floor( tr2[0] );
            point_2i v2 = floor( tr2[1] );
            point_2i v3 = floor( tr2[2] );

            int y_min = min(v1.y, v2.y, v3.y);

            // create new SIDE arrays
            int y_height = max(v1.y, v2.y, v3.y) - y_min + 1;

            m_Sides.setup(y_min, y_height);

            process_side(triangle[0], triangle[1]);
            process_side(triangle[1], triangle[2]);
            process_side(triangle[2], triangle[0]);

            // init iterator
            y = y_min;
            x = m_Sides.get(y).lo();
        }

    private:

        void process_side(point_3 const &p0, point_3 const &p1)
        {
            using triangle_rasterization::AddSideToRaster;

            AddSideToRaster  processor (m_heightmap, m_Sides, p0.z, p1.z);
            rasterize_segment(segment_2(p0, p1), processor);
        }
    
    public:

        point_2i operator * () const {
            return point_2i( x, y );
        }

        SAFE_BOOL_OPERATOR(m_Sides.is_valid(y))

        range_2 get_zrange() const 
        {
            HeightMap::const_iterator entry = m_heightmap.find( point_2i(x,y) );

            range_2 zrange ;

            if (entry != m_heightmap.end())
                zrange = entry->second;

            // calc height for internal cell
            point_2 p[4];
            p[0] = point_2(x,     y);
            p[1] = point_2(x,     y + 1);
            p[2] = point_2(x + 1, y);
            p[3] = point_2(x + 1, y + 1);

            cg::barycentric_coords bc;
            for (int i=0; i<4; i++)
                if (triangle_aux_.IsInTriangleBarycentric(p[i], bc)) // проверяем, лежат ли они внутри проекции треугольника на растр
                {
                    // находим Z-координату точки, лежащей на треугольнике
                    double z = triangle_aux_.GenerateHeightForPoint( p[i].x, p[i].y );
                    zrange.unite(z);
                }

            return zrange;
        }

        triangle3_rasterization_iterator& operator ++ () 
        {
            x ++;
            if ( x > m_Sides.get(y).hi() )
            {
                y ++;

                if ( m_Sides.is_valid(y) )
                    x = m_Sides.get(y).lo();
            }

            return *this;
        }

    private:
        triangle_raster_aux     triangle_aux_;

        Sides       m_Sides;
        HeightMap   m_heightmap;

        int x, y;
    };

    namespace traster_details
    {
        using triangle_rasterization::Sides;

        // процессор, который используется внутри растеризатора треугольника
        // вызывается при построении сторон треугольника
        template <class ActualProcessor>
            struct SegmentContructionCellProcessor
        {
            SegmentContructionCellProcessor(
                Sides & sides, ActualProcessor & actual_processor)
                :   sides_(sides), actual_processor_(actual_processor)
            {}

            bool operator () (point_2i const &idx, double in_ratio, double out_ratio)
            {
                Assert(sides_.is_valid(idx.y));
                sides_.get(idx.y).unite(idx.x);

                actual_processor_(idx, in_ratio, out_ratio);

                return false;
            }
        
        private:
            Sides           & sides_;
            ActualProcessor & actual_processor_;
        };
    }

    template <class Processor>
        bool rasterize_triangle (triangle_2 const &t, Processor & processor)
    {
        return rasterize_triangle( t[0], t[1], t[2], processor );
    }

    template <class point_t, class Processor>
        bool rasterize_triangle (point_t const &tv1, point_t const& tv2, point_t const& tv3, Processor & processor)
    {
        // клетки для вершин треугольника
        point_2i v1 = floor( tv1 );
        point_2i v2 = floor( tv2 );
        point_2i v3 = floor( tv3 );

        int min_y = min(v1.y, v2.y, v3.y);
        int max_y = max(v1.y, v2.y, v3.y);

        namespace tr = traster_details;

        tr::Sides       sides(min_y, max_y - min_y + 1);

        // ToDo: избавиться от упоминания SideProcessor

        typedef 
            tr::SegmentContructionCellProcessor<Processor::SideProcessor> sproc;
    
        // По-хорошему здесь надо сделать: 
        //    передавать в rasterize_triangle прямоугольник отсечения
        //    клиповать по нему отрезки ==> автоматически отклипуются треугольники

        sproc   ab_processor (sides, processor.side_processor(0,1));
        rasterize_segment(segment_2(tv1, tv2), ab_processor);

        sproc   bc_processor (sides, processor.side_processor(1,2));
        rasterize_segment(segment_2(tv2, tv3), bc_processor);

        sproc   ca_processor (sides, processor.side_processor(2,0));
        rasterize_segment(segment_2(tv3, tv1), ca_processor);

        point_2i  cell;

        for (cell.y = min_y; cell.y <= max_y; ++cell.y) 
        {
            range_2i const & x_range = sides.get(cell.y);

            for (cell.x = x_range.lo(); cell.x <= x_range.hi(); ++cell.x)
            {
                if (processor(cell))
                    return true;
            }
        }

        return false;
    }


}
