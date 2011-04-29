#pragma once

namespace cg         {
namespace pt_vs_rect {

    // Если less_y "ниже" more_y, 
    enum location { less_x, more_y, more_x, less_y, undefined };

    // то ++ перебирает вершины прямоугольника по часовой стрелке
    inline location & operator ++ (location & L)
    {
        return L = static_cast<location>((L + 1) % undefined);
    }

    // а -- -против часовой стрелки
    inline location & operator -- (location & L)
    {
        return L = static_cast<location>((L - 1) % undefined);
    }
    namespace cw         
    {
        // возвращает координаты угла, который "на конце" стороны L
        inline point_2  getExitCorner(rectangle_2 const & rc, location L)
        {
            switch (L)
            {
            case    less_x: return rc.xY();
            case    more_y: return rc.XY();
            case    more_x: return rc.Xy();
            case    less_y: return rc.xy();
            }
            Assert(L != undefined);
            Assert(0);
            return point_2();
        }

        // Возвращает "смещение" до угла, который "на конце" стороны L
        inline point_2i getCornerOfs(location L)
        {
            switch (L)
            {
            case    less_x: return point_2i(0,1);
            case    more_y: return point_2i(1,1);
            case    more_x: return point_2i(1,0);
            case    less_y: return point_2i(0,0);
            }
            Assert(L != undefined);
            Assert(0);
            return point_2i();
        }

        // Определяет положение точки относительно прямоугольника
        inline location classify(rectangle_2 const & rc, point_2 const & pt, double eps = 1e-8)
        {
//            Assert(contains(rc, pt));
            return
                eq(pt.x, rc.x.lo(), eps)   ?    less_x :
                eq(pt.y, rc.y.lo(), eps)    ?    less_y :
                eq(pt.x, rc.x.hi(), eps)  ?    more_x : 
                eq(pt.y, rc.y.hi(), eps) ?    more_y :
                undefined;
        }

        inline double classifyByBestX(rectangle_2 const & rc, point_2 const & pt, location & loc)
        {
            double L = cg::abs(pt.x - rc.x.lo());
            double R = cg::abs(pt.x - rc.x.hi());
            return L < R ? (loc = less_x, L) : (loc = more_x, R);
        }


        inline double classifyByBestY(rectangle_2 const & rc, point_2 const & pt, location & loc)
        {
            double T = cg::abs(pt.y - rc.y.lo());
            double B = cg::abs(pt.y - rc.y.hi());
            return T < B ? (loc = less_y, T) : (loc = more_y, B);
        }

        inline location classifyByBest(rectangle_2 const & rc, point_2 const & pt)
        {
            location xloc, yloc;

            return classifyByBestX(rc, pt, xloc) < classifyByBestY(rc, pt, yloc) ? xloc : yloc;
        }


        // упорядочивает точки, лежащие на сторонах прямоугольника, по часовой стрелке
        inline bool compare(location A, point_2 const & ptA, location B, point_2 const & ptB)
        {
            if (A != B) return A < B;

            switch (A) // A == B
            {
            case less_x:    return ptA.y < ptB.y;
            case more_y:    return ptA.x < ptB.x;
            case more_x:    return ptA.y > ptB.y;
            case less_y:    return ptA.x > ptB.x;
            }
            return false;   // undefined;
        }

        // Пока что этот класс используется как trait'ы для отклипованных по клеткам точек,
        // так и точек вытащенных из клетки полигонов. В общем это неправильно - должно быть два traits
        // Сейчас мы явно завязываемся на то, что первые 4 индекса - это углы прямоугольника
        // Соответственно, PointSet должен назначать индексы, начиная с 4
        // Именно этот класс должен разруливать, угловая это точка или обычная
        template <class PointSet>
            struct PtOnRectTraits
        {
            PtOnRectTraits (PointSet const & p, rectangle_2 const & rc)
                :   points_ (p)
                ,   rc_     (rc)
            {}

            typedef typename PointSet::point_id     point_id;
            typedef          pt_vs_rect::location   location;

            point_id getExitCorner(location L) const 
            { 
                Assert(L >= 0 && L <= 3); return L;     
            }

            location classify(point_2 const & pt, double eps = 1e-8) const
            {  
                return cw::classify(rc_, pt, eps); 
            }

            location classify(point_id pt, double eps = 1e-8) const
            {
                return classify(at(pt), eps);
            }

            bool operator () (point_id ptA, location A, point_id ptB) const
            {
                return cw::compare(A, at(ptA), classify(ptB), at(ptB));
            }

            bool operator () (point_id ptA, point_id ptB) const
            {
                return cw::compare(classify(ptA), at(ptA), classify(ptB), at(ptB));
            }

            bool is_undefined(point_id const & pt) const
            {
                return classify(pt) == undefined;
            }

            typedef point_2     value_type;
            typedef point_id    size_type;

            point_2 operator [] (point_id id) const
            {
                return at(id);
            }

            point_2 at(point_id x) const
            {
                return corner(x) ? getCorner(x) : point_2(points_.at(x));
            }

            point_2 getCorner(point_id x) const
            {
                return cw::getExitCorner(rc_,static_cast<location>(x));
            }

        private:

            bool corner(point_id x) const { return 0 <= x && x < 4; }

        private:
            PointSet const &   points_;
            rectangle_2        rc_;
        };

        template <class PointSet>
            PtOnRectTraits < PointSet > 
                ptOnRectTraits (PointSet const & points, rectangle_2 const & rc)
        {
            return PtOnRectTraits<PointSet>(points, rc);
        }

        // Класс, передаваемый в качестве traits в PolyByRectClipper
        // Точки задаются индексами в массиве
        // Индексы 0-3 зарезервированы.
        // Они соответствуют углам клетки (т.е. без клетки, для которой они определены, они не имеют смысла)
        template <class PointSet>
            struct HolderOnIndices
        {
            typedef             int      point_id;

            typedef PtOnRectTraits<HolderOnIndices> comparer_type;


            comparer_type comparer(rectangle_2 const & rc) const 
            {
                return comparer_type(*this, rc);
            }

            bool compare(point_id a, point_id b) const
            {
                Assert(!corner(a) && !corner(b));
                return a == b || eq(at(a), at(b));
            }

            HolderOnIndices (PointSet const & points) : points_ (points) {} 

            point_2 const & at(point_id x) const 
            { 
                Assert(!corner(x));  // x == 0..3 не имеет смысла без указания клетки
                return points_[x - 4]; 
            }

            point_2 at(point_id x, rectangle_2 const & rc) const
            {
                return corner(x) ? getCorner(x, rc) : at(x);
            }

            point_2 getCorner(point_id x, cg::rectangle_2 const & rc) const
            {
                Assert(corner(x));
                return cw::getExitCorner(rc, static_cast<location>(x));
            }

        private:

            bool corner(point_id x) const { return 0 <= x && x < 4; }

        private:
            PointSet const & points_;
        };

    }
}}