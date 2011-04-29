#pragma once

namespace cg
{       
    template <class Point>
        struct DuplicateEliminator2dTraits
    {
        typedef 
           typename Point::scalar_type  
           scalar_type;

        struct Comparer
        {
            bool operator () (Point const & lhs, Point const & rhs) const
            {
                return lhs.x < rhs.x || lhs.x == rhs.x && lhs.y < rhs.y;
            }
        };

        Point construct_start_point(Point const & src, scalar_type majorAxis) const
        {
            Point pt(src);
            pt.x = src.x - majorAxis;
            pt.y = src.y;
            return pt;
        }

        // провер€ет, что pt оказалась дальше на majorAxis по главной оси от src
        bool  is_farther(Point const & pt, Point const & src, scalar_type majorAxis) const
        {
            return pt.x - src.x > majorAxis;
        }

        bool is_closer(Point const & pt, Point const & src, scalar_type threshold) const
        {
            return distance_sqr(pt, src) < threshold * threshold;
        }
    };

    template <class Point>
        struct DuplicateEliminator3dTraits
           : DuplicateEliminator2dTraits< Point >
    {        
        struct Comparer
        {
            bool operator () (Point const & lhs, Point const & rhs) const
            {
                return lhs.x < rhs.x
                   || (lhs.x == rhs.x && lhs.y < rhs.y)
                   || (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z < rhs.z);
            }
        };

         Point construct_start_point(Point const & src, scalar_type majorAxis) const
         {
            return Point(src.x - majorAxis, src.y, src.z);
         }
    };    

     template < int Dim, class Point > struct DuplicateEliminatorTraits {};

     template < class Point > struct DuplicateEliminatorTraits< 2, Point >
     { typedef DuplicateEliminator2dTraits< Point > type; };

     template < class Point > struct DuplicateEliminatorTraits< 3, Point >
     { typedef DuplicateEliminator3dTraits< Point > type; };
        
    // ѕредназначен дл€ выкидывани€ дублирующихс€ точек
    // ¬ идеале облако точек (св€зный граф, в котором длина любого ребра не превышает эпсилон), 
    // должно быть заменено их центром.
    // ƒанный элиминатор будет замен€ть облако первой попавшей в него точкой.
    // Ѕолее того, он будет оставл€ть от облака не одну точку.
    // ќставл€ть одну точку - значит хранить дл€ каждого облака его радиус, а это влом
    template <class Point, class Traits=typename DuplicateEliminatorTraits< Point::dimension, Point >::type>
        struct DuplicatePointsEliminator
    {
        typedef 
           typename Traits::Comparer           
           Comparer;

        typedef 
           std::set<Point, Comparer>           
           Points;

        typedef 
           typename Points::const_iterator              
           PCI;

        typedef 
           typename Points::iterator                    
           PI;

        typedef 
           PCI                                 
           const_iterator;

        typedef 
           std::pair<PI, bool>                 
           insert_return_type;

        DuplicatePointsEliminator(double eps) 
            :   epsilon_    (eps) 
        {}

        double eps() const { return epsilon_; }

        // true, если вставка произошла успешно
        insert_return_type    insert(Point const & pt)
        {
            Point const & start_pt = traits_.construct_start_point(pt, epsilon_);

            for (PI it = points_.lower_bound(start_pt); it != points_.end(); ++it)
            {
                if (traits_.is_farther(*it, pt, epsilon_))
                {
                    break;
                }

                if (traits_.is_closer(*it, pt, epsilon_))
                {
                    return std::make_pair(it, false);
                }
            }
            // inserting...
            return points_.insert(pt);
        }

        const_iterator begin() const { return points_.begin(); }
        const_iterator end  () const { return points_.end();   }

    private:
        Points       points_;

        // когда сливаютс€ точки
        double       epsilon_;

        Traits       traits_;
    };
}