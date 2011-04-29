#pragma once
#include <list>
#include "Geometry\cycle_val.h"

namespace filter
{
   // как минимум дл€ cycled_value тип разницы двух значений не совпадает с типом самого значени€,
   // поэтому необходима метафункци€ diff_type  
   template < typename T >
   struct diff1_type 
   {
      typedef T type ;
   };

   template < typename T >
   struct diff2_type
   {
      typedef typename diff1_type< typename diff1_type< T >::type >::type type ;
   };
                   
   template <>
   struct diff1_type< cg::degree180_value >
   {
      typedef double type ;   
   };

    //////////////////////////////////////////////////////////////////////////
    // сглаживающие функции
    //////////////////////////////////////////////////////////////////////////
    
//-- фильтраци€ одномерных сигналов

    // сглаживание, ограничивающее максимальное приращение по модулю
    // cX - current X, dX - desired X 
    template <typename vector, typename norm> 
        typename diff1_type< vector >::type 
        ApproachLinearX ( vector const& cX, vector const& dX, norm deltaAbsX )
    {
        vector delta = ( dX - cX ) ; 
        norm   deltaNorm = cg::norm( delta ) ;

        if ( deltaNorm > deltaAbsX )
           delta   *= deltaAbsX / deltaNorm ;

        return delta ;
    }

    // сглаживание, ограничивающее скорость изменени€
    template <typename vector, typename norm> 
        typename diff1_type< vector >::type 
        ApproachLinearX( vector const& cX, vector const& dX, 
                         typename diff1_type< vector >::type const& vX, norm dt )
    {
        return ApproachLinearX <vector, norm> ( cX, dX, (typename diff1_type< vector >::type)( vX * dt )) ;
    }

    // DX/dt = alpha * (desired - X) 
    template <typename T> T ApproachExpX(T cX, T dX, T alpha)
    {
        T dLX = cg::abs(dX - cX) * alpha;
        return (cX < dX) ? (cX + dLX) : (cX - dLX);
    }

    // DX/dt = alpha * (desired - X)^gamma
    template <typename T> T ApproachExpX(T cX, T dX, T alpha, T gamma, T offset)
    {
        T dLX;
        T arg = cg::abs(dX - cX);
        if (arg < offset)
            dLX = alpha * offset * pow(arg / offset, gamma);
        else 
            dLX = alpha * arg;

        return (cX < dX) ? (cX + dLX) : (cX - dLX);
    }

    // сглаживание, ограничивающее максимальную первую и вторую производную
    // при достижении желаемой позиции скорость обнул€етс€ (!)
    template < class scalar >
      typename diff1_type< scalar >::type 
      BreakApproachSpeed
         (  scalar curPos, scalar desPos, 
            typename diff1_type< scalar >::type curSpeed, 
            typename diff1_type< scalar >::type maxSpeed, 
            typename diff2_type< scalar >::type maxAccel, 
            double dt, double smoothFactor = 1 ) 
   {
      Assert( cg::ge( smoothFactor, 1 )) ;

      typedef
         typename diff1_type< scalar >::type
         dscalar ;

      dscalar dpos    = desPos - curPos ;
      dscalar speed   = dpos / ( smoothFactor * dt ) ;

      dscalar maxBreakSpeed = cg::sqrt( dscalar(2) * maxAccel * cg::abs ( dpos ) ) ; 
      dscalar    breakSpeed = maxBreakSpeed - maxAccel * dt / 2 ; // средн€€ скорость на calc step'e

      if ( maxBreakSpeed - maxAccel * dt < 0 )
         return dpos / dt ;

      // ограничиваем ускорение 
      speed = cg::bound<dscalar>( speed, curSpeed - maxAccel * dt, curSpeed + maxAccel * dt );
      
      // если дл€ торможени€ нужна меньша€ скорость - используем ее 
      if ( ( speed * dpos ) > 0 && breakSpeed < cg::norm( speed ))
         speed = cg::sign( speed ) * breakSpeed ;

//       if ( breakSpeed < cg::norm( speed )) 
//          speed = cg::sign( dpos ) * breakSpeed ;
//       else 
//          speed = cg::bound<dscalar>( speed, curSpeed - maxAccel * dt, curSpeed + maxAccel * dt );

      return cg::bound<dscalar>( speed, -maxSpeed, maxSpeed );
   }

    // сглаживание, ограничивающее максимальную первую и вторую производную;
    // при достижении желаемой позиции скорость не (!!) обнул€етс€
    template < typename vector, typename norm > 
       typename diff1_type< vector >::type 
         ApproachSpeed 
         (  vector const& curPos, vector const& desPos, 
            typename diff1_type< vector >::type const& curV, 
            norm maxVNorm, norm maxANorm, double dt, double smoothFactor = 1. )
    {
       Assert( !cg::eq_zero( dt )) ;

       typedef diff1_type < vector >::type dvector ;
       
       //-- max velocity constraint
       dvector desV = ( desPos - curPos ) / ( dt * smoothFactor ) ; 
       
       norm desVNorm = cg::norm( desV ) ;
       if ( desVNorm > maxVNorm ) 
          desV *= maxVNorm / desVNorm ;
       
       //-- max accel constraint
       dvector newV = cg::clamp_d( 0., cg::norm( desV - curV ), curV, desV ) ( maxANorm * dt ) ;

       //-- new velocity
       return newV ;
    }

    // фильтр с пам€тью. ќкно задаетс€ frameSize, дефолтное значение def
    // процессор задает профиль коэффициентов
    template <typename T > 
        struct FrameFilter
    {
        FrameFilter (size_t frameSize, T const& def = T()) 
            : m_Next  ( 0 )
            , m_Values( frameSize, def )
        {
        }

        size_t size  () { return m_Values.size(); }
        void   clear () { m_Values.resize(0) ; }
        void   push  (T const& v)  
        { 
            m_Values[m_Next] = v;
            m_Next = (m_Next + 1) % m_Values.size();
        }

        template<class Processor>
           Processor visit( Processor proc )
        {
            for (size_t i = m_Next; i != m_Values.size(); ++i)
               proc(m_Values[i]);

            for (size_t i = 0; i != m_Next; ++i)
               proc(m_Values[i]);

            return proc;
        }

    private:
        typedef std::vector<T> values_type;

        size_t       m_Next     ;
        values_type  m_Values   ;
    };

    template<typename T>
       struct AverageProcessor
    {
       AverageProcessor( size_t size, T const& start = T() )
          : m_Size(size)
          , m_Sum (start)
       {}

       void operator()( T const& t ) { m_Sum += t; }
       T    operator()() const { return m_Sum / m_Size; }

    private:
       size_t  m_Size;
       T       m_Sum;
    };
       

} // end of namespace 'filter'

// end of file 'filter.h'

