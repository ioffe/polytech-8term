#pragma once

#include "geometry\primitives\point.h"
#include "geometry\point_ops.h" 

namespace sections   {
namespace gen        {

   namespace details
   {      
      // ����� ����� ����������� ����� e � ���������� z = h      
      template < class Scalar, size_t Dim >
         cg::point_t< Scalar, Dim > interpolate( cg::point_t< Scalar, Dim > const & p1, cg::point_t< Scalar, Dim > const & p2, double h )
      {         
         Assert( !(p1.z < h && p2.z < h || p1.z > h && p2.z > h) );

         if( p1.z == p2.z )
         {
            // ����� �������������� � ������������� ��������������
            // ����� �����-������. ������� ����� �� �� ����� ����.
            Assert( p1 == p2 );
            return p1;
         }

         typedef Scalar scalar_type;

         return cg::point_t< Scalar, Dim >( cg::Lerp< scalar_type, cg::point_t< Scalar, Dim > >( p1.z, p2.z, p1, p2 )( static_cast< scalar_type >( h ) ) );
      }

      struct v_entry 
      {
         point_3  pt;
         int      sgn;
         size_t   idx;

         friend bool operator < (v_entry const & lhs, v_entry const & rhs) 
         {
               return lhs.sgn < rhs.sgn;
         }
      };
      
      template < class Vertices >
      struct OutputFormatter
      {
         OutputFormatter( Vertices const & v, double h, size_t & side_1, size_t & side_2 ) 
            : v_( v ), h_( h ), side_1_( side_1 ), side_2_( side_2 )
         {}

         void _0_2_2_1_ ()
         {
            side_1_ = v_[1].idx;
            side_2_ = v_[0].idx;

            // 2 - ����� ������� �����. ��� ������ ���� ������
            if ( !cg::right_turn_strict( v_[0].pt, v_[1].pt, v_[2].pt ) )
               std::swap( side_1_, side_2_ );
         }

         void _1_0_0_2_ ()
         {
            side_1_ = v_[2].idx;
            side_2_ = v_[1].idx;

            // 0 - ����� ������ �����. ��� ������ ���� �����
            if( cg::right_turn_strict( v_[1].pt, v_[2].pt, v_[0].pt ) )
               std::swap( side_1_, side_2_ );
         }

      private:                  
         Vertices const &        v_;
         double                  h_;
         size_t      &           side_1_;
         size_t      &           side_2_;
      };
   }

   // e1, e2 ����������� ����� �������, ��� �������������� ������ ����� ����������� �� ������� �������.
   // �.�. ������ ����� ��������� ����� � ������� ���������   
   inline bool triangle_section( const cg::triangle_3 & tr, double h, size_t & side_1, size_t & side_2 )
   {            
      //  ������ ������� �������� �� �������������
      if ( ( tr[0].z > h && tr[1].z > h && tr[2].z > h ) ||
           ( tr[0].z < h && tr[1].z < h && tr[2].z < h ) )
         return false;

      typedef details::v_entry Vertices[3];

      Vertices    v;

      // ������� ������� ������������ � ������������� �� ������������ ���������
      for (int i = 0; i != 3; ++i)
      {         
         v[i].pt  = tr[i];
         v[i].sgn = cg::sign( tr[i].z - h);
         v[i].idx = i;
      }

      int b = v[0].sgn + v[1].sgn + v[2].sgn;

      // ���� ����������� �� ����� ����������� � ���������� 
      // ��� ������������ � ����� �����, �� ������������
      
      // ��������� ���: ������������� ������ ������������, ������� ���� ��������� �������
      // ����� ����������� �� ����� ����� (��� ����� ����������� ���� ���������) ����� �����������
      // � ����������� �� ����� ������ - ���. ���� ����� �������� �������� ������������������ �������� -- 
      // ��� ������� ��������� �������������� ����� � ��������� ������� ���� 3 � ���� 0 ��� �����, ��� ��� 
      // ���������� ������� ������ ������ ������ � ����� ������ �� �����, �� � ��� �� ��������. 
      // _Kovalev
      if (b >= 3 || b <= -2)
         return false;

      std::sort( &v[0], &v[3] );

      details::OutputFormatter< Vertices > output( v, h, side_1, side_2 );

      switch (v[0].sgn)
      {
      case -1:
         switch (v[1].sgn)
         {
         case -1:
         {
            Assert(v[2].sgn == 1);

            output._0_2_2_1_();
            break;
         }         
         case 0:
         {
            if (v[2].sgn == 1)
            {
               output._0_2_2_1_();
            }
            else
            {
               Assert(v[2].sgn == 0);

               // ��� ����� ������� ����� �������� �� �������� �����,
               // ������� � ��������� � ��������� ������� ������������� ������.
               // � ��� ��� ����������

               return false;
            }
            break;
         }         
         case 1:
         {
            Assert(v[2].sgn == 1);

            output._1_0_0_2_();
            break;
         }         
         default:
            Assert(0);
         }
         break;

      case 0:
         switch (v[1].sgn)
         {
         case -1: 
            Assert(0); 
            break;

         case  0:
         {
            if (v[2].sgn == 1)
            {
               output._0_2_2_1_();

               break;
            }

            if (v[2].sgn == 1)
               return false;
            break;
         }         
         case  1:
         {
            Assert(v[2].sgn == 1);

            output._1_0_0_2_();
            break;
         }         
         default:
            Assert(0);
         }
         break;
      case 1:
         Assert(0);
      }

      return true;
   }

}}