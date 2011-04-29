#pragma once
#pragma warning (push)
#pragma warning (disable : 4706)

#include <boost\exception.hpp>

#pragma warning (pop)

#include "contours/readytouse/layered_contours.h"
#include "straight_skeleton.h"

namespace cg
{

template< class VertexBufferIterator, class VertexBuffer, class ObjectsRandomIterator >
   class StraightSkeletonClusteredGenerator
{
public:
   typedef
      typename VertexBuffer::value_type
      VertexT;

   typedef
      typename ObjectsRandomIterator::value_type::const_iterator
      MultiContourRandomIterator;

   typedef
      typename MultiContourRandomIterator::value_type::const_iterator
      VerticesIterator;

   typedef
      typename MultiContourRandomIterator::value_type::const_iterator::value_type
      IndexType;

   typedef
      std::vector< IndexType >
      IndicesType;

   typedef
      std::vector< IndicesType >
      ObjectType;

   typedef
      std::vector< ObjectType >
      ObjectsType;

   typedef
      StraightSkeletonGenerator< VertexBuffer, MultiContourRandomIterator >
      straight_skeleton;

   typedef
      boost::shared_ptr< straight_skeleton >
      straight_skeleton_ptr;

   StraightSkeletonClusteredGenerator ( StraightSkeletonClusteredGenerator const &other )
      : vertexBuffer_ (other.vertexBuffer_)
      , objects_ (other.objects_)
      , attrs_ (other.attrs_)
      , attrIdx2objIdxs_ (other.attrIdx2objIdxs_)
      , delayProcess_ (other.delayProcess_)
      , obj2cluster_ (other.obj2cluster_)
      , objCntOffset_ (other.objCntOffset_)
      , clusterCnt2Obj_ (other.clusterCnt2Obj_)
      , verifyInput_ (other.verifyInput_)
   {
      for (size_t c = 0; c < other.clusters_.size(); ++c)
         clusters_.push_back(straight_skeleton_ptr (new straight_skeleton (*other.clusters_[c])));
   }

   StraightSkeletonClusteredGenerator( VertexBuffer const &vertexBuffer,
         ObjectsRandomIterator objBegin, ObjectsRandomIterator objEnd,
         ss::ConstructionParams const &attr, bool delayProcess = false, bool verify = false )
      : vertexBuffer_ (vertexBuffer)
      , delayProcess_ (delayProcess)
      , attrs_ (1, attr)
      , verifyInput_ (verify)
   {
      for (ObjectsRandomIterator oIt = objBegin; oIt != objEnd; ++oIt)
         objects_.push_back(ObjectType (oIt.begin(), oIt.end()));

      std::vector< size_t > &idxs = attrIdx2objIdxs_[0];
      idxs.reserve(objects_.size());
      for (size_t i = 0; i < objects_.size(); ++i)
         idxs.push_back(i);

      process();
   }

   template< class AttrIterator, class MapObject >
      StraightSkeletonClusteredGenerator(
            VertexBufferIterator vertexBuffersBegin,
            VertexBufferIterator vertexBuffersEnd,
            ObjectsRandomIterator objBegin, ObjectsRandomIterator objEnd,
            AttrIterator attrBegin, AttrIterator attrEnd,
            MapObject objIdx2attrIdx, bool delayProcess = false )
      : delayProcess_ (delayProcess)
      , attrs_ (attrBegin, attrEnd)
   {
      for (ObjectsRandomIterator oIt = objBegin; oIt != objEnd; ++oIt)
         objects_.push_back(ObjectType (oIt->begin(), oIt->end()));

      VertexBufferIterator vIt = vertexBuffersBegin;
      for (size_t i = 0; i < objects_.size(); ++i, ++vIt)
      {
         if (i != 0)
         {
            for (size_t c = 0; c < objects_[i].size(); ++c)
               for (size_t v = 0; v < objects_[i][c].size(); ++v)
                  objects_[i][c][v] += vertexBuffer_.size();
         }

         vertexBuffer_.insert(vertexBuffer_.end(), (*vIt).begin(), (*vIt).end());
      }

      for (size_t i = 0; i < objIdx2attrIdx.size(); ++i)
         attrIdx2objIdxs_[objIdx2attrIdx[i]].push_back(i);

      process();
   }

   size_t size() const
   {
      return clusters_.size();
   }

   straight_skeleton_ptr &operator[] ( size_t idx )
   {
      return clusters_[idx];
   }

   straight_skeleton_ptr &getCluster( size_t objIdx )
   {
      Assert(obj2cluster_.find(objIdx) != obj2cluster_.end());
      return clusters_[obj2cluster_[objIdx]];
   }

   std::pair< size_t, size_t > clusterIdx( size_t objIdx, size_t cnt, size_t vertex )
   {
      return std::make_pair(cnt + objCntOffset_[objIdx], vertex);
   }

   size_t objIdx( size_t cluster, size_t cnt )
   {
      Assert(clusterCnt2Obj_.find(std::make_pair(cluster, cnt)) != clusterCnt2Obj_.end());
      return clusterCnt2Obj_[std::make_pair(cluster, cnt)];
   }

private:
   void process()
   {
      distribute_objs_to_clusters();
   }

   struct LayerStub
   {
      template< class iter, class attr >
         void add_contour( iter, iter, size_t group_id, attr const & )
      {
         groups.insert(group_id);
      }

      std::set< size_t > groups;
   };

   void distribute_objs_to_clusters()
   {
      objCntOffset_.resize(objects_.size());

      // build_status::counter progress (L"Creating clusters");

      size_t curCluster = 0;
      for (size_t a = 0; a < attrs_.size(); ++a)
      {
         // Distribute contours with equal attributes to clusters
         std::vector< size_t > &idxs = attrIdx2objIdxs_[a];

         typedef
            cg::contours::gen::details::CreateLayeredContours< LayerStub >
            layers_generator;

         layers_generator lgen;

         for (size_t c = 0; c < idxs.size(); ++c)
         {
            ObjectType const &curObj = objects_[idxs[c]];

            for (size_t oC = 0; oC < curObj.size(); ++oC)
            {
               std::vector< VertexT > contour;
               contour.reserve(curObj[oC].size());
               for (size_t oCV = 0; oCV < curObj[oC].size(); ++oCV)
                  contour.push_back(vertexBuffer_[curObj[oC][oCV]]);

               lgen.addContour(contour.begin(), contour.end(), cg::Empty (), idxs[c], 0);
            }
         }

         lgen.createLayers();

         // Construct new clusters
         for (size_t l = 0; l < lgen.layers().size(); ++l, ++curCluster)
         {
            std::vector< MultiContourRandomIterator::value_type > skel_input;
            {
               build_status::step prep_step (L"prepare cluster constructing");

               LayerStub const &layer = *lgen.layers()[l];
               size_t curCntOffset = 0;
               skel_input.reserve(layer.groups.size());
               std::set< size_t >::const_iterator cIt = layer.groups.begin();
               for (; cIt != layer.groups.end(); ++cIt)
               {               
                  ObjectType const &curObj = objects_[*cIt];
                  for (size_t mcit = 0; mcit < curObj.size(); ++mcit)
                  {
                     clusterCnt2Obj_[std::make_pair(curCluster, skel_input.size())] = *cIt;
                     skel_input.push_back(curObj[mcit]);
                  }
                  objCntOffset_[*cIt] = curCntOffset;
                  obj2cluster_[*cIt] = curCluster;
                  curCntOffset += curObj.size();
               }
            }

            if (verifyInput_)
            {
               build_status::step verification_step (L"verifying skeleton input");

               try
               {
                  cg::verification::check_input< straight_skeleton >
                     (vertexBuffer_, skel_input.begin(), skel_input.end(),
                     (float)cg::epsilon< double >());
               }
               catch (cg::verification::invalid_input_contours &result)
               {
                  cg::verification::invalid_input_contour_objects newRes (result.algorithm);
                  std::vector< cg::verification::verification_result_objects > tmp;
                  tmp.reserve(result.errors.size());
                  for (size_t i = 0; i < result.errors.size(); ++i)
                  {
                     size_t objIdxA =
                        clusterCnt2Obj_[std::make_pair(curCluster, result.errors[i].contourA())];
                     size_t objIdxB =
                        clusterCnt2Obj_[std::make_pair(curCluster, result.errors[i].contourB())];

                     tmp.push_back(cg::verification::verification_result_objects (
                        result.errors[i].verificator(),
                        objIdxA, result.errors[i].contourA() - objCntOffset_[objIdxA],
                        objIdxB, result.errors[i].contourB() - objCntOffset_[objIdxB],
                        result.errors[i].refPoint()));
                  }
                  if (!tmp.empty())
                     newRes.errors = cg::array_1d< cg::verification::verification_result_objects > (tmp.begin(), tmp.end());

                  throw newRes;
               }
            }

            {
               build_status::step new_cl_step (L"cluster constructing - skeleton initialization");

               straight_skeleton *skel =
                  new straight_skeleton (vertexBuffer_,
                     skel_input.begin(), skel_input.end(), attrs_[a], delayProcess_);

               clusters_.push_back(straight_skeleton_ptr (skel));
            }
         }
      }
   }

private:
   // Input data
   bool verifyInput_;
   VertexBuffer vertexBuffer_;
   ObjectsType objects_;
   std::vector< ss::ConstructionParams > attrs_;
   std::map< size_t, std::vector< size_t > > attrIdx2objIdxs_;
   bool delayProcess_;

   // Intermediate & Output data
   std::map< size_t, size_t > obj2cluster_;
   std::vector< straight_skeleton_ptr > clusters_;
   std::vector< size_t > objCntOffset_;
   std::map< std::pair< size_t, size_t >, size_t > clusterCnt2Obj_;
};

}
