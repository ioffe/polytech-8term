#pragma once

#pragma warning (push)
#pragma warning (disable : 4706)

#include <boost\exception.hpp>

#pragma warning (pop)

namespace cg
{

SKELETON_EXT_METHOD(void) construct_initial_dcel( VertexBuffer const &vertexBuffer )
{
   dcel_.reserve(cntSizes_.back());

   for (MultiContourRandomIterator cIt = multiContour_.begin(); cIt != multiContour_.end(); ++cIt)
   {
      std::vector< cg::point_2 > contour;
      std::vector< ss::VertexData > addData;

      contour.reserve(cIt->size());
      addData.reserve(cIt->size());

      for (MultiContourRandomIterator::value_type::const_iterator vIt = cIt->begin(); vIt != cIt->end(); ++vIt)
      {
         cg::point_2 curPoint = vertexBuffer[*vIt] - basePoint_;
         size_t vertexId = dcelIdx(std::make_pair(cIt - multiContour_.begin(), vIt - cIt->begin()));
         addData.push_back(ss::VertexData (cg::point_2 (), 0, vertexId));
         contour.push_back(curPoint);
      }

      if (attr_.outside)
      {
         std::reverse(contour.begin(), contour.end());
         std::reverse(addData.begin(), addData.end());
      }

      dcel_.addContour(contour.begin(), contour.end(), addData.begin());
   }
}

SKELETON_EXT_METHOD(void) create_correspondence_data()
{
   correspondence_.resize(multiContour_.size());
   for (size_t c = 0; c != multiContour_.size(); ++c)
   {
      correspondence_[c].resize(multiContour_[c].size());
      for (size_t v = 0; v != multiContour_[c].size(); ++v)
      {
         size_t vertex = dcelIdx(std::make_pair(c, v));
         size_t incEdge = dcel_.vertex(vertex).incidentEdge;
         size_t twinEdge = dcel_.edge(incEdge).twinEdge;
         size_t twinNextEdge  = dcel_.edge(twinEdge).nextEdge;
         correspondence_[c][v].corr.insert(CorrespondenceElement (vertex, twinNextEdge));
         correspondence_[c][v].first = correspondence_[c][v].last = *correspondence_[c][v].corr.begin();
      }
   }
}

namespace
{
   inline bool equal( cg::segment_2 const &a, cg::segment_2 const &b )
   {
      return a.P0() == b.P0() && a.P1() == b.P1();
   }

   template< class Skeleton >
      void throw_overlap( Skeleton const &skel, size_t vertexA, size_t vertexB )
   {
      cg::verification::invalid_input_contours result (skel.algorithm_name());
      Skeleton::MultiContourIdx idxA = skel.cntIdx(vertexA);
      Verify(skel.isMultiContourVertex(idxA));
      Skeleton::MultiContourIdx idxB = skel.cntIdx(vertexB);
      Verify(skel.isMultiContourVertex(idxB));

      cg::verification::verification_result error (cg::verification::Verificator (cg::verification::VF_EDGE_OVERLAP |
         cg::verification::VF_SELF_INTERSECTION), idxA.first, idxB.first, skel.realVertexPos(vertexA));

      result.errors = array_1d< cg::verification::verification_result > (1, error);

      throw result;
   }
}

namespace details
{
   template< class DCEL >
      struct compare_vertices
   {
      compare_vertices ( DCEL const &dcel ) : dcel (dcel)
      {
      }

      bool operator () ( size_t a, size_t b )
      {
         return dcel.vertex(a).pos.x < dcel.vertex(b).pos.x ||
            (dcel.vertex(a).pos.x == dcel.vertex(b).pos.x &&
             dcel.vertex(a).pos.y < dcel.vertex(b).pos.y);
      }

   private:
      DCEL const &dcel;
   };
}

SKELETON_EXT_METHOD(void) fix_dcel_calculate_bisectors()
{
   std::vector< size_t > vertices;
   vertices.reserve(dcel_.verticesSize());
   for (size_t v = 0; v < dcel_.verticesSize(); ++v)
      vertices.push_back(v);

   // Determine line contours
   std::vector< bool > lineVertex (vertices.size(), false);
   for (DCEL::cycle_iterator cIt = dcel_.cyclesBegin(); cIt != dcel_.cyclesEnd(); ++cIt)
   {
      bool lineCycle = false;
      for (DCEL::cycle_edge_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
      {
         if (dcel_.vertex(util::prev(eIt)->vertexOrigin).pos == dcel_.vertex(eIt->vertexDestination).pos)
         {
            lineCycle = true;
            break;
         }
      }
      if (lineCycle)
      {
         for (DCEL::cycle_edge_iterator eIt = cIt->begin; eIt != cIt->end; ++eIt)
            lineVertex[eIt->vertexOrigin] = true;
      }
   }

   // Sort vertices
   std::sort(vertices.begin(), vertices.end(), details::compare_vertices< DCEL > (dcel_));

   // Iterate vertices equivalence groups
   size_t vbeg = 0;
   while (vbeg < vertices.size())
   {
      size_t vend = vbeg;
      while (vend < vertices.size() - 1 &&
             cg::eq(dcel_.vertex(vertices[vend]).pos, dcel_.vertex(vertices[vend + 1]).pos))
      {
         ++vend;
      }

      // Merge edges

      std::map< size_t, size_t > corr;

      size_t es = dcel_.vertex(vertices[vbeg]).incidentEdge;
      for (DCEL::exiting_edge_iterator eIt = dcel_.exitingEdgeBegin(es); eIt != dcel_.exitingEdgeEnd(es); ++eIt)
         corr.insert(std::make_pair(eIt.index(), vertices[vbeg]));

      for (size_t v = vbeg + 1; v <= vend; ++v)
      {
         std::vector< size_t > edges;
         edges.reserve(2);
         size_t es = dcel_.vertex(vertices[v]).incidentEdge;
         for (DCEL::exiting_edge_iterator eIt = dcel_.exitingEdgeBegin(es); eIt != dcel_.exitingEdgeEnd(es); ++eIt)
            edges.push_back(eIt.index());
            
         for (size_t i = 0; i < edges.size(); ++i)
            corr.insert(std::make_pair(edges[i], vertices[v]));

         for (size_t i = 0; i < edges.size(); ++i)
         {
            size_t place = dcel_.vertex(vertices[vbeg]).incidentEdge;
            if (!dcel::isVertexDangling(dcel_, vertices[vbeg]))
            {
               place = dcel_.findEdgeNextToLine(dcel_.edge(edges[i]).vertexDestination, vertices[vbeg],
                  DCEL::RobustOrientationPredicate (), false);
               bool eqEdge = equal(cg::dcel::edgeSegment(dcel_, dcel_.edge(place)), cg::dcel::edgeSegment(dcel_, dcel_.edge(edges[i])));

               if (eqEdge)
               {
                  DCEL const & dcel_c = const_cast<DCEL const &>(dcel_);

                  cg::dcel::exiting_edge_const_itpair< DCEL > eqRange =
                     cg::dcel::exitingEdgeEqualRange(dcel_c, dcel_c.exitingEdgeBegin(place));

                  if (util::next(eqRange.first) != eqRange.second || eqRange.first->hole == dcel_.edge(edges[i]).hole)
                     throw_overlap(*this, vertices[v], eqRange.first->vertexOrigin);

                  if (!eqRange.first->hole)
                     place = util::prev(dcel_.exitingEdgeBegin(place)).index();
               }
            }

            size_t prevPlaceEdge = dcel_.edge(place).prevEdge;
            dcel_.edge(prevPlaceEdge).nextEdge = edges[i];
            dcel_.edge(edges[i]).prevEdge = prevPlaceEdge;
            dcel_.edge(place).prevEdge = dcel_.edge(edges[i]).twinEdge;
            dcel_.edge(dcel_.edge(edges[i]).twinEdge).nextEdge = place;
         }
      }

      // Calculate bisectors

      cg::point_2 curPoint = dcel_.vertex(vertices[vbeg]).pos;
      DCEL::exiting_edge_iterator eIter = dcel_.exitingEdgeBegin(es);
      while (eIter != dcel_.exitingEdgeEnd(es))
      {
         if (eIter->hole)
         {
            ++eIter;
            continue;
         }

         DCEL::exiting_edge_iterator nextIt = util::next(eIter);

         if (eIter->hole || !nextIt->hole)
            throw_overlap(*this, eIter->vertexOrigin, nextIt->vertexOrigin);

         size_t curV = corr[eIter.index()];
         size_t nextV = corr[nextIt.index()];

         cg::point_2 prevPoint = dcel_.vertex(nextIt->vertexDestination).pos;
         cg::point_2 nextPoint = dcel_.vertex(eIter->vertexDestination).pos;

         dcel_.vertex(curV).data.bisector = calculateBisector(prevPoint, curPoint, nextPoint);

         DCEL::exiting_edge_iterator nextStepIt = util::next(nextIt);
         if (nextStepIt.index() != eIter.index())
         {
            size_t twinEdge = eIter->twinEdge;
            size_t nextTwinEdge = dcel_.edge(twinEdge).nextEdge;
            size_t prevNextEdge = nextIt->prevEdge;

            dcel_.edge(twinEdge).nextEdge = nextIt.index();
            nextIt->prevEdge = twinEdge;
            dcel_.edge(nextTwinEdge).prevEdge = prevNextEdge;
            dcel_.edge(prevNextEdge).nextEdge = nextTwinEdge;
         }

         if (curV != nextV)
         {
            // Correct correspondence data
            MultiContourIdx idx = cntIdx(curV);
            Verify(isMultiContourVertex(idx));
            VertexCorrespondenceData &curData = correspondence_[idx.first][idx.second];
            curData.corr.begin()->nextEdgeId = dcel_.edge(eIter->twinEdge).nextEdge;
            curData.first = curData.last = *curData.corr.begin();

            idx = cntIdx(nextV);
            Verify(isMultiContourVertex(idx));
            correspondence_[idx.first][idx.second].contactVertex = curV;

            nextIt->vertexOrigin = curV;
            dcel_.edge(nextIt->twinEdge).vertexDestination = curV;

            dcel_.vertex(curV).incidentEdge = eIter.index();
         }

         if (nextStepIt.index() != eIter.index())
         {
            es = nextStepIt.index();
            eIter = dcel_.exitingEdgeBegin(es);
         }
         else
            eIter = nextIt;
      }

      vbeg = vend + 1;
   }
}

SKELETON_EXT_METHOD(void) subdivideReflexAngles()
{
   for (size_t c = 0; c != multiContour_.size(); ++c)
   {
      ss::EventData prevEvent;
      for (size_t v = 0; v != multiContour_[c].size(); ++v)
      {
         size_t parentVertex = dcelIdx(std::make_pair(c, v));

         size_t nextEdge = dcel_.vertex(parentVertex).incidentEdge;
         size_t prevEdge = dcel_.edge(nextEdge).prevEdge;

         size_t prev = dcel_.edge(prevEdge).vertexOrigin;
         size_t cur = parentVertex;
         size_t next = dcel_.edge(nextEdge).vertexDestination;

         if (isVertexReflex(prevEdge, nextEdge))
         {
            cg::point_2 prevP = vertexPosFixed(prev);
            cg::point_2 curP = vertexPosFixed(cur);
            cg::point_2 nextP = vertexPosFixed(next);

            cg::point_2 nextVec = cg::normalized(nextP - curP);
            cg::point_2 prevVec = cg::normalized(prevP - curP);

            double supplAngleCos = nextVec * prevVec;
            if (supplAngleCos > cos(cg::grad2rad(attr_.subdivisionMinAngle)))
            {
               double alpha = cg::rad2grad(acos(cg::clamp< double >(-1, 1, -1, 1)(supplAngleCos)));

               size_t num_points = 2;
               double subdivStep = 0;
               if (alpha >= 2 * attr_.subdivisionMinAngle - 180)
               {
                  subdivStep = 90 - alpha * 0.5;
               }
               else
               {
                  num_points = 2 + cg::ceil((2 * attr_.subdivisionMinAngle - alpha - 180) /
                     (180 - attr_.subdivisionMinAngle));
                  if (num_points % 2 == 0)
                     ++num_points;

                  subdivStep = (360 - attr_.subdivisionMinAngle - alpha) / (num_points + 1);
               }

               size_t numSections = num_points + 1;
               double delta = 0.5 * (180 - alpha - (num_points - 1) * subdivStep);

               double firstBisectorScale = 1 / cos(cg::grad2rad(delta));
               double secondBisectorScale = 1 / cos(cg::grad2rad(subdivStep - delta));

               cg::point_2 curBisector = cg::rotation_2 (delta) * cg::normal(nextVec);

               cg::rotation_2 stepRot (subdivStep);
               size_t bisectorIdx = 0;
               do
               {
                  dcel_.vertex(cur).data.bisector = curBisector;

                  if (bisectorIdx % 2 == 0)
                     dcel_.vertex(cur).data.bisector *= firstBisectorScale;
                  else
                     dcel_.vertex(cur).data.bisector *= secondBisectorScale;

                  if (bisectorIdx != numSections - 2)
                  {
                     curBisector = stepRot * curBisector;

                     size_t newVertex = dcel_.addVertex(dcel_.vertex(cur));
                     dcel_.vertex(newVertex).data.parentVertices.insert(parentVertex);
                     dcel_.vertex(newVertex).data.processed = false;
                     dcel_.vertex(newVertex).data.t = t_;

                     size_t prevEdgeTwin = dcel_.edge(prevEdge).twinEdge;
                     size_t nextEdgeTwin = dcel_.edge(nextEdge).twinEdge;
                     size_t newEdge = dcel_.addEdge(DCEL::Edge (newVertex, cur, prevEdge, nextEdge, static_cast< size_t >( -1 ), false));
                     dcel_.vertex(newVertex).incidentEdge = newEdge;

                     size_t newEdgeTwin = dcel_.addEdge(DCEL::Edge (cur, newVertex, nextEdgeTwin, prevEdgeTwin, newEdge, true));
                     dcel_.edge(newEdge).twinEdge = newEdgeTwin;

                     if (cur != parentVertex)
                        addCorrespondenceData(parentVertex, cur, newEdgeTwin);

                     dcel_.edge(nextEdge).prevEdge = newEdge;
                     dcel_.edge(prevEdge).nextEdge = newEdge;
                     dcel_.edge(nextEdgeTwin).nextEdge = newEdgeTwin;
                     dcel_.edge(prevEdgeTwin).prevEdge = newEdgeTwin;

                     dcel_.edge(prevEdge).vertexDestination = newVertex;
                     dcel_.edge(prevEdgeTwin).vertexOrigin = newVertex;

                     cur = newVertex;
                     nextEdge = newEdge;
                  }
                  else
                     addCorrespondenceData(parentVertex, cur, dcel_.edge(prevEdge).twinEdge);

                  bisectorIdx++;
               } while (bisectorIdx < numSections - 1);

               updateCorrespondenceData(parentVertex, parentVertex,
                  dcel_.edge(dcel_.edge(dcel_.vertex(parentVertex).incidentEdge).prevEdge).twinEdge);

               correspondence_[c][v].last = *correspondence_[c][v].corr.find(cur);
            }
         }
      }
   }
}

} // End of 'cg' namespace
