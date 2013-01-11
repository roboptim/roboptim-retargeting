#include "roboptim/retargeting/interaction-mesh.hh"

namespace roboptim
{
  namespace retargeting
  {
    InteractionMesh::InteractionMesh ()
    {}

    InteractionMesh::~InteractionMesh ()
    {}

    void
    InteractionMesh::computeVertexWeights ()
    {
      edge_iterator_t edgeIt;
      edge_iterator_t edgeEnd;

      double weightSum = 0.;

      boost::tie (edgeIt, edgeEnd) = boost::edges (graph ());
      for (; edgeIt != edgeEnd; ++edgeIt)
	{
	  Edge& edge = graph ()[*edgeIt];
	  const Vertex& source = graph ()[boost::source (*edgeIt, graph ())];
	  const Vertex& target = graph ()[boost::target (*edgeIt, graph ())];

	  std::cout << "--- edge ---" << std::endl
		    << "source position: "
		    << source.position[0] << " "
		    << source.position[1] << " "
		    << source.position[2] << std::endl
		    << "target position: "
		    << target.position[0] << " "
		    << target.position[1] << " "
		    << target.position[2] << std::endl;

	  edge.weight = (source.position - target.position).squaredNorm ();
	  if (edge.weight == 0.)
	    edge.weight = 1.;
	  else
	    edge.weight = 1. / edge.weight;
	  weightSum += edge.weight;
	}

      // Normalize weights.
      if (weightSum > 0.)
	{
	  boost::tie (edgeIt, edgeEnd) = boost::edges (graph ());
	  for (; edgeIt != edgeEnd; ++edgeIt)
	    graph ()[*edgeIt].weight /= weightSum;
	}
    }
  } // end of namespace retargeting.
} // end of namespace roboptim.
