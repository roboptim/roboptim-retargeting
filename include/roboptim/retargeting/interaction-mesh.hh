#ifndef ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
# define ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
# include <roboptim/core/function.hh>

# include <boost/shared_ptr.hpp>
# include <boost/graph/graph_traits.hpp>
# include <boost/graph/adjacency_list.hpp>

namespace YAML
{
  class Node;
} // end of namespace YAML.

namespace roboptim
{
  namespace retargeting
  {
    class InteractionMesh;
    typedef boost::shared_ptr<InteractionMesh> InteractionMeshShPtr_t;

    /// \brief Interaction mesh nodes (vertices)
    ///
    /// Contain both the vertex position in the Euclidian space
    /// and the vertex id.
    ///
    /// The id is used to determine what is the vertex position
    /// in the optimization vector.
    struct Vertex
    {
      /// \brief Vertex id, aka position amongst optimization variables.
      unsigned id;
      /// \brief Vertex position in Euclidian space.
      Eigen::Matrix<double, 3, 1> position;
    };

    /// \brief Links betweens nodes in the interactive mesh.
    ///
    /// Contains each node weight to avoid useless recomputations.
    struct Edge
    {
      /// \brief Edge weight when computing Laplacian coordinates.
      double weight;
    };

    /// \brief Store an interaction mesh structure.
    ///
    /// An interaction mesh is a bidirectional graph.
    ///
    /// This class wraps a BGL (Boost Graph Library).
    class InteractionMesh
    {
    public:
      typedef boost::adjacency_list<boost::vecS,
				    boost::vecS,
				    boost::undirectedS,
				    Vertex,
				    Edge> graph_t;
      typedef boost::graph_traits<graph_t>::vertex_iterator
      vertex_iterator_t;
      typedef boost::graph_traits<graph_t>::edge_iterator
      edge_iterator_t;
      typedef boost::graph_traits<graph_t>::out_edge_iterator
      out_edge_iterator_t;
      typedef boost::graph_traits<graph_t>::edge_descriptor
      edge_descriptor_t;
      typedef boost::graph_traits<graph_t>::vertex_descriptor
      vertex_descriptor_t;
      typedef boost::property_map<graph_t, boost::vertex_index_t>::type
      index_map_t;
      typedef boost::graph_traits <graph_t>::adjacency_iterator
      adjacency_iterator_t;

      explicit InteractionMesh ();
      ~InteractionMesh ();

      static InteractionMeshShPtr_t loadMesh
      (const std::string& trajectoryFile, unsigned frameId);

      const graph_t& graph () const throw ()
      {
	return graph_;
      }

      graph_t& graph () throw ()
      {
	return graph_;
      }

      /// \brief Compute current edges weights based on vertices positions.
      ///
      /// Each time a vertex position is updated the weights are invalidated
      /// and should be recomputed.
      void computeVertexWeights ();

    private:
      /// \brief Underlying Boost graph.
      graph_t graph_;
    };

    void operator >> (const YAML::Node& node, InteractionMesh& mesh);

  } // end of namespace retargeting.

} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
