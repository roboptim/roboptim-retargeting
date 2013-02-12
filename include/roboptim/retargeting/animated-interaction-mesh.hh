#ifndef ROBOPTIM_RETARGETING_ANIMATED_INTERACTION_MESH
# define ROBOPTIM_RETARGETING_ANIMATED_INTERACTION_MESH
# include <string>
# include <vector>

# include <log4cxx/logger.h>

# include <boost/shared_ptr.hpp>
# include <boost/graph/graph_traits.hpp>
# include <boost/graph/adjacency_list.hpp>

# include <roboptim/core/function.hh>

// Include this after roboptim::Function to import other Eigen
// headers properly and preserver roboptim-core cpp symbols.
# include <Eigen/StdVector>

namespace YAML
{
  class Node;
} // end of namespace YAML.

namespace roboptim
{
  namespace retargeting
  {
    class AnimatedInteractionMesh;
    typedef boost::shared_ptr<AnimatedInteractionMesh>
    AnimatedInteractionMeshShPtr_t;

    /// \brief Interaction mesh nodes (vertices)
    ///
    /// Contain both the vertex position in the Euclidian space
    /// and the vertex id.
    ///
    /// The id is used to determine what is the vertex position
    /// in the optimization vector.
    ///
    /// Vertex id is implicitly defined by its position in the
    /// graph vertices list. It also matches the vertex position
    /// in the optimization variables vector.
    struct Vertex
    {
      typedef Eigen::VectorBlock<Eigen::VectorXd, 3> position_t;

      /// \brief Vertex label.
      std::string label;

      /// \brief Vertex position in Euclidian space at each frame.
      std::vector<position_t,
		  Eigen::aligned_allocator<position_t> > positions;
    };

    /// \brief Links betweens nodes in the interactive mesh.
    ///
    /// Contains each node weight to avoid useless recomputations.
    struct Edge
    {
      /// \brief Edge weight when computing Laplacian coordinates.
      std::vector<double> weight;

      /// \brief Scaling that should be applied to this edge during
      ///        retargeting.
      ///
      /// 1 means no change.
      double scale;
    };

    /// \brief Stores a set of interaction mesh representing a motion.
    class AnimatedInteractionMesh
    {
    public:
      typedef std::vector<std::string> labelsVector_t;

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


      explicit AnimatedInteractionMesh ();
      ~AnimatedInteractionMesh ();

      static AnimatedInteractionMeshShPtr_t loadAnimatedMesh
      (const std::string& trajectoryFile,
       const std::string& characterFile);

      static AnimatedInteractionMeshShPtr_t makeFromOptimizationVariables
      (const Eigen::Matrix<double, Eigen::Dynamic, 1>& x,
       AnimatedInteractionMeshShPtr_t previousAnimatedMesh);

      const double& framerate () const
      {
	return framerate_;
      }

      unsigned
      numFrames () const
      {
	return numFrames_;
      }

      unsigned
      numVertices () const
      {
	return numVertices_;
      }

      std::size_t
      optimizationVectorSize () const
      {
	return 3
	  * numVertices_
	  * numFrames_;
      }
      std::size_t
      optimizationVectorSizeOneFrame () const
      {
	return 3 * numVertices_;
      }

      const graph_t& graph () const
      {
	return graph_;
      }
      graph_t& graph ()
      {
	return graph_;
      }

      const Eigen::VectorXd&
      makeOptimizationVector () const
      {
	return state_;
      }

      Eigen::VectorBlock<const Eigen::VectorXd, Eigen::Dynamic>
      makeOptimizationVectorOneFrame (unsigned frameId) const
      {
	return state_.segment (frameId * optimizationVectorSizeOneFrame (),
			       optimizationVectorSizeOneFrame ());
      }

      /// \brief Access the interal vector directly.
      ///
      /// Warning: this API is unsafe, it is your responsibility to
      /// update edges weights if you modify the state manually.
      Eigen::VectorXd&
      state ()
      {
	return state_;
      }

      const Eigen::VectorXd&
      state () const
      {
	return state_;
      }

      void writeGraphvizGraphs (std::ostream& out, unsigned id);
      void writeGraphvizGraphs (const std::string& path);
      void writeTrajectory (const std::string& filename);

      /// \brief Compute current edges weights based on vertices positions.
      ///
      /// Each time a vertex position is updated the weights are invalidated
      /// and should be recomputed.
      void computeVertexWeights ();

      /// \brief Recompute cached data using graph.
      void recomputeCachedData ();

    protected:
      vertex_iterator_t
      getVertexFromLabel (const std::string& label) const;

      static void loadEdgesFromYaml
      (const YAML::Node& node, AnimatedInteractionMeshShPtr_t animatedMesh);


    private:
      /// \brief Class logger.
      static log4cxx::LoggerPtr logger;

      /// \name Metadata
      /// \{

      /// \brief Frames per second.
      double framerate_;
      /// \brief Number of frames.
      unsigned numFrames_;
      /// \brief Number of vertices.
      unsigned numVertices_;

      /// \}

      /// \brief Current mesh state.
      Eigen::VectorXd state_;

      /// \brief Underlying Boost graph.
      graph_t graph_;
    };

    /// \brief Write an edge using the Graphviz format.
    template <class Name>
    class InteractionMeshGraphEdgeWriter
    {
    public:
      InteractionMeshGraphEdgeWriter (Name name,
				      unsigned frameId)
	: name (name),
	  frameId (frameId)
      {}

      template <class Edge>
      void
      operator() (std::ostream& out, const Edge& v) const
      {
	out << "[label=\""
	    << "weight: ";
	if (frameId < name[v].weight.size ())
	  out << name[v].weight[frameId];
	else
	  out << "n/a";
	out
	  << ", scale: "
	  << name[v].scale
	  << "\"]";
      }
    private:
      Name name;
      unsigned frameId;
    };

    /// \brief Write a vertex using the Graphviz format.
    template <class Name>
    class InteractionMeshGraphVertexWriter
    {
    public:
      InteractionMeshGraphVertexWriter (Name name,
					unsigned frameId)
	: name (name),
	  frameId (frameId)
      {}

      template <class Vertex>
      void
      operator() (std::ostream& out, const Vertex& v) const
      {
	out << "[label=\""
	    << "id: "
	    << v
	    << ", label: "
	    << name[v].label
	    << ", position: [";
	if (frameId < name[v].positions.size ())
	  out
	    << name[v].positions[frameId][0] << ", "
	    << name[v].positions[frameId][1] << ", "
	    << name[v].positions[frameId][2] << "]";
	  else
	    out << "n/a";
	out << "\"]";
      }
    private:
      Name name;
      unsigned frameId;
    };

  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_ANIMATED_INTERACTION_MESH
