#ifndef ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
# define ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
# include <roboptim/core/function.hh>

# include <boost/shared_ptr.hpp>
# include <boost/graph/graph_traits.hpp>
# include <boost/graph/adjacency_list.hpp>

namespace roboptim
{
  namespace retargeting
  {
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
				    boost::no_property,
				    boost::property<
				      boost::vertex_index_t,
				      roboptim::Function::size_type> > graph_t;
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

      const graph_t& graph () const throw ()
      {
	return graph_;
      }

      graph_t& graph () throw ()
      {
	return graph_;
      }

    private:
      graph_t graph_;
    };

    typedef boost::shared_ptr<InteractionMesh> InteractionMeshShPtr_t;
  } // end of namespace retargeting.
} // end of namespace roboptim.

#endif //! ROBOPTIM_RETARGETING_INTERACTION_MESH_HH
