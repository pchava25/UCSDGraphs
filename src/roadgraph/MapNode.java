package roadgraph;

import java.util.List;
import java.util.ArrayList;

import geography.GeographicPoint;

public class MapNode implements Comparable<MapNode>{
	private GeographicPoint location;
	private List<MapEdge> edges;
	private List<MapNode> neighbors;
	private MapDistance distances;
	
	public MapNode(GeographicPoint location)
	{
		this.location=location;
		this.edges=new ArrayList<MapEdge>();
		this.neighbors=new ArrayList<MapNode>();
		this.distances=new MapDistance();
	}
	public GeographicPoint getLocation() {
		return location;
	}
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}
	public List<MapEdge> getEdges() {
		return edges;
	}
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	public List<MapNode> getNeighbors() {
		return neighbors;
	}
	public void addNeighbor(MapNode neighbor) {
		neighbors.add(neighbor);
	}	
	public MapDistance getDistances() {
		return distances;
	}
	public void setDistances(MapDistance distances) {
		this.distances = distances;
	}
	public int compareTo(MapNode other)
	{
		return this.distances.compareTo(other.distances);
	}
}
