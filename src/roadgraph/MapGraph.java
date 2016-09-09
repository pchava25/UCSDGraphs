/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.ListIterator;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Queue;
import java.util.Random;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.function.Consumer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint,MapNode> vertices;
	private List<MapEdge> edges;
	private HashSet<MapNode> nodesVisited;
	private Double pathDistance=new Double(0.0);

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		vertices=new HashMap<GeographicPoint,MapNode>();
		edges=new ArrayList<MapEdge>();
		nodesVisited=new HashSet<MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if(location==null || location.equals(null) || vertices.containsKey(location))
		{
			return false;
		}
		MapNode newNode=new MapNode(location);
		vertices.put(location,newNode);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		if(from==null || to==null || roadName=="" || roadType=="" || 
				roadName==null || roadType==null || from.equals(null) || 
				to.equals(null) || length<0 || !vertices.containsKey(from) || 
				!vertices.containsKey(to))
		{
			throw new IllegalArgumentException();
		}		
		MapEdge newEdge=new MapEdge(from,to,roadName,roadType,length);
		
		vertices.get(from).addEdge(newEdge);
		vertices.get(from).addNeighbor(vertices.get(to));		
		edges.add(newEdge);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		
		//Initialization
		MapNode sNode=vertices.get(start);
		MapNode gNode=vertices.get(goal);
		
		List<GeographicPoint> result=new ArrayList<GeographicPoint>();
		
		
		if(start==null && goal==null && start.equals(null) && goal.equals(null) 
				&& !vertices.containsValue(start) && !vertices.containsKey(goal))
		{
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
			
		}
		//

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = bfsSearch(sNode, gNode, parentMap,nodeSearched);

		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return constructPath(sNode, gNode, parentMap);
	}
	
	
	private List<GeographicPoint> constructPath(MapNode start, MapNode goal,
			HashMap<MapNode, MapNode> parentMap)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = goal;
		while (curr != start) {
			path.addFirst(curr.getLocation());
			curr = parentMap.get(curr);
		}
		path.addFirst(start.getLocation());
		return path;
	}
	
	private boolean bfsSearch(MapNode start,MapNode goal,HashMap<MapNode, MapNode> parentMap,
			Consumer<GeographicPoint> nodeSearched)
	{
		Queue<MapNode> queue=new LinkedList<MapNode>();
		HashSet<MapNode> visited=new HashSet<MapNode>();
		boolean found=false;
		
		MapNode curr;
		queue.add(start);
		visited.add(start);		
		nodeSearched.accept(start.getLocation());
		
		while(!queue.isEmpty())
		{			
			curr=queue.remove();
			if(curr==goal)
			{						
				found=true;
				break;
			}
			List<MapNode> neighbors=curr.getNeighbors();
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					nodeSearched.accept(next.getLocation());
					parentMap.put(next, curr);
					queue.add(next);
				}
			}					
		}
		
		return found;
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		//Initialization
				MapNode sNode=vertices.get(start);
				MapNode gNode=vertices.get(goal);
				
				List<GeographicPoint> result=new ArrayList<GeographicPoint>();
				
				
				if(start==null && goal==null && start.equals(null) && goal.equals(null) 
						&& !vertices.containsValue(start) && !vertices.containsKey(goal))
				{
					System.out.println("Start or goal node is null!  No path exists.");
					return null;
					
				}
				//

				HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
				
				boolean found = dijkstraSearch(sNode, gNode, parentMap,nodeSearched);
				//System.out.println("found: "+found);
				if (!found) {
					System.out.println("No path exists");
					return null;
				}
				
				
				// Hook for visualization.  See writeup.
				//nodeSearched.accept(next.getLocation());
				
				return constructPath(sNode, gNode, parentMap);

	
	}
	private boolean dijkstraSearch(MapNode start,MapNode goal,HashMap<MapNode, MapNode> parentMap,
			Consumer<GeographicPoint> nodeSearched)
	{ 
		PriorityQueue<MapNode> queue=new PriorityQueue<MapNode>();
		HashSet<MapNode> visited=new HashSet<MapNode>();
		boolean found=false;
		
		MapNode curr;
		MapDistance dist=new MapDistance();
		dist.setCurrentDistance(0.0);
		dist.setHeuristicEstimate(0.0);
		//System.out.println("priority "+dist.getPriority());
		start.setDistances(dist);
		
		queue.add(start);
		//visited.add(start);		
		//nodeSearched.accept(start.getLocation());
		
		while(!queue.isEmpty())
		{			
			curr=queue.remove();
			nodeSearched.accept(start.getLocation());
			//System.out.println("curr "+curr);
			if(!visited.contains(curr))
			{
				visited.add(curr);	
			//	System.out.println("curr "+curr.getCurrentDistance()+" "+curr.getHeuristicEstimate());
				
			
				if(curr==goal)
				{						
					found=true;
					break;
				}
				List<MapNode> neighbors=curr.getNeighbors();
				ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
				while (it.hasPrevious()) {
					MapNode next = it.previous();
					if (!visited.contains(next)) {
						//visited.add(next);
						//nodeSearched.accept(next.getLocation());
						//System.out.println("curr prio "+curr.getDistances().getPriority());
						//System.out.println("next prio "+next.getDistances().getPriority());
						//System.out.println("value "+curr.compareTo(next));
						//System.out.println();
						//
						if(curr.compareTo(next)==-1)	
						{							
							MapDistance dist1=new MapDistance();
							dist1.setCurrentDistance(curr.getDistances().getCurrentDistance()+curr.getLocation().distance(next.getLocation()));
							dist1.setHeuristicEstimate(0.0);
							//System.out.println("distance "+distance);
							//System.out.println("next.getDistances().getCurrentDistance() "+next.getDistances().getCurrentDistance());
							//System.out.println(distance<next.getDistances().getCurrentDistance());
							if(dist1.getCurrentDistance()<next.getDistances().getCurrentDistance())
							{								
								
								
								next.setDistances(dist1);
								//System.out.println("curr dis"+next.getDistances().getCurrentDistance());
								//System.out.println("heu dis"+next.getDistances().getHeuristicEstimate());
								//System.out.println(dist1.getCurrentDistance());
								//System.out.println("path Distance "+pathDistance);
								
								pathDistance+=dist1.getCurrentDistance();
								
								//System.out.println("path Distance 2 "+pathDistance);
								
								parentMap.put(next, curr);
								queue.add(next);
							}
						}					
					}
				}
			}
		}		
		//System.out.println("visited "+visited.size());
		nodesVisited.addAll(visited);
		return found;
	}	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		//Initialization
		MapNode sNode=vertices.get(start);
		MapNode gNode=vertices.get(goal);
		
		List<GeographicPoint> result=new ArrayList<GeographicPoint>();
		
		
		if(start==null && goal==null && start.equals(null) && goal.equals(null) 
				&& !vertices.containsValue(start) && !vertices.containsKey(goal))
		{
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
			
		}
		//

		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = astar(sNode, gNode, parentMap,nodeSearched);

		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return constructPath(sNode, gNode, parentMap);

	}
	private boolean astar(MapNode start,MapNode goal,HashMap<MapNode, MapNode> parentMap,
			Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<MapNode> queue=new PriorityQueue<MapNode>();
		HashSet<MapNode> visited=new HashSet<MapNode>();
		boolean found=false;
		
		MapNode curr;
		MapDistance dist=new MapDistance();
		dist.setCurrentDistance(0.0);
		dist.setHeuristicEstimate(start.getLocation().distance(goal.getLocation()));
		start.setDistances(dist);
		//System.out.println("priority "+start.getPriority());
		queue.add(start);
		//visited.add(start);		
		//nodeSearched.accept(start.getLocation());
		
		while(!queue.isEmpty())
		{			
			curr=queue.remove();
			nodeSearched.accept(start.getLocation());
			//System.out.println("curr "+curr);
			if(!visited.contains(curr))
			{
				visited.add(curr);	
			//	System.out.println("curr "+curr.getCurrentDistance()+" "+curr.getHeuristicEstimate());
				//System.out.println("hello");
			
				if(curr==goal)
				{						
					//System.out.println("hello6");
					found=true;
					break;
				}
				List<MapNode> neighbors=curr.getNeighbors();
				ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
				while (it.hasPrevious()) {
					//System.out.println("hello2");
					MapNode next = it.previous();
					//MapDistance distReset=new MapDistance();
					
					//distReset.setHeuristicEstimate(next.getLocation().distance(goal.getLocation()));
					//next.setDistances(distReset);
					
					if (!visited.contains(next)) {
						//System.out.println("hello3");
						//visited.add(next);
						//nodeSearched.accept(next.getLocation());
						//System.out.println("curr distance "+curr.getDistances().getPriority());
						//System.out.println("curr distance "+next.getDistances().getPriority());
						//System.out.println(curr.compareTo(next));
						if(curr.compareTo(next)==-1)	
						{
							//System.out.println("hello4");
							MapDistance dist1=new MapDistance();
							dist1.setCurrentDistance(curr.getDistances().getCurrentDistance()+curr.getLocation().distance(next.getLocation()));
							dist1.setHeuristicEstimate(next.getLocation().distance(goal.getLocation()));
							//System.out.println(dist1.getPriority());
							if(dist1.getCurrentDistance()<next.getDistances().getCurrentDistance())							
							{		
								//System.out.println("hello5");
								next.setDistances(dist1);					
								parentMap.put(next, curr);
								queue.add(next);
							}
						}					
					}
				}
			}
		}		
		nodesVisited.addAll(visited);
		return found;
	}
	
	private void printVisited(){
		for(MapNode node:nodesVisited)
		{
			System.out.println("lat: "+node.getLocation().getX()+" lon: "+node.getLocation().getY());
		}
	}
	private int countVisited(){
		return nodesVisited.size();
	}
	
	public List<GeographicPoint> greedyDijkstra(List<GeographicPoint> cities,GeographicPoint start) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return greedyDijkstra(cities, start,temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> greedyDijkstra(List<GeographicPoint> cities,GeographicPoint start,
			Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		List<GeographicPoint> bestPath = new ArrayList<GeographicPoint>();
		//System.out.println(start);
		GeographicPoint currCity= start;
		
		//List<GeographicPoint> citiesToVisit = cities;
		
		cities.remove(start);
		bestPath.add(currCity);
		//System.out.println(citiesToVisit.size());
		while(!cities.isEmpty())
		{
			//System.out.println(citiesToVisit.size());
			GeographicPoint nextCity=currCity;
			Double pathLength=0.0;
			//System.out.println("path length "+pathLength);
			//System.out.println("path distance "+pathDistance);
			//System.out.println(citiesToVisit.toString());
			for(int i=0;i<cities.size();i++)
			{
				//System.out.println(citiesToVisit.size());
				dijkstra(nextCity,cities.get(i));
				if(pathLength<pathDistance)
				{
					nextCity=cities.get(i);
					//break;
				}
				pathLength=pathDistance;
			}
			
			//System.out.println("before removed "+cities.size());
			bestPath.add(nextCity);
			currCity=nextCity;
			//System.out.println("index "+cities.indexOf(currCity));
			//System.out.println("contains "+cities.contains(currCity));
			cities.remove(currCity);
			//System.out.println("removed "+cities.remove(currCity));
			//System.out.println(cities.toString());
			//System.out.println("after removed "+cities.size());
		
			
		}
				
		return bestPath;
	}
	
	public static void main(String[] args)
	{
		
		System.out.print("Making a new map...");
		 
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		System.out.println("vertices "+theMap.getNumVertices());
		System.out.println("edges "+theMap.getNumEdges());
		GeographicPoint start=new GeographicPoint(1.0,1.0); 
		GeographicPoint goal=new GeographicPoint(4.0,0.0);
		GeographicPoint location=null;
		System.out.println(theMap.addVertex(null));
		//theMap.addEdge(null, null, "","",8);
		
		System.out.println(theMap.bfs(start,goal)); 

		// You can use this method for testing.  
		
		//Use this code in Week 3 End of Week Quiz
		//MapGraph theMap1 = new MapGraph();
		//System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/maps/utc.map", theMap1);
		//System.out.println("DONE.");

		//GeographicPoint start1 = new GeographicPoint(32.8648772, -117.2254046);
		//GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		//List<GeographicPoint> route = theMap1.greedydijkstra(start1,end);
		//theMap1.printVisited();
		//System.out.println("route "+route);
		//System.out.println("count Visited "+theMap1.countVisited());
		//List<GeographicPoint> route2 = theMap1.aStarSearch(start1,end);
		//theMap1.printVisited();
		
		//System.out.println("count Visited "+theMap1.countVisited());
		//System.out.println("route2 "+route2);
		MapGraph theMap2 = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/new_york.map", theMap2);
		System.out.println("DONE.");
/*
		Set vertices=new HashSet<GeographicPoint>();
		List<GeographicPoint> newVertices=new ArrayList<GeographicPoint>();
		vertices=theMap2.getVertices();
		int i=0;
		while(i<10)
		{
		Object[] v=vertices.toArray();
		Object key = v[new Random().nextInt(v.length)];
				
		newVertices.add((GeographicPoint)key);
			System.out.println(i+" "+(GeographicPoint)key);
			i++;
		}
		
		
		Object[] v1=newVertices.toArray();
		Object key1 = v1[new Random().nextInt(v1.length)];
	*/
	 	
	 
		List<GeographicPoint> newVertices=new ArrayList<GeographicPoint>();
		newVertices.add(new GeographicPoint(40.7584291,-73.9926299));
		newVertices.add(new GeographicPoint(40.7540684,-73.995845));		
		newVertices.add(new GeographicPoint(40.7576858,-74.0006278));
		newVertices.add(new GeographicPoint(40.7595167,-73.9992636));		
		newVertices.add(new GeographicPoint(40.757565,-73.996758));
		
		Object[] v1=newVertices.toArray();
		Object key1 = v1[new Random().nextInt(v1.length)];
		
		GeographicPoint startPoint=(GeographicPoint)key1;
		//System.out.println("check if contains "+newVertices.contains(startPoint));
		List<GeographicPoint> path=theMap2.greedyDijkstra(newVertices,startPoint);
		
		theMap2.printVisited();
		System.out.println("Start "+startPoint);
		System.out.println("greedy algorithm best path "+path);
		System.out.println("path length "+path.size());
		for(int i=0;i<path.size();i++)
		{
			System.out.println(path.get(i));
		}
		System.out.println("count Visited "+theMap2.countVisited());
		System.out.println("number of vertices "+theMap2.getNumVertices());
		System.out.println("path distance "+theMap2.pathDistance);
	}	
}
