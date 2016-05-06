package roadgraph;

import java.util.ArrayList;

import geography.GeographicPoint;

public class MapDistance implements Comparable<MapDistance>{
	private Double currentDistance;
	private Double heuristicEstimate;
	private Double priority;
	
	public MapDistance()
	{		
		this.currentDistance=Double.POSITIVE_INFINITY;
		this.heuristicEstimate=Double.POSITIVE_INFINITY;
		this.priority=this.currentDistance+this.heuristicEstimate;
	}
	
	public Double getCurrentDistance() {
		return currentDistance;
	}
	public void setCurrentDistance(Double currentDistance) {
		this.currentDistance = currentDistance;
		this.priority=this.currentDistance+this.heuristicEstimate;
	}	
	public Double getHeuristicEstimate() {
		return heuristicEstimate;
	}
	public void setHeuristicEstimate(Double heuristicEstimate) {
		this.heuristicEstimate = heuristicEstimate;
		this.priority=this.currentDistance+this.heuristicEstimate;
	}
	public Double getPriority() {
		return priority;
	}	

	public int compareTo(MapDistance other)
	{
		return Double.compare(this.priority,other.priority);
	}
	
}
