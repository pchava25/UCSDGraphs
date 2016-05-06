package roadgraph;

import geography.GeographicPoint;

public class MapEdge {

	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private String streetType;
	private Double distance;
	
	public MapEdge(GeographicPoint start, GeographicPoint end, String streetName, String streetType, Double distance) {
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.streetType = streetType;
		this.distance = distance;
	}
	
	public GeographicPoint getStart() {
		return start;
	}
	public void setStart(GeographicPoint start) {
		this.start = start;
	}
	public GeographicPoint getEnd() {
		return end;
	}
	public void setEnd(GeographicPoint end) {
		this.end = end;
	}
	public String getStreetName() {
		return streetName;
	}
	public void setStreetName(String streetName) {
		this.streetName = streetName;
	}
	public String getStreetType() {
		return streetType;
	}
	public void setStreetType(String streetType) {
		this.streetType = streetType;
	}
	public Double getDistance() {
		return distance;
	}
	
	public void setDistance(Double distance) {
		this.distance = distance;
	}
	
	
	
}
