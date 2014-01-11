package trafficSim.agent;

import java.awt.Color;
import java.util.List;

import repast.simphony.annotate.AgentAnnot;
import repast.simphony.space.grid.GridPoint;
import trafficSim.Main;
import trafficSim.car.CarAction;
import trafficSim.car.CarColor;
import trafficSim.car.CarPosition;
import trafficSim.car.CarReasoner;
import trafficSim.car.CarReasonerState;
import trafficSim.car.context.CarContext;
import trafficSim.factory.CarContextFactory;
import trafficSim.map.CarMap;
import trafficSim.utils.Direction;
import trafficSim.utils.Speed;
import trafficSim.utils.Turn;
import trafficSim.utils.Utilities;
import trafficSim.utils.norms.Norm;

/**
 * The car agent
 *
 * @author Javier Morales (jmoralesmat@gmail.com)
 *
 */
@AgentAnnot(displayName = "Car Agent")
public class Car implements TrafficElement
{
	//-----------------------------------------------------------------
	// Attributes
	//-----------------------------------------------------------------

	private CarReasoner reasoner;
	private CarPosition position;
	private CarContext context;

	private CarColor color;

	private CarReasonerState reasonerState;
	private Turn turn;

	//	private Action lastPerformedAction;
	private CarAction nextAction;

	private boolean hasTurned;
	private boolean collided; 

	private Norm normToViolate;
	private Norm normToApply;
	
	private short id;
	private int defaultSpeed;
	private int speed;

	private int ticksCollided;
	private int expectedArrivalTicks;
	private int numTicksTraveling;
	
	private int successfulJourneys;
	private int collisions;
	private int journeys;
	
	private float fitness;
	
	private int group;


	//-----------------------------------------------------------------
	// Constructors
	//-----------------------------------------------------------------

	/**
	 * This constructor chooses a random start point for the car
	 */
	public Car(short id, boolean withReasoner){
		this.id = id;
		this.hasTurned = false;
		this.collided = false;
		this.ticksCollided = 0;
		this.defaultSpeed = 1;
		this.numTicksTraveling = 0;
		this.speed = this.defaultSpeed;
		this.color = new CarColor();
		
		//To calculate fitness function
		this.successfulJourneys = 0;
		this.collisions = 0;
		this.journeys = 0;
		this.fitness = 0;
		this.group = 0;

		
		
		if(withReasoner){
			this.reasoner = new CarReasoner();
		}

		this.reasonerState = CarReasonerState.NoNormActivated;
		this.nextAction = CarAction.Go;

		// Reasoner for the car. Only create it if it's not a clone. If
		// it's a clone, it doesn't need the reasoner, so due to details of
		// efficiency we don't assign anyone to it
		//		if(withReasoner) {
		//			this.reasoner = new CarReasoner(this);
		//		}
	}

	//-----------------------------------------------------------------
	// Scheduled methods
	//-----------------------------------------------------------------

	public void move()
	{
		CarMap carMap = Main.getMap();

		this.incTicks();

		// Execute last decided action and turn if needed
		this.execute(nextAction);
		this.turn(carMap);
	}
	/**
	 * Reason about the current situation. Then decide what to do
	 */
	public void perceiveAndReason() {
		CarMap carMap = Main.getMap();

		// Perceive and reason to get the action to apply in the next step
		this.context = this.perceive(carMap);
		this.nextAction = this.reason();
	}

	/**
	 * 
	 * @param action
	 */
	private void execute(CarAction action) {

		// Apply the next action to do
		switch (this.nextAction)
		{
		case Go:
			this.speed = defaultSpeed;
			movement();
			break;

		case Stop:
			this.speed = 0;
			break;
			
		default:
			break;
		}
	}

	/**
	 * The car perceives the environment 
	 */
	private CarContext perceive(CarMap carMap) {
		return this.generateScope(carMap);
	}

	/**
	 * Makes the car reason about its current situation
	 */
	private CarAction reason() {
		CarAction action;

		// Decide next action with the reasoner
		action = reasoner.decideAction(this, context);
		this.reasonerState = reasoner.getState();

		this.normToViolate = reasoner.getNormToViolate();
		this.normToApply = reasoner.getNormToApply();
		
		return action;
	}

	//-----------------------------------------------------------------
	// Methods
	//-----------------------------------------------------------------

	/**
	 * Initializes the car 
	 */
	public void init(CarPosition position) {

		this.position = position;
		this.turn = Utilities.getRandomTurn();
		this.collided = false;

		// Set expected arrival time
		switch(this.turn) {
		case Left:
			this.expectedArrivalTicks = 23;
			break;
		case Straight:
			this.expectedArrivalTicks = 21;
			break;
		case Right:
			this.expectedArrivalTicks = 19;
			break;
		}
	}

	/**
	 * Moves the car one step in the direction that it's oriented
	 */
	private void movement() {
		position = position.add(getMovementVector());
	}

	/**
	 * Check if has to turn and turn if so. Turns the car to the direction it has to turn
	 */
	public void turn(CarMap carMap) {
		int x = position.getX();
		int y = position.getY();

		if(!hasTurned) {
			if( (position.getDirection()==Direction.North && ((turn==Turn.Right && y == carMap.getLowerLane()) ||
					((turn==Turn.Left || turn==Turn.Straight) && y==carMap.getUpperLane()))) ||
					(position.getDirection()==Direction.South && ((turn==Turn.Right && y == carMap.getUpperLane()) ||
							((turn==Turn.Left || turn==Turn.Straight) && y==carMap.getLowerLane()))) ||
							(position.getDirection()==Direction.East && ((turn==Turn.Right && x == carMap.getLeftLane()) ||
									((turn==Turn.Left || turn==Turn.Straight) && x==carMap.getRightLane()))) ||
									(position.getDirection()==Direction.West && ((turn==Turn.Right && x == carMap.getRightLane()) ||
											((turn==Turn.Left || turn==Turn.Straight) && x==carMap.getLeftLane()))))
			{
				position.turn(turn);
				hasTurned=true;
			}
		}
	}

	/**
	 * Calculates the movement vector of the car
	 */
	private GridPoint calcMovementVector() {
		int movement[] = Utilities.getDirVector(position.getDirection());
		for(int i=0; i<movement.length; i++)
			movement[i] *= speed;
		return new GridPoint(movement[0],movement[1]);
	}

	/**
	 * Generates the car scope in the current moment
	 */
	public CarContext generateScope(CarMap carMap)
	{
		CarContext context = CarContextFactory.getCarContextIn(carMap, id, CarContext.Type.Front);
		
		//this.leftSideScope  = CarScope.generate(carMap.getMap(), this, CarScope.Type.Left);
		//this.rightSideScope  = CarScope.generate(carMap.getMap(), this, CarScope.Type.Right);

		return context;
	}

	/**
	 * 
	 */
	private void incTicks() {
		this.numTicksTraveling++;
	}

	//-----------------------------------------------------------------
	// Getters and setters
	//-----------------------------------------------------------------

	/**
	 * Returns the movement vector of the car
	 * 
	 * @return
	 */
	private GridPoint getMovementVector(){
		return calcMovementVector();
	}

	/**
	 * Returns the estimated speed of the car
	 * 
	 * @return
	 */
	public Speed getEstimatedSpeed() {
		if(speed == 0)
			return Speed.None;
		else if(speed == 1)
			return Speed.Medium;
		else if(speed == 2)
			return Speed.High;
		else
			return Speed.VeryHigh;
	}

	/**
	 * Returns the estimated speed of the car relative to the other car's speed
	 * 
	 * @return
	 */
	public Speed getEstimatedSpeed(int otherSpeed) {
		if(speed == 0)
			return Speed.None;

		int diff = speed - otherSpeed;
		switch(diff) {
		case -2:	return Speed.MuchLower;
		case -1:	return Speed.Lower;
		case 0:		return Speed.Equal;
		case 1:		return Speed.Higher;
		case 2:		return Speed.MuchHigher;
		default:	return Speed.None;
		}
	}

	/**
	 * Returns the car scope
	 */
	public CarContext getFrontScope() {
		return context;
	}

	/**
	 * 
	 * @return
	 */
	public int getSpeed() {
		return speed;
	}

	/**
	 * Returns the position of the car at the moment
	 * 
	 * @return
	 */
	public CarPosition getPosition() {
		return position;
	}

	/**
	 * 
	 * @return
	 */
	public CarReasoner getReasoner() {
		return reasoner;
	}

	/**
	 * 
	 * @return
	 */
	public long getId() {
		return id;
	}

	/**
	 * Returns the x coordinate of the car
	 */
	public int getX() {
		return position.getX();
	}

	/**
	 * Returns the y coordinate of the car
	 */
	public int getY() {
		return position.getY();
	}

	/**
	 * Returns a string describing the object
	 */
	@Override
	public String toString(){
		return this.position.getDirection().getArrow();
	}

	/**
	 * Returns a extended string describing the object
	 * @return
	 */
	public String toStringExt(){
		return "Pos: " + position + " turn: " + turn.name() + " has turned: " + hasTurned;		
	}

	/**
	 * Returns true if the car has collided
	 */
	public boolean isCollided() {
		return collided;
	}

	/**
	 * @param collided the collided to set
	 */
	public void setCollided(boolean collided) {
		this.collided = collided;
		if(collided) {
			this.ticksCollided = 0;
		}
	}

	/**
	 * 
	 * @return
	 */
	public int getTicksCollided() {
		return this.ticksCollided;
	}

	/**
	 * 
	 * @return
	 */
	public void setTicksCollided(int ticksCollided) {
		this.ticksCollided = ticksCollided;
	}

	/**
	 * 
	 */
	public void setSpeed(int speed) {
		this.speed = speed; 
	}

	/**
	 * @param pos
	 */
	public void setPosition(CarPosition pos){
		position = pos;
	}

	/**
	 * 
	 * @return
	 */
	public CarAction getNextAction() {
		return this.nextAction;
	}

	/**
	 * 
	 * @return
	 */
	public CarReasonerState getReasonerState() {
		return this.reasonerState;
	}

	/**
	 * 
	 * @param state
	 */
	public void setReasonerState(CarReasonerState state) {
		this.reasonerState = state;
	}

	/**
	 * 
	 * @param color
	 */
	public void setCarColor(CarColor color) {
		this.color = color;
	}

	/**
	 * 
	 * @return
	 */
	public Color getColor() {
		return this.color.getColor();
	}

	/**
	 * 
	 * @return
	 */
	public int getExpectedArrivalTicks() {
		return this.expectedArrivalTicks;
	}

	/**
	 * 
	 * @return
	 */
	public int getFinalArrivalTicks() {
		return this.numTicksTraveling;
	}

	/**
	 * 
	 * @return
	 */
	public boolean isCasualStop() {
		return this.reasoner.isCasualStop();
	}

	/**
	 * 
	 * @return
	 */
	public int getNumTicksTraveling() {
		return this.numTicksTraveling;
	}


	/**
	 * Returns the last norm that was violated by the car
	 * 
	 * @return
	 */
	public Norm getNormToViolate() {
		return this.normToViolate;
	}

	/**
	 * Sets the last violated norm
	 * 
	 * @param n
	 */
	public void setNormToViolate(Norm n) {
		this.normToViolate = n;
	}

	/**
	 * Returns the last norm that was applied by the car
	 * 
	 * @return
	 */
	public Norm getNormToApply() {
		return this.normToApply;
	}

	/**
	 * Sets the last applied norm
	 * 
	 * @param n
	 */
	public void setNormToApply(Norm n) {
		this.normToApply = n;
	}
	/**
	 * Clones the car
	 */
	public Car clone(boolean withScopes) {
		CarPosition pos = new CarPosition(position.getX(), position.getY(),
				position.getDirection());

		Car car = new Car(this.id, false);
		car.init(pos);

		car.setReasonerState(this.reasonerState);
		car.setSpeed(this.speed);
		car.setCollided(this.collided);
		car.setTicksCollided(this.ticksCollided);
		car.setCarColor(this.color);

		if(withScopes && this.getFrontScope() != null) {
			car.context = context;
		}
		return car;
	}
	
	
	public int getSuccessfulJourneys() {
  	return successfulJourneys;
  }

	public void setSuccessfulJourneys(int successfulJourneys) {
  	this.successfulJourneys = successfulJourneys;
  }

	public int getCollisions() {
  	return collisions;
  }

	public void setCollisions(int collisions) {
  	this.collisions = collisions;
  }

	public int getJourneys() {
  	return journeys;
  }

	public void setJourneys(int journeys) {
  	this.journeys = journeys;
  }

	public void setFitness(float f) {
		this.fitness = f;
  }
	
	public float getFitness(){
		return this.fitness;
	}
	
	
	/**
	 * 
	 * @return
	 */
	public List<Norm> getNorms()
	{
		return this.reasoner.getNorms();
	}
	
	/**
	 * 
	 * @return
	 */
	public boolean hasNorms()
	{
		return this.reasoner.hasNorms();
	}
}
