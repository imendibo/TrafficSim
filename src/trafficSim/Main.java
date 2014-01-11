package trafficSim;

import java.util.ArrayList;
import java.util.Random;

import repast.simphony.annotate.AgentAnnot;
import repast.simphony.context.Context;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.space.grid.Grid;
import trafficSim.agent.Car;
import trafficSim.agent.TrafficElement;
import trafficSim.car.CarAction;
import trafficSim.config.TrafficConfiguration;
import trafficSim.map.CarMap;
import trafficSim.metrics.TrafficMetrics;
import trafficSim.utils.TrafficGrammar;
import trafficSim.utils.norms.Norm;
import trafficSim.utils.norms.NormModality;
import trafficSim.utils.norms.NormPrecondition;

/**
 * Scene Manager - Main class of implementation. Controls cooperation
 * of all the components, agent generation etc.
 *
 * @author Javier Morales (jmoralesmat@gmail.com)
 *
 */
@AgentAnnot(displayName = "Main Agent")
public class Main implements TrafficElement
{

	//-----------------------------------------------------------------
	// Static
	//-----------------------------------------------------------------

	/**
	 * Random machine
	 */
	private static Random random;
	
	/**
	 * 
	 */
	private static CarMap carMap = null;

	/**
	 * 
	 */
	public static TrafficGrammar grammar = new TrafficGrammar();
	
	//-----------------------------------------------------------------
	// Attributes
	//-----------------------------------------------------------------


	/**
	 * 
	 */
	private TrafficMetrics trafficMetrics;
	
	/**
	 * 
	 */
	private static TrafficConfiguration config;
	
	/**
	 * Other
	 */	
	public static long tick = 0;
	public static long T = 100;
	public static int fitnessTime = 0;
	ArrayList<Integer> normProportion;
	int carNumber;

	
	//-----------------------------------------------------------------
	// Methods
	//-----------------------------------------------------------------

	/**
	 * Constructor
	 * 
	 * @param grid
	 * @param context
	 * @param valueLayer
	 * @param map
	 */
	public Main(Context<TrafficElement> context, Grid<TrafficElement> map)
	{
		long seed = System.currentTimeMillis();

		TrafficConfiguration.init();
		
		// Set the defined seed only if the simulation is not batch and the seed is != 0
		if(TrafficConfiguration.SIM_RANDOM_SEED != 0l) {
			seed = TrafficConfiguration.SIM_RANDOM_SEED;
		}
		
		carNumber = 60;
		normProportion = new ArrayList<Integer>();
		random = new Random(seed);
		carMap = new CarMap(context, map, carNumber);
		
		System.out.println("\nStarting simulation with random seed = " + seed);
		
		this.initNormsPool(carMap);
		
	}

	/**
	 * Step method. Controls everything in the simulation
	 */
	@ScheduledMethod(start=1, interval=1, priority=2)
	public void step(){
		if(tick++ > 0){
			fitnessTime++;
			System.out.println("\n==================== Tick: " + tick +  " =====================");

			// 1. Move cars, emit cars and handle new collisions
			carMap.step();
			
			/*
			 * Do the Fitness Function
			 */
			if(fitnessTime >= T){
				System.out.println("---------------------------------------------------");
				System.out.println("fitness function");
				
				fitnessFunction(carMap);
				fitnessTime = 0;
			}
		}
		
		// Step resume
		this.printStepResume();
		
		System.out.println("\n======================= End Step ========================\n");

		// Stop simulation if required update
		if(tick >= TrafficConfiguration.SIM_MAX_TICKS || this.mustStop()){
			System.out.println("End of simulation");
			RunEnvironment.getInstance().endRun();
			
//			System.out.println("Tick NSCardinality NSNumClauses NumCreatedGeneralisationNodes NumVisitedGeneralisationNodes " +
//					"NumCreatedNorms NumVisitedNorms NumVisitedPredicates NumQueriesToNN");
			
		}
	}

	//-----------------------------------------------------------------
	// Other
	//-----------------------------------------------------------------

	private void fitnessFunction(CarMap carMap) {
		int k = 1;
		float f = 0;
		
		for (Car c : carMap.getAllCars()){
			System.out.println("car = "+c.getId());
			System.out.println(c.getSuccessfulJourneys());
			System.out.println(c.getCollisions());
			
			f = (float) ((k * c.getSuccessfulJourneys()) - (2 * k * c.getCollisions()));
			System.out.println(f);
			c.setFitness(f);
		}
		float fNorm = 0, fNoNorm = 0;
		//ArrayList<Float> fitnesses = new ArrayList<Float>();
		
		for (Car c : carMap.getAllCars()){
			 c.hasNorms();
			 c.getNorms();
			 
			if(c.getId() <= normProportion.get(0)){
				fNorm = fNorm + c.getFitness();
			}else{
				fNoNorm = fNoNorm + c.getFitness();
			}
		}
		fNorm = fNorm / normProportion.get(0);
		fNoNorm = fNoNorm / normProportion.get(1);
		
		System.out.println("fitness norm = "+fNorm);
		System.out.println("fitness no norm = "+fNoNorm);
		
		//Normalize de fitness function to positive
		if (fNorm < 0 || fNoNorm < 0){
		  float m = Math.min(fNorm,fNoNorm);
		  if (fNorm < fNoNorm){
		  	fNorm = Math.abs(m);
		  	fNoNorm = fNorm - Math.abs(fNoNorm) + Math.abs(m);
		  }else{
		  	fNoNorm = Math.abs(m);
		    fNorm = fNoNorm - Math.abs(fNorm) + Math.abs(m);
		  }
		}
		
		System.out.println("fitness norm = "+fNorm);
		System.out.println("fitness no norm = "+fNoNorm);
		
		//Calculate the percentage of the fitness functions with the population proportions. 
		float proportionNorm = normProportion.get(0)*100/carNumber;
		float proportionNoNorm = normProportion.get(1)*100/carNumber;

		System.out.println("Proportion norm = "+proportionNorm);
		System.out.println("Proportion no norm = "+proportionNoNorm);
		
		float fitnessNorm = (float) ((proportionNorm * fNorm) / ((proportionNorm * fNorm) + (proportionNoNorm * fNoNorm))) * 100;
		float fitnessNoNorm = (float) ((proportionNoNorm * fNoNorm) / ((proportionNoNorm * fNoNorm) + (proportionNorm * fNorm))) * 100;
		
		System.out.println("proportion fnorm "+fitnessNorm);
		System.out.println("proportion fnonorm "+fitnessNoNorm);

		//Make the asexual reproduction of the norms for the next iterations.
		asexualReproduction(fitnessNorm, fitnessNoNorm, carMap);

		//Reset all the values to calculate new fitness function.
		for (Car c : carMap.getAllCars()){
			c.setSuccessfulJourneys(0);
			c.setCollisions(0);
			c.setJourneys(0);
			c.setFitness(0);
		}
  }

	private void asexualReproduction(float fitnessNorm, float fitnessNoNorm, CarMap map) {
		int countNorm = 0;
		int countNoNorm = 0;
		//Each car decide whereas pick a norm or no norm, by the percentage of probability of these ones.
		for (Car c : map.getAllCars()){
  		int n = (int)(100.0 * Math.random()) + 1;
  		if(n <= fitnessNorm){
				map.broadcastAddNorm(n1, c);
				countNorm++;
  		}else{
  			countNoNorm++;
  			//No norm... so nothing.
  		}
		}
		//Delete last proportions.
		normProportion.removeAll(normProportion);

		//New proportions.
		normProportion.add(countNorm);
		normProportion.add(countNoNorm);
		
		System.out.println("New Proportions... norm = "+countNorm+" no norm = "+countNoNorm);
  }

	Norm n1;
	
	private void initNormsPool(CarMap map) {
		
		/**
		 * Give way to left.
		 */
		NormPrecondition precond = new NormPrecondition();
		precond.add("l", ">");
		precond.add("f", "*");
		precond.add("r", "*");
		n1 = new Norm(precond, NormModality.Prohibition, CarAction.Go);
		n1.setId(1);
		
		/**
		 * Give way to right.
		 */
		precond = new NormPrecondition();
		precond.add("l", "*");
		precond.add("f", "*");
		precond.add("r", "<");
		Norm n2 = new Norm(precond, NormModality.Prohibition, CarAction.Go);
		n2.setId(2);
		
		/**
		 * Keep security distance.
		 */
		precond = new NormPrecondition();
		precond.add("l", "*");
		precond.add("f", "^");
		precond.add("r", "*");
		Norm n3 = new Norm(precond, NormModality.Prohibition, CarAction.Go);
		n3.setId(3);
		
		
		/**
		 * Stop when there's no car at sight.
		 */
		precond = new NormPrecondition();
		precond.add("l", "-");
		precond.add("f", "-");
		precond.add("r", "-");
		Norm n4 = new Norm(precond, NormModality.Prohibition, CarAction.Go);
		n4.setId(4);
		
		/**
		 * Give way to left (generilised by n1).
		 */
		precond = new NormPrecondition();
		precond.add("l", ">");
		precond.add("f", ">");
		precond.add("r", ">");
		Norm n5 = new Norm(precond, NormModality.Prohibition, CarAction.Go);
		n5.setId(5);
		
		//10 agents with norm 1
		normProportion.add(15);
		//10 with no norm
		normProportion.add(45);
		
		//Add norm to the cars		
		
		for (Car c : map.getAllCars()){
				if(c.getId() <= 15){
					map.broadcastAddNorm(n1, c);
				}else{
					//map.broadcastAddNorm(n3, c);
					//no norm...
				}
		}
		
		/*map.broadcastAddNorm(n1);
		map.broadcastAddNorm(n2);
		map.broadcastAddNorm(n3);
		map.broadcastAddNorm(n4);
		map.broadcastAddNorm(n5);*/
	}
	
	/**
	 * 
	 */
	private boolean mustStop() {
		//Stop condition criterion.
		//If the norms fitness function don't change in x ticks?
		return false; 
	}
	
	/**their own method for traffic regulation
	 * Prints information about metrics in the current step
	 */
	private void printStepResume() {
		System.out.println("\nSTEP RESUME");
		System.out.println("----------------------------");
		System.out.println("Step: " + tick);
		CarMap map = this.getMap();

//		System.out.println("Car journeys:");
//		for (Car c : map.getAllCars()){
//			System.out.println("Car = "+c.getId());
//			System.out.println("Journey = "+c.getJourneys());
//		}
//		
//		System.out.println("Car collisions:");
//		for (Car c : map.getAllCars()){
//			if(c.getCollisions()!=0){
//				System.out.println("Car = "+c.getId());
//				System.out.println("Collision = "+c.getCollisions());
//			}
//		}
	}
	
	//-----------------------------------------------------------------
	// Static methods
	//-----------------------------------------------------------------

	/**
	 * 
	 * @return
	 */
	public static long getTick() {
		return tick;
	}

	/**
	 * 
	 * @return
	 */
	public static Random getRandom() {
		return random;
	}

	/**
	 * 
	 * @return
	 */
	public static CarMap getMap() {
		return carMap;
	}
	
	/**
	 * 
	 */
	public static TrafficConfiguration getConfig() {
		return config;
	}

  public int getX() {
	  return -1;
  }

  public int getY() {
	  return -1;
  }

  public void move() {
	  
  }
}

