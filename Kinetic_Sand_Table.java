import java.lang.*;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPin;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinPullResistance;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;


/*
Inputs: 2 switches
(2-pins)
Outputs: Speed of 2 4-step motors
(8-pins)
*/

public class Kinetic_Sand_Table
{
	final static GpioController gpio = GpioFactory.getInstance();

	public static double current_Polar[] = new double[2]; // [r,pheta]
	public static double max_Polar[] = new double[2]; // [units,degrees]
	public static int current_Steps[] = new int[2]; // [armStep, rotationStep]
	public static int max_Step[] = new int[2]; // [maxArmStep, maxRotationStep]
	public static boolean rotationBuffer[] = {true, false, false, false, false, false, false, false};
	public static boolean armBuffer[] = {true, false, false, false, false, false, false, false};

	public static double marbleVelocity;

	public static double armMmPerStepRatio;
	public static double rotationDegreePerStepRatio;
	public static double armVelocity;
	public static double tangentialVelocity;
	public static double angularVelocity;
	public static long armDelay;
	public static long rotationDelay;

	public static GpioPinDigitalInput rotationZeroButton = gpio.provisionDigitalInputPin(RaspiPin.GPIO_05, "rotationZeroButton", PinPullResistance.PULL_UP);
	public static GpioPinDigitalInput armZeroButton = gpio.provisionDigitalInputPin(RaspiPin.GPIO_04, "armZeroButton", PinPullResistance.PULL_UP);

	public static GpioPinDigitalOutput rotationMotor1 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_21, "rotationMotor1", PinState.LOW);
	public static GpioPinDigitalOutput rotationMotor2 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_22, "rotationMotor2", PinState.LOW);
	public static GpioPinDigitalOutput rotationMotor3 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_23, "rotationMotor3", PinState.LOW);
	public static GpioPinDigitalOutput rotationMotor4 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_24, "rotationMotor4", PinState.LOW);
	public static GpioPinDigitalOutput armMotor1 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_07, "armMotor1", PinState.LOW);
	public static GpioPinDigitalOutput armMotor2 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_00, "armMotor2", PinState.LOW);
	public static GpioPinDigitalOutput armMotor3 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02, "armMotor3", PinState.LOW);
	public static GpioPinDigitalOutput armMotor4 = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_03, "armMotor4", PinState.LOW);


	Kinetic_Sand_Table() //constructer not used because no objects
	{}

	public static void initialize () //acts like the constructer
	{
		Kinetic_Sand_Table.current_Polar[0] = 0; //r
		Kinetic_Sand_Table.current_Polar[1] = 0; //pheta
		Kinetic_Sand_Table.max_Polar[0] = 1000; //max r
		Kinetic_Sand_Table.max_Polar[1] = 360; //max pheta
		Kinetic_Sand_Table.current_Steps[0] = 50; //armStep
		Kinetic_Sand_Table.current_Steps[1] = 0; //rotationStep
		Kinetic_Sand_Table.max_Step[0] = 370; //maxArmStep
		Kinetic_Sand_Table.max_Step[1] = 384; //maxRotationStep

		Kinetic_Sand_Table.marbleVelocity = 6; // (mm/s)

		Kinetic_Sand_Table.rotationDegreePerStepRatio = 1.0666; // (degree/step)
		Kinetic_Sand_Table.tangentialVelocity = 0; // (mm/s)
		Kinetic_Sand_Table.angularVelocity = 0; // (rad/s)  Kinetic_Sand_Table.tangentialVelocity/(Kinetic_Sand_Table.current_Steps[0]*Kinetic_Sand_Table.armMmPerStepRatio);
		Kinetic_Sand_Table.rotationDelay = 0; // (ms/step)

		Kinetic_Sand_Table.armMmPerStepRatio = 1.3; // (mm/step)
		Kinetic_Sand_Table.armVelocity = 0; // (mm/s)
		Kinetic_Sand_Table.armDelay = 0; //(int) Math.round(1000*Kinetic_Sand_Table.armMmPerStepRatio/Kinetic_Sand_Table.armVelocity); // (ms/step)

		// Initializing Outputs
		Kinetic_Sand_Table.rotationMotor1.high();
		Kinetic_Sand_Table.rotationMotor2.low();
		Kinetic_Sand_Table.rotationMotor3.low();
		Kinetic_Sand_Table.rotationMotor4.low();
		Kinetic_Sand_Table.armMotor1.high();
		Kinetic_Sand_Table.armMotor2.low();
		Kinetic_Sand_Table.armMotor3.low();
		Kinetic_Sand_Table.armMotor4.low();

		// Initializing Shutdown Behavior
		Kinetic_Sand_Table.rotationZeroButton.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.armZeroButton.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.rotationMotor1.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.rotationMotor2.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.rotationMotor3.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.rotationMotor4.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.armMotor1.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.armMotor2.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.armMotor3.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Kinetic_Sand_Table.armMotor4.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
	}


	public static void main(String[] args)
	{
		long sleepDelay = 3000;

		Kinetic_Sand_Table.initialize();
		Kinetic_Sand_Table.firstReset();

		long startSleep = System.currentTimeMillis();
		while ((System.currentTimeMillis() - startSleep) < sleepDelay)
		{} // do nothing
		/*
		Kinetic_Sand_Table.spiral();

		startSleep = System.currentTimeMillis();
		while ((System.currentTimeMillis() - startSleep) < sleepDelay)
		{} // do nothing
		*/
		Kinetic_Sand_Table.alchemist();
	}


	public static void firstReset()
	{
		//run until the zero switches hit
		boolean finishedRotating = false;
		boolean finishedMovingArm = false;

		Kinetic_Sand_Table.tangentialVelocity = Kinetic_Sand_Table.marbleVelocity;
		Kinetic_Sand_Table.angularVelocity = Kinetic_Sand_Table.tangentialVelocity/(Kinetic_Sand_Table.current_Steps[0]*Kinetic_Sand_Table.armMmPerStepRatio);
		Kinetic_Sand_Table.rotationDelay = (long) (Kinetic_Sand_Table.rotationDegreePerStepRatio*(Math.PI/180)/Kinetic_Sand_Table.angularVelocity*1000);

		Kinetic_Sand_Table.armVelocity = Kinetic_Sand_Table.marbleVelocity;
		Kinetic_Sand_Table.armDelay = (long) (Kinetic_Sand_Table.armMmPerStepRatio/Kinetic_Sand_Table.armVelocity*1000);

		long rotationStartTime = System.currentTimeMillis();
		long armStartTime = System.currentTimeMillis();
		long currentTime = System.currentTimeMillis();

		System.out.println(Kinetic_Sand_Table.armDelay + " " + Kinetic_Sand_Table.rotationDelay);

		while((!finishedRotating) || (!finishedMovingArm))
		{
			currentTime = System.currentTimeMillis();
			if(Kinetic_Sand_Table.rotationZeroButton.isLow())
			{
				finishedRotating = true;
			}
			if(Kinetic_Sand_Table.armZeroButton.isLow())
			{
				finishedMovingArm = true;
			}
			if(!finishedRotating)
			{
				if ((currentTime - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
				{
					Kinetic_Sand_Table.rotateClockwiseHalfStep();
					rotationStartTime = currentTime;
				}
			}
			if(!finishedMovingArm)
			{
				if ((currentTime - armStartTime) > Kinetic_Sand_Table.armDelay)
				{
					Kinetic_Sand_Table.retractArmHalfStep();
					armStartTime = currentTime;
				}
			}
		}

		//reset values
		Kinetic_Sand_Table.current_Polar[0] = 5; //r
		Kinetic_Sand_Table.current_Polar[1] = 0; //pheta
		Kinetic_Sand_Table.current_Steps[0] = (int) (5/Kinetic_Sand_Table.max_Polar[0]*Kinetic_Sand_Table.max_Step[0]); //armStep
		Kinetic_Sand_Table.current_Steps[1] = 0; //rotationStep
	}

	public static void spiral()
	{
		// initial conditions for parametric equation
		double t = 200;
		double x = 5*t*Math.cos(t);
		double y = 5*t*Math.sin(t);

		double polar[] = Kinetic_Sand_Table.cartesianToPolar(x, y);

		Kinetic_Sand_Table.goToPolarCoordRoundThenOut(polar[0], polar[1]);

		for (t=200; t>=5; t=t-.01)// resolution
		{
			System.out.println(t + "  " + x + "  " + y);
			x = 5*t*Math.cos(t);
			y = 5*t*Math.sin(t);

			polar = Kinetic_Sand_Table.cartesianToPolar(x, y);

			Kinetic_Sand_Table.goToPolarCoordStraight(polar[0], polar[1]);
		}
	}

	public static void alchemist()
	{
		// initial conditions for parametric equation
		double t = -12.5;
		double x = 450*(1.1*Math.cos(t)-Math.cos(10.25*t));
		double y = 450*(1.1*Math.sin(t)-Math.sin(10.25*t));

		double polar[] = Kinetic_Sand_Table.cartesianToPolar(x, y);

		Kinetic_Sand_Table.goToPolarCoordRoundThenOut(polar[0], polar[1]);

		for (t=-12.5; t<=12.5; t=t+.01)// resolution
		{
			System.out.println(t + "  " + x + "  " + y);
			x = 450*(1.1*Math.cos(t)-Math.cos(10.25*t));
			y = 450*(1.1*Math.sin(t)-Math.sin(10.25*t));

			polar = Kinetic_Sand_Table.cartesianToPolar(x, y);

			Kinetic_Sand_Table.goToPolarCoordStraight(polar[0], polar[1]);
		}
	}

	public static void goToPolarCoordRoundThenOut(double r, double pheta)
	{
		int targetSteps[] = Kinetic_Sand_Table.polarToSteps(r, pheta);

		Kinetic_Sand_Table.tangentialVelocity = Kinetic_Sand_Table.marbleVelocity;
		Kinetic_Sand_Table.angularVelocity = Kinetic_Sand_Table.tangentialVelocity/(Kinetic_Sand_Table.current_Steps[0]*Kinetic_Sand_Table.armMmPerStepRatio);
		Kinetic_Sand_Table.rotationDelay = (long) (Kinetic_Sand_Table.rotationDegreePerStepRatio*(Math.PI/180)/Kinetic_Sand_Table.angularVelocity*1000);

		long rotationStartTime = System.currentTimeMillis();

		if (targetSteps[1] > Kinetic_Sand_Table.current_Steps[1]) //need to find the bigger angle to make it the shorter route
		{
			if ((targetSteps[1] - Kinetic_Sand_Table.current_Steps[1]) <= Kinetic_Sand_Table.max_Step[1]/2)
			{
				while (!(Kinetic_Sand_Table.current_Steps[1] == targetSteps[1]))
				{
					if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
					{
						Kinetic_Sand_Table.rotateCounterClockwiseHalfStep();
						rotationStartTime = System.currentTimeMillis();
					}
				}
			}
			else
			{
				while (!(Kinetic_Sand_Table.current_Steps[1] == targetSteps[1]))
				{
					if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
					{
						Kinetic_Sand_Table.rotateClockwiseHalfStep();
						rotationStartTime = System.currentTimeMillis();
					}
				}
			}
		}
		else if (targetSteps[1] < Kinetic_Sand_Table.current_Steps[1])
		{
			if ((Kinetic_Sand_Table.current_Steps[1] - targetSteps[1]) <= Kinetic_Sand_Table.max_Step[1]/2)
			{
				while (!(Kinetic_Sand_Table.current_Steps[1] == targetSteps[1]))
				{
					if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
					{
						Kinetic_Sand_Table.rotateClockwiseHalfStep();
						rotationStartTime = System.currentTimeMillis();
					}
				}
			}
			else
			{
				while (!(Kinetic_Sand_Table.current_Steps[1] == targetSteps[1]))
				{
					if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
					{
						Kinetic_Sand_Table.rotateCounterClockwiseHalfStep();
						rotationStartTime = System.currentTimeMillis();
					}
				}
			}
		}

		Kinetic_Sand_Table.armVelocity = Kinetic_Sand_Table.marbleVelocity;
		Kinetic_Sand_Table.armDelay = (long) (Kinetic_Sand_Table.armMmPerStepRatio/Kinetic_Sand_Table.armVelocity*1000);

		long armStartTime = System.currentTimeMillis();

		if (Kinetic_Sand_Table.current_Steps[0]<targetSteps[0])
		{
			while (!(Kinetic_Sand_Table.current_Steps[0] == targetSteps[0]))
			{
				if ((System.currentTimeMillis() - armStartTime) > Kinetic_Sand_Table.armDelay)
				{
					Kinetic_Sand_Table.extendArmHalfStep();
					armStartTime = System.currentTimeMillis();
				}
			}
		}
		if (Kinetic_Sand_Table.current_Steps[0]>targetSteps[0])
		{
			while (!(Kinetic_Sand_Table.current_Steps[0] == targetSteps[0]))
			{
				if ((System.currentTimeMillis() - armStartTime) > Kinetic_Sand_Table.armDelay)
				{
					Kinetic_Sand_Table.retractArmHalfStep();
					armStartTime = System.currentTimeMillis();
				}
			}
		}
	}

	public static void goToPolarCoordStraight(double r, double pheta)
	{
		double cartesian0[] = Kinetic_Sand_Table.polarToCartesian(Kinetic_Sand_Table.current_Polar[0], Kinetic_Sand_Table.current_Polar[1]);
		double cartesian1[] = Kinetic_Sand_Table.polarToCartesian(r, pheta);
		double x0 = cartesian0[0];
		double y0 = cartesian0[1];
		double x1 = cartesian1[0];
		double y1 = cartesian1[1];
		double x = x0;
		double y = y0;
		double dx = x1 - x0;
		double dy = y1 - y0;

		double directionVector[] = new double[2];
		directionVector[0] = x1-x0;
		directionVector[1] = y1-y0;
		double orthogonalVector[] = new double[2];
		orthogonalVector[0] = y0;
		orthogonalVector[1] = -x0;
		double angleBetween = Math.toDegrees(Math.acos((directionVector[0]*orthogonalVector[0]+directionVector[1]*orthogonalVector[1])/(Math.sqrt(directionVector[0]*directionVector[0]+directionVector[1]*directionVector[1])*Math.sqrt(orthogonalVector[0]*orthogonalVector[0]+orthogonalVector[1]*orthogonalVector[1]))));
		if (angleBetween > 90)
		{
			angleBetween = 180 - angleBetween;
		}

		int totalSteps;
		if (Math.abs(dx) > Math.abs(dy))
		{
			totalSteps = (int) Math.round(Math.abs(dx));
		}
		else
		{
			totalSteps = (int) Math.round(Math.abs(dy));
		}
		double Xincrement = dx / (float) totalSteps;
		double Yincrement = dy / (float) totalSteps;
		double polar[] = new double[2];
		int steps[] = new int[2];
		for(int v=0; v < totalSteps; v++)
		{
		   x = x + Xincrement;
		   y = y + Yincrement;
		   polar = Kinetic_Sand_Table.cartesianToPolar(x, y);
		   steps = Kinetic_Sand_Table.polarToSteps(polar[0], polar[1]);

		   Kinetic_Sand_Table.armVelocity = Kinetic_Sand_Table.marbleVelocity*Math.sin(Math.toRadians(angleBetween));
		   Kinetic_Sand_Table.armDelay = (long) (Kinetic_Sand_Table.armMmPerStepRatio/Kinetic_Sand_Table.armVelocity*1000);

		   Kinetic_Sand_Table.tangentialVelocity = Kinetic_Sand_Table.marbleVelocity*Math.cos(Math.toRadians(angleBetween));
		   Kinetic_Sand_Table.angularVelocity = Kinetic_Sand_Table.tangentialVelocity/(Kinetic_Sand_Table.current_Steps[0]*Kinetic_Sand_Table.armMmPerStepRatio);
		   Kinetic_Sand_Table.rotationDelay = (long) (Kinetic_Sand_Table.rotationDegreePerStepRatio*(Math.PI/180)/Kinetic_Sand_Table.angularVelocity*1000);

		   long armStartTime = System.currentTimeMillis();
		   long rotationStartTime = System.currentTimeMillis();

		   while (!(Kinetic_Sand_Table.current_Steps[0]==steps[0])||!(Kinetic_Sand_Table.current_Steps[1]==steps[1]))
		   {
			   if (Kinetic_Sand_Table.current_Steps[0] < steps[0])
			   {
				   if ((System.currentTimeMillis() - armStartTime) > Kinetic_Sand_Table.armDelay)
				   {
					   Kinetic_Sand_Table.extendArmHalfStep();
					   armStartTime = System.currentTimeMillis();
				   }
			   }
			   else if (Kinetic_Sand_Table.current_Steps[0] > steps[0])
			   {
				   if ((System.currentTimeMillis() - armStartTime) > Kinetic_Sand_Table.armDelay)
				   {
					   Kinetic_Sand_Table.retractArmHalfStep();
					   armStartTime = System.currentTimeMillis();
				   }
			   }

			   if (Math.abs(Kinetic_Sand_Table.current_Steps[1]-steps[1]) > (Kinetic_Sand_Table.max_Step[1]/2))
			   {
				   if (Kinetic_Sand_Table.current_Steps[1] > steps[1])
				   {
					   if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
					   {
						   Kinetic_Sand_Table.rotateCounterClockwiseHalfStep();
						   rotationStartTime = System.currentTimeMillis();
					   }
				   }
				   else if (Kinetic_Sand_Table.current_Steps[1] < steps[1])
				   {
					   if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
					   {
						   Kinetic_Sand_Table.rotateClockwiseHalfStep();
						   rotationStartTime = System.currentTimeMillis();
					   }
				   }
			   }
			   else if (Kinetic_Sand_Table.current_Steps[1] < steps[1])
			   {
				   if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
				   {
					   Kinetic_Sand_Table.rotateCounterClockwiseHalfStep();
					   rotationStartTime = System.currentTimeMillis();
				   }
			   }
			   else if (Kinetic_Sand_Table.current_Steps[1] > steps[1])
			   {
				   if ((System.currentTimeMillis() - rotationStartTime) > Kinetic_Sand_Table.rotationDelay)
				   {
					   Kinetic_Sand_Table.rotateClockwiseHalfStep();
					   rotationStartTime = System.currentTimeMillis();
				   }
			   }
		   }
		}

	}

	public static void extendArmHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Kinetic_Sand_Table.armBuffer[1];
		newArray[1] = Kinetic_Sand_Table.armBuffer[2];
		newArray[2] = Kinetic_Sand_Table.armBuffer[3];
		newArray[3] = Kinetic_Sand_Table.armBuffer[4];
		newArray[4] = Kinetic_Sand_Table.armBuffer[5];
		newArray[5] = Kinetic_Sand_Table.armBuffer[6];
		newArray[6] = Kinetic_Sand_Table.armBuffer[7];
		newArray[7] = Kinetic_Sand_Table.armBuffer[0];
		Kinetic_Sand_Table.armBuffer[0] = newArray[0];
		Kinetic_Sand_Table.armBuffer[1] = newArray[1];
		Kinetic_Sand_Table.armBuffer[2] = newArray[2];
		Kinetic_Sand_Table.armBuffer[3] = newArray[3];
		Kinetic_Sand_Table.armBuffer[4] = newArray[4];
		Kinetic_Sand_Table.armBuffer[5] = newArray[5];
		Kinetic_Sand_Table.armBuffer[6] = newArray[6];
		Kinetic_Sand_Table.armBuffer[7] = newArray[7];

		Kinetic_Sand_Table.current_Steps[0]++;
		Kinetic_Sand_Table.current_Polar = Kinetic_Sand_Table.stepsToPolar(Kinetic_Sand_Table.current_Steps[0], Kinetic_Sand_Table.current_Steps[1]);

		Kinetic_Sand_Table.updateArmPosition();
	}

	public static void retractArmHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Kinetic_Sand_Table.armBuffer[7];
		newArray[1] = Kinetic_Sand_Table.armBuffer[0];
		newArray[2] = Kinetic_Sand_Table.armBuffer[1];
		newArray[3] = Kinetic_Sand_Table.armBuffer[2];
		newArray[4] = Kinetic_Sand_Table.armBuffer[3];
		newArray[5] = Kinetic_Sand_Table.armBuffer[4];
		newArray[6] = Kinetic_Sand_Table.armBuffer[5];
		newArray[7] = Kinetic_Sand_Table.armBuffer[6];
		Kinetic_Sand_Table.armBuffer[0] = newArray[0];
		Kinetic_Sand_Table.armBuffer[1] = newArray[1];
		Kinetic_Sand_Table.armBuffer[2] = newArray[2];
		Kinetic_Sand_Table.armBuffer[3] = newArray[3];
		Kinetic_Sand_Table.armBuffer[4] = newArray[4];
		Kinetic_Sand_Table.armBuffer[5] = newArray[5];
		Kinetic_Sand_Table.armBuffer[6] = newArray[6];
		Kinetic_Sand_Table.armBuffer[7] = newArray[7];

		Kinetic_Sand_Table.current_Steps[0]--;
		Kinetic_Sand_Table.current_Polar = Kinetic_Sand_Table.stepsToPolar(Kinetic_Sand_Table.current_Steps[0], Kinetic_Sand_Table.current_Steps[1]);

		Kinetic_Sand_Table.updateArmPosition();
	}

	public static void updateArmPosition()
	{
		if(Kinetic_Sand_Table.armBuffer[0])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm0");
			Kinetic_Sand_Table.armMotor1.high();
			Kinetic_Sand_Table.armMotor2.low();
			Kinetic_Sand_Table.armMotor3.low();
			Kinetic_Sand_Table.armMotor4.low();
		}
		if(Kinetic_Sand_Table.armBuffer[1])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm1");
			Kinetic_Sand_Table.armMotor1.high();
			Kinetic_Sand_Table.armMotor2.high();
			Kinetic_Sand_Table.armMotor3.low();
			Kinetic_Sand_Table.armMotor4.low();
		}
		if(Kinetic_Sand_Table.armBuffer[2])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm2");
			Kinetic_Sand_Table.armMotor1.low();
			Kinetic_Sand_Table.armMotor2.high();
			Kinetic_Sand_Table.armMotor3.low();
			Kinetic_Sand_Table.armMotor4.low();
		}
		if(Kinetic_Sand_Table.armBuffer[3])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm3");
			Kinetic_Sand_Table.armMotor1.low();
			Kinetic_Sand_Table.armMotor2.high();
			Kinetic_Sand_Table.armMotor3.high();
			Kinetic_Sand_Table.armMotor4.low();
		}
		if(Kinetic_Sand_Table.armBuffer[4])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm4");
			Kinetic_Sand_Table.armMotor1.low();
			Kinetic_Sand_Table.armMotor2.low();
			Kinetic_Sand_Table.armMotor3.high();
			Kinetic_Sand_Table.armMotor4.low();
		}
		if(Kinetic_Sand_Table.armBuffer[5])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm5");
			Kinetic_Sand_Table.armMotor1.low();
			Kinetic_Sand_Table.armMotor2.low();
			Kinetic_Sand_Table.armMotor3.high();
			Kinetic_Sand_Table.armMotor4.high();
		}
		if(Kinetic_Sand_Table.armBuffer[6])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm6");
			Kinetic_Sand_Table.armMotor1.low();
			Kinetic_Sand_Table.armMotor2.low();
			Kinetic_Sand_Table.armMotor3.low();
			Kinetic_Sand_Table.armMotor4.high();
		}
		if(Kinetic_Sand_Table.armBuffer[7])
		{System.out.println(Kinetic_Sand_Table.current_Steps[0] + " arm7");
			Kinetic_Sand_Table.armMotor1.high();
			Kinetic_Sand_Table.armMotor2.low();
			Kinetic_Sand_Table.armMotor3.low();
			Kinetic_Sand_Table.armMotor4.high();
		}
	}

	public static void rotateClockwiseHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Kinetic_Sand_Table.rotationBuffer[7];
		newArray[1] = Kinetic_Sand_Table.rotationBuffer[0];
		newArray[2] = Kinetic_Sand_Table.rotationBuffer[1];
		newArray[3] = Kinetic_Sand_Table.rotationBuffer[2];
		newArray[4] = Kinetic_Sand_Table.rotationBuffer[3];
		newArray[5] = Kinetic_Sand_Table.rotationBuffer[4];
		newArray[6] = Kinetic_Sand_Table.rotationBuffer[5];
		newArray[7] = Kinetic_Sand_Table.rotationBuffer[6];
		Kinetic_Sand_Table.rotationBuffer[0] = newArray[0];
		Kinetic_Sand_Table.rotationBuffer[1] = newArray[1];
		Kinetic_Sand_Table.rotationBuffer[2] = newArray[2];
		Kinetic_Sand_Table.rotationBuffer[3] = newArray[3];
		Kinetic_Sand_Table.rotationBuffer[4] = newArray[4];
		Kinetic_Sand_Table.rotationBuffer[5] = newArray[5];
		Kinetic_Sand_Table.rotationBuffer[6] = newArray[6];
		Kinetic_Sand_Table.rotationBuffer[7] = newArray[7];

		Kinetic_Sand_Table.current_Steps[1]--;
		if(Kinetic_Sand_Table.current_Steps[1]<0)
		{
			Kinetic_Sand_Table.current_Steps[1] = Kinetic_Sand_Table.max_Step[1] - 1;
		}
		Kinetic_Sand_Table.current_Polar = Kinetic_Sand_Table.stepsToPolar(Kinetic_Sand_Table.current_Steps[0], Kinetic_Sand_Table.current_Steps[1]);

		Kinetic_Sand_Table.updateRotationPosition();
	}

	public static void rotateCounterClockwiseHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Kinetic_Sand_Table.rotationBuffer[1];
		newArray[1] = Kinetic_Sand_Table.rotationBuffer[2];
		newArray[2] = Kinetic_Sand_Table.rotationBuffer[3];
		newArray[3] = Kinetic_Sand_Table.rotationBuffer[4];
		newArray[4] = Kinetic_Sand_Table.rotationBuffer[5];
		newArray[5] = Kinetic_Sand_Table.rotationBuffer[6];
		newArray[6] = Kinetic_Sand_Table.rotationBuffer[7];
		newArray[7] = Kinetic_Sand_Table.rotationBuffer[0];
		Kinetic_Sand_Table.rotationBuffer[0] = newArray[0];
		Kinetic_Sand_Table.rotationBuffer[1] = newArray[1];
		Kinetic_Sand_Table.rotationBuffer[2] = newArray[2];
		Kinetic_Sand_Table.rotationBuffer[3] = newArray[3];
		Kinetic_Sand_Table.rotationBuffer[4] = newArray[4];
		Kinetic_Sand_Table.rotationBuffer[5] = newArray[5];
		Kinetic_Sand_Table.rotationBuffer[6] = newArray[6];
		Kinetic_Sand_Table.rotationBuffer[7] = newArray[7];

		Kinetic_Sand_Table.current_Steps[1]++;
		if(Kinetic_Sand_Table.current_Steps[1] == Kinetic_Sand_Table.max_Step[1])
		{
			Kinetic_Sand_Table.current_Steps[1] = 0;
		}
		Kinetic_Sand_Table.current_Polar = Kinetic_Sand_Table.stepsToPolar(Kinetic_Sand_Table.current_Steps[0], Kinetic_Sand_Table.current_Steps[1]);

		Kinetic_Sand_Table.updateRotationPosition();
	}

	public static void updateRotationPosition()
	{
		if(Kinetic_Sand_Table.rotationBuffer[0])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate0");
			Kinetic_Sand_Table.rotationMotor1.high();
			Kinetic_Sand_Table.rotationMotor2.low();
			Kinetic_Sand_Table.rotationMotor3.low();
			Kinetic_Sand_Table.rotationMotor4.low();
		}
		if(Kinetic_Sand_Table.rotationBuffer[1])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate1");
			Kinetic_Sand_Table.rotationMotor1.high();
			Kinetic_Sand_Table.rotationMotor2.high();
			Kinetic_Sand_Table.rotationMotor3.low();
			Kinetic_Sand_Table.rotationMotor4.low();
		}
		if(Kinetic_Sand_Table.rotationBuffer[2])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate2");
			Kinetic_Sand_Table.rotationMotor1.low();
			Kinetic_Sand_Table.rotationMotor2.high();
			Kinetic_Sand_Table.rotationMotor3.low();
			Kinetic_Sand_Table.rotationMotor4.low();
		}
		if(Kinetic_Sand_Table.rotationBuffer[3])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate3");
			Kinetic_Sand_Table.rotationMotor1.low();
			Kinetic_Sand_Table.rotationMotor2.high();
			Kinetic_Sand_Table.rotationMotor3.high();
			Kinetic_Sand_Table.rotationMotor4.low();
		}
		if(Kinetic_Sand_Table.rotationBuffer[4])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate4");
			Kinetic_Sand_Table.rotationMotor1.low();
			Kinetic_Sand_Table.rotationMotor2.low();
			Kinetic_Sand_Table.rotationMotor3.high();
			Kinetic_Sand_Table.rotationMotor4.low();
		}
		if(Kinetic_Sand_Table.rotationBuffer[5])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate5");
			Kinetic_Sand_Table.rotationMotor1.low();
			Kinetic_Sand_Table.rotationMotor2.low();
			Kinetic_Sand_Table.rotationMotor3.high();
			Kinetic_Sand_Table.rotationMotor4.high();
		}
		if(Kinetic_Sand_Table.rotationBuffer[6])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate6");
			Kinetic_Sand_Table.rotationMotor1.low();
			Kinetic_Sand_Table.rotationMotor2.low();
			Kinetic_Sand_Table.rotationMotor3.low();
			Kinetic_Sand_Table.rotationMotor4.high();
		}
		if(Kinetic_Sand_Table.rotationBuffer[7])
		{System.out.println(Kinetic_Sand_Table.current_Steps[1] + " rotate7");
			Kinetic_Sand_Table.rotationMotor1.high();
			Kinetic_Sand_Table.rotationMotor2.low();
			Kinetic_Sand_Table.rotationMotor3.low();
			Kinetic_Sand_Table.rotationMotor4.high();
		}
	}

	public static double[] stepsToPolar(int armSteps, int rotationSteps)
	{
		double polar[] = new double[2]; // [r, pheta]
		polar[0] = (armSteps/(double) Kinetic_Sand_Table.max_Step[0])*Kinetic_Sand_Table.max_Polar[0];
		polar[1] = (rotationSteps/(double) Kinetic_Sand_Table.max_Step[1])*Kinetic_Sand_Table.max_Polar[1];
		return polar;
	}

	public static int[] polarToSteps(double r, double pheta)
	{
		int steps[] = new int[2]; // [armStep, rotationStep]
		steps[0] = (int) Math.round((r/Kinetic_Sand_Table.max_Polar[0])*Kinetic_Sand_Table.max_Step[0]);
		steps[1] = (int) Math.round((pheta/Kinetic_Sand_Table.max_Polar[1])*Kinetic_Sand_Table.max_Step[1]);
		if (steps[1] == Kinetic_Sand_Table.max_Step[1])
		{
			steps[1] = 0;
		}
		return steps;
	}

	public static double[] cartesianToPolar(double x, double y)
	{
		double polar[] = new double[2]; // [r,pheta]
		polar[0] = Math.sqrt(x*x + y*y);
		polar[1] = Math.toDegrees(Math.atan2(y, x));
		if (polar[1] < 0)
		{
			polar[1] = polar[1] + 360;
		}
		return polar;
	}

	public static double[] polarToCartesian(double r, double pheta)
	{
		double cartesian[] = new double[2]; // [x,y]
		cartesian[0] = r*Math.cos(Math.toRadians(pheta));
		cartesian[1] = r*Math.sin(Math.toRadians(pheta));
		return cartesian;
	}
	
	public static void sleep(int waitTime)
	{
		try
		{
			Thread.sleep(waitTime);
		}
		catch (Exception e)
		{
			System.out.println(e);
		}
	}

}
