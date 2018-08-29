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

public class Understanding
{
	final static GpioController gpio = GpioFactory.getInstance();

	public static double current_Polar[] = new double[2]; // [r,pheta]
	public static double max_Polar[] = new double[2]; //Units????
	public static int current_Steps[] = new int[2]; // [armStep, rotationStep]
	public static int max_Step[] = new int[2]; // [maxArmStep, maxRotationStep]
	public static boolean rotationBuffer[] = {true, false, false, false, false, false, false, false};
	public static boolean armBuffer[] = {true, false, false, false, false, false, false, false};
	
	public static double velocity;
	
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

		
	Understanding()
	{
		
	}
	
	public static void initialize ()
	{
		Understanding.current_Polar[0] = 0; //r
		Understanding.current_Polar[1] = 0; //pheta
		Understanding.max_Polar[0] = 1000; //max r
		Understanding.max_Polar[1] = 360; //max pheta
		Understanding.current_Steps[0] = 0; //armStep
		Understanding.current_Steps[1] = 0; //rotationStep
		Understanding.max_Step[0] = 5000; //maxArmStep
		Understanding.max_Step[1] = 768; //maxRotationStep
		
		Understanding.velocity = 4; //(mm/s)
		
		// Initializing Outputs
		Understanding.rotationMotor1.high();
		Understanding.rotationMotor2.low();
		Understanding.rotationMotor3.low();
		Understanding.rotationMotor4.low();
		Understanding.armMotor1.high();
		Understanding.armMotor2.low();
		Understanding.armMotor3.low();
		Understanding.armMotor4.low();
		
		// Initializing Shutdown Behavior
		Understanding.rotationZeroButton.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.armZeroButton.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.rotationMotor1.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.rotationMotor2.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.rotationMotor3.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.rotationMotor4.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.armMotor1.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.armMotor2.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.armMotor3.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
		Understanding.armMotor4.setShutdownOptions(true, PinState.LOW, PinPullResistance.OFF);
	}
	
	
	public static void main(String[] args)
	{
		Understanding.initialize();
		Understanding.firstReset();
		Understanding.sleep(3000);
		Understanding.test();
	}
	
	
	
	
	public static void test()
	{
		boolean finishedMovingArm = false;
		int waitTime = 4; //ms
		while(!finishedMovingArm)
		{
			if(Understanding.armZeroButton.isLow())
			{
				finishedMovingArm = true;
			}
			if(!finishedMovingArm)
			{
				extendArmHalfStep();
			}
			
			Understanding.sleep(waitTime);
		}
		System.out.println(Understanding.current_Steps[0]);
	}
	
	public static void firstReset()
	{
		//run until the zero switches hit
		boolean finishedRotating = false;
		boolean finishedMovingArm = false;
		int waitTime = 4; //ms
		
		while((!finishedRotating) || (!finishedMovingArm))
		{
			if(Understanding.rotationZeroButton.isLow())
			{
				finishedRotating = true;
			}
			if(Understanding.armZeroButton.isLow())
			{
				finishedMovingArm = true;
			}
			if(!finishedRotating)
			{
				Understanding.rotateClockwiseHalfStep();
			}
			if(!finishedMovingArm)
			{
				Understanding.retractArmHalfStep();
			}
			
			Understanding.sleep(waitTime);
		}
		
		//reset values
		Understanding.current_Polar[0] = 0; //r
		Understanding.current_Polar[1] = 0; //pheta
		Understanding.current_Steps[0] = 0; //armStep
		Understanding.current_Steps[1] = 0; //rotationStep
	}
	
	public static void spiral()
	{
		// initial conditions for parametric equation
		double t = 200;
		double x = 5*t*Math.cos(t);
		double y = 5*t*Math.sin(t);
		
		double polar[] = Understanding.cartesianToPolar(x, y);
		
		Understanding.goToPolarCoordRoundThenOut(polar[0], polar[1]);
		
		for (t=200; t>=5; t=t-.01)// resolution
		{
			System.out.println(t + "  " + x + "  " + y);
			x = 5*t*Math.cos(t);
			y = 5*t*Math.sin(t);
			
			polar = Understanding.cartesianToPolar(x, y);
			
			Understanding.goToPolarCoordStraight(polar[0], polar[1]);
		}
	}
	
	public static void alchemist()
	{
		// initial conditions for parametric equation
		double t = -12.5;
		double x = 450*(1.1*Math.cos(t)-Math.cos(10.25*t));
		double y = 450*(1.1*Math.sin(t)-Math.sin(10.25*t));
		
		double polar[] = Understanding.cartesianToPolar(x, y);
		
		Understanding.goToPolarCoordRoundThenOut(polar[0], polar[1]);
		
		for (t=-12.5; t<=12.5; t=t+.01)// resolution
		{
			System.out.println(t + "  " + x + "  " + y);
			x = 450*(1.1*Math.cos(t)-Math.cos(10.25*t));
			y = 450*(1.1*Math.sin(t)-Math.sin(10.25*t));
			
			polar = Understanding.cartesianToPolar(x, y);
			
			Understanding.goToPolarCoordStraight(polar[0], polar[1]);
		}
	}

	public static void goToPolarCoordRoundThenOut(double r, double pheta)
	{
		int waitTime = 4; //ms
		int targetSteps[] = Understanding.polarToSteps(r, pheta);
		
		if (targetSteps[1] > Understanding.current_Steps[1]) //need to find the bigger angle to make it the shorter route
		{
			if ((targetSteps[1] - Understanding.current_Steps[1]) <= Understanding.max_Step[1]/2)
			{
				while (!(Understanding.current_Steps[1] == targetSteps[1]))
				{
					Understanding.rotateCounterClockwiseHalfStep();
					Understanding.sleep(waitTime);
				}
			}
			else
			{
				while (!(Understanding.current_Steps[1] == targetSteps[1]))
				{
					Understanding.rotateClockwiseHalfStep();
					Understanding.sleep(waitTime);
				}
			}
		}
		if (targetSteps[1] < Understanding.current_Steps[1])
		{
			if ((Understanding.current_Steps[1] - targetSteps[1]) <= Understanding.max_Step[1]/2)
			{
				while (!(Understanding.current_Steps[1] == targetSteps[1]))
				{
					Understanding.rotateClockwiseHalfStep();
					Understanding.sleep(waitTime);
				}
			}
			else
			{
				while (!(Understanding.current_Steps[1] == targetSteps[1]))
				{
					Understanding.rotateCounterClockwiseHalfStep();
					Understanding.sleep(waitTime);
				}
			}
		}
		
		
		if (Understanding.current_Steps[0]<targetSteps[0])
		{
			while (!(Understanding.current_Steps[0] == targetSteps[0]))
			{
				Understanding.extendArmHalfStep();
				Understanding.sleep(waitTime);
			}
		}
		if (Understanding.current_Steps[0]>targetSteps[0])
		{
			while (!(Understanding.current_Steps[0] == targetSteps[0]))
			{
				Understanding.retractArmHalfStep();
				Understanding.sleep(waitTime);
			}
		}
	}
	
	public static void goToPolarCoordStraight(double r, double pheta)
	{
		int waitTime = 4; //ms
		
		double cartesian0[] = Understanding.polarToCartesian(Understanding.current_Polar[0], Understanding.current_Polar[1]);
		double cartesian1[] = Understanding.polarToCartesian(r, pheta);
		double x0 = cartesian0[0];
		double y0 = cartesian0[1];
		double x1 = cartesian1[0];
		double y1 = cartesian1[1];
		double x = x0;
		double y = y0;
		double dx = x1 - x0;
		double dy = y1 - y0;
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
		   polar = Understanding.cartesianToPolar(x, y);
		   steps = Understanding.polarToSteps(polar[0], polar[1]);
		   while (!(Understanding.current_Steps[0]==steps[0])||!(Understanding.current_Steps[1]==steps[1]))
		   {
			   if (Understanding.current_Steps[0] < steps[0])
			   {
				   Understanding.extendArmHalfStep();
			   }
			   else if (Understanding.current_Steps[0] > steps[0])
			   {
				   Understanding.retractArmHalfStep();
			   }
			   if (Math.abs(Understanding.current_Steps[1]-steps[1]) > Understanding.max_Step[1]/2)
			   {
				   if (Understanding.current_Steps[1] > steps[1])
				   {
					   Understanding.rotateCounterClockwiseHalfStep();
				   }
				   else if (Understanding.current_Steps[1] < steps[1])
				   {
					   Understanding.rotateClockwiseHalfStep();
				   }
			   }
			   else if (Understanding.current_Steps[1] < steps[1])
			   {
				   Understanding.rotateCounterClockwiseHalfStep();
			   }
			   else if (Understanding.current_Steps[1] > steps[1])
			   {
				   Understanding.rotateClockwiseHalfStep();
			   }
			   Understanding.sleep(waitTime);
		   }
		}
		
	}
	
	public static void extendArmHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Understanding.armBuffer[1];
		newArray[1] = Understanding.armBuffer[2];
		newArray[2] = Understanding.armBuffer[3];
		newArray[3] = Understanding.armBuffer[4];
		newArray[4] = Understanding.armBuffer[5];
		newArray[5] = Understanding.armBuffer[6];
		newArray[6] = Understanding.armBuffer[7];
		newArray[7] = Understanding.armBuffer[0];
		Understanding.armBuffer[0] = newArray[0];
		Understanding.armBuffer[1] = newArray[1];
		Understanding.armBuffer[2] = newArray[2];
		Understanding.armBuffer[3] = newArray[3];
		Understanding.armBuffer[4] = newArray[4];
		Understanding.armBuffer[5] = newArray[5];
		Understanding.armBuffer[6] = newArray[6];
		Understanding.armBuffer[7] = newArray[7];
		
		Understanding.current_Steps[0]++;
		Understanding.current_Polar = Understanding.stepsToPolar(Understanding.current_Steps[0], Understanding.current_Steps[1]);
		
		if(Understanding.armBuffer[0])
		{
			Understanding.armMotor1.high();
			Understanding.armMotor2.low();
			Understanding.armMotor3.low();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[1])
		{
			Understanding.armMotor1.high();
			Understanding.armMotor2.high();
			Understanding.armMotor3.low();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[2])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.high();
			Understanding.armMotor3.low();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[3])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.high();
			Understanding.armMotor3.high();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[4])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.low();
			Understanding.armMotor3.high();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[5])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.low();
			Understanding.armMotor3.high();
			Understanding.armMotor4.high();
		}
		if(Understanding.armBuffer[6])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.low();
			Understanding.armMotor3.low();
			Understanding.armMotor4.high();
		}
		if(Understanding.armBuffer[7])
		{
			Understanding.armMotor1.high();
			Understanding.armMotor2.low();
			Understanding.armMotor3.low();
			Understanding.armMotor4.high();
		}
	}
	
	public static void retractArmHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Understanding.armBuffer[7];
		newArray[1] = Understanding.armBuffer[0];
		newArray[2] = Understanding.armBuffer[1];
		newArray[3] = Understanding.armBuffer[2];
		newArray[4] = Understanding.armBuffer[3];
		newArray[5] = Understanding.armBuffer[4];
		newArray[6] = Understanding.armBuffer[5];
		newArray[7] = Understanding.armBuffer[6];
		Understanding.armBuffer[0] = newArray[0];
		Understanding.armBuffer[1] = newArray[1];
		Understanding.armBuffer[2] = newArray[2];
		Understanding.armBuffer[3] = newArray[3];
		Understanding.armBuffer[4] = newArray[4];
		Understanding.armBuffer[5] = newArray[5];
		Understanding.armBuffer[6] = newArray[6];
		Understanding.armBuffer[7] = newArray[7];
		
		Understanding.current_Steps[0]--;
		Understanding.current_Polar = Understanding.stepsToPolar(Understanding.current_Steps[0], Understanding.current_Steps[1]);
		
		if(Understanding.armBuffer[0])
		{
			Understanding.armMotor1.high();
			Understanding.armMotor2.low();
			Understanding.armMotor3.low();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[1])
		{
			Understanding.armMotor1.high();
			Understanding.armMotor2.low();
			Understanding.armMotor3.low();
			Understanding.armMotor4.high();
		}
		if(Understanding.armBuffer[2])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.low();
			Understanding.armMotor3.low();
			Understanding.armMotor4.high();
		}
		if(Understanding.armBuffer[3])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.low();
			Understanding.armMotor3.high();
			Understanding.armMotor4.high();
		}
		if(Understanding.armBuffer[4])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.low();
			Understanding.armMotor3.high();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[5])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.high();
			Understanding.armMotor3.high();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[6])
		{
			Understanding.armMotor1.low();
			Understanding.armMotor2.high();
			Understanding.armMotor3.low();
			Understanding.armMotor4.low();
		}
		if(Understanding.armBuffer[7])
		{
			Understanding.armMotor1.high();
			Understanding.armMotor2.high();
			Understanding.armMotor3.low();
			Understanding.armMotor4.low();
		}
	}
	
	public static void rotateClockwiseHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Understanding.rotationBuffer[7];
		newArray[1] = Understanding.rotationBuffer[0];
		newArray[2] = Understanding.rotationBuffer[1];
		newArray[3] = Understanding.rotationBuffer[2];
		newArray[4] = Understanding.rotationBuffer[3];
		newArray[5] = Understanding.rotationBuffer[4];
		newArray[6] = Understanding.rotationBuffer[5];
		newArray[7] = Understanding.rotationBuffer[6];
		Understanding.rotationBuffer[0] = newArray[0];
		Understanding.rotationBuffer[1] = newArray[1];
		Understanding.rotationBuffer[2] = newArray[2];
		Understanding.rotationBuffer[3] = newArray[3];
		Understanding.rotationBuffer[4] = newArray[4];
		Understanding.rotationBuffer[5] = newArray[5];
		Understanding.rotationBuffer[6] = newArray[6];
		Understanding.rotationBuffer[7] = newArray[7];
		
		Understanding.current_Steps[1]--;
		if(Understanding.current_Steps[1]<0)
		{
			Understanding.current_Steps[1] = Understanding.max_Step[1] - 1;
		}
		Understanding.current_Polar = Understanding.stepsToPolar(Understanding.current_Steps[0], Understanding.current_Steps[1]);
		
		if(Understanding.rotationBuffer[0])
		{
			Understanding.rotationMotor1.high();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[1])
		{
			Understanding.rotationMotor1.high();
			Understanding.rotationMotor2.high();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[2])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.high();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[3])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.high();
			Understanding.rotationMotor3.high();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[4])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.high();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[5])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.high();
			Understanding.rotationMotor4.high();
		}
		if(Understanding.rotationBuffer[6])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.high();
		}
		if(Understanding.rotationBuffer[7])
		{
			Understanding.rotationMotor1.high();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.high();
		}
	}
	
	public static void rotateCounterClockwiseHalfStep()
	{
		boolean newArray[]= new boolean[8];
		newArray[0] = Understanding.rotationBuffer[1];
		newArray[1] = Understanding.rotationBuffer[2];
		newArray[2] = Understanding.rotationBuffer[3];
		newArray[3] = Understanding.rotationBuffer[4];
		newArray[4] = Understanding.rotationBuffer[5];
		newArray[5] = Understanding.rotationBuffer[6];
		newArray[6] = Understanding.rotationBuffer[7];
		newArray[7] = Understanding.rotationBuffer[0];
		Understanding.rotationBuffer[0] = newArray[0];
		Understanding.rotationBuffer[1] = newArray[1];
		Understanding.rotationBuffer[2] = newArray[2];
		Understanding.rotationBuffer[3] = newArray[3];
		Understanding.rotationBuffer[4] = newArray[4];
		Understanding.rotationBuffer[5] = newArray[5];
		Understanding.rotationBuffer[6] = newArray[6];
		Understanding.rotationBuffer[7] = newArray[7];
		
		Understanding.current_Steps[1]++;
		if(Understanding.current_Steps[1] == Understanding.max_Step[1])
		{
			Understanding.current_Steps[1] = 0;
		}
		Understanding.current_Polar = Understanding.stepsToPolar(Understanding.current_Steps[0], Understanding.current_Steps[1]);
		
		if(Understanding.rotationBuffer[0])
		{
			Understanding.rotationMotor1.high();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[1])
		{
			Understanding.rotationMotor1.high();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.high();
		}
		if(Understanding.rotationBuffer[2])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.high();
		}
		if(Understanding.rotationBuffer[3])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.high();
			Understanding.rotationMotor4.high();
		}
		if(Understanding.rotationBuffer[4])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.low();
			Understanding.rotationMotor3.high();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[5])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.high();
			Understanding.rotationMotor3.high();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[6])
		{
			Understanding.rotationMotor1.low();
			Understanding.rotationMotor2.high();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.low();
		}
		if(Understanding.rotationBuffer[7])
		{
			Understanding.rotationMotor1.high();
			Understanding.rotationMotor2.high();
			Understanding.rotationMotor3.low();
			Understanding.rotationMotor4.low();
		}
	}
	
	public static double[] stepsToPolar(int armSteps, int rotationSteps)
	{
		double polar[] = new double[2]; // [r, pheta]
		polar[0] = (armSteps/(double) Understanding.max_Step[0])*Understanding.max_Polar[0];
		polar[1] = (rotationSteps/(double) Understanding.max_Step[1])*Understanding.max_Polar[1];
		return polar;
	}
	
	public static int[] polarToSteps(double r, double pheta)
	{
		int steps[] = new int[2]; // [armStep, rotationStep]
		steps[0] = (int) Math.round((r/Understanding.max_Polar[0])*Understanding.max_Step[0]);
		steps[1] = (int) Math.round((pheta/Understanding.max_Polar[1])*Understanding.max_Step[1]);
		if (steps[1] == Understanding.max_Step[1])
		{
			steps[1] = 0;
		}
		return steps;
	}
	
	public static double[] cartesianToPolar(double x, double y)
	{
		double polar[] = new double[2]; // [r,pheta]
		polar[0] = Math.sqrt(x*x + y*y);
		polar[1] = Math.atan2(y, x);
		polar[1] = polar[1]*180/Math.PI;
		if (polar[1] < 0)
		{
			polar[1] = polar[1] + 360;
		}
		return polar;
	}
	
	public static double[] polarToCartesian(double r, double pheta)
	{
		double cartesian[] = new double[2]; // [x,y]
		cartesian[0] = r*Math.cos(pheta*Math.PI/180);
		cartesian[1] = r*Math.sin(pheta*Math.PI/180);
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
