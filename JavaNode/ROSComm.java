package application;
 
import com.google.flatbuffers.FlatBufferBuilder;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixBuilder;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.ServoMotion;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
// For pinging
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.nio.ByteBuffer;
import org.zeromq.ZMQ;
import ros_kuka.flatbuffer.*;
import kuka_joints.flatbuffer.*;
import kuka_joints.flatbuffer.Quaternion;
import kuka_joints.flatbuffer.Vector3;
/**
 * Requests a point and moves straight towards it. 
 */
 
 
public class ROSComm extends RoboticsAPIApplication {
    //private Controller kuka_Sunrise_Cabinet;
	//private Controller controller;
    private LBR lbr;
    
    double[] homePose = new double[] {Math.toRadians(-90),  Math.toRadians(90), 0, Math.toRadians(0), Math.toRadians(0), Math.toRadians(0), 0};
    double[] teleopPose = new double[] {Math.toRadians(0), Math.toRadians(90), 0, Math.toRadians(-60), Math.toRadians(0), Math.toRadians(90), 0}; // ready for teleoperation
    Frame startFrame = null;
    Frame destFrame = null;
    

    // Tool Data 
    private Tool toolAttachedToLBR;
    private LoadData loadData;
    private final String toolFrame = "toolFrame";
    private final double[] translationOfTool = { 0, 0, 0 };
    private final double mass = 1;
    private final double[] centerOfMassInMillimeter = { 0, 0, 0 };
    boolean isHome=false;
 
 // ZMQ
    ZMQ.Context context = ZMQ.context(1);
    ZMQ.Socket subscriber = context.socket(ZMQ.SUB);
    ZMQ.Socket publisher = context.socket(ZMQ.PUB);

  // Pose Monitoring
    double[] jointValues = new double[]{};
     double[] quat = new double[]{};
    double updateCounter = 0; 
    private static final int UPDATERATE = 100; //  msec
 // Samrt Servo
 private SmartServo SSMotion = null;
  ISmartServoRuntime theServoRuntime = null;

    public void initialize() {
    	//controller = (SunriseController) getContext().getDefaultController();
        lbr = getContext().getDeviceFromType(LBR.class);

        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        loadData = new LoadData();
        loadData.setMass(mass);
        loadData.setCenterOfMass(centerOfMassInMillimeter[0], centerOfMassInMillimeter[1],centerOfMassInMillimeter[2]);
        toolAttachedToLBR = new Tool("Tool", loadData);
        toolAttachedToLBR.attachTo(lbr.getFlange());

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(translationOfTool[0], translationOfTool[1],translationOfTool[2]);
        ObjectFrame aTransformation = toolAttachedToLBR.addChildFrame(toolFrame+ "(TCP)", trans);
        toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        toolAttachedToLBR.attachTo(lbr.getFlange());

        /*-----------------------------------------------------------------------------------------------------------------------*/
        //       How to define a user key:
        /*-----------------------------------------------------------------------------------------------------------------------*/

               IUserKeyBar calibrationBar = getApplicationUI().createUserKeyBar("SP");
               IUserKeyListener keyListener = new IUserKeyListener(){
       			@Override
       			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
       				if(event == UserKeyEvent.KeyDown){
       					if(isHome==true){ // move to teleoperation pose
       						getLogger().info("Move to Teleoperation Pose!");  
       						try{
       						toolAttachedToLBR.moveAsync((ptp(teleopPose).setJointVelocityRel(0.6)));
       						}
       						catch(Exception e)
       						{
       							System.out.println("error: "+e.toString());
       						}
       						key.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Yellow, UserKeyLEDSize.Normal);
       						isHome=false;return;
       						
       					
       					}
       					else{
       						getLogger().info("Move to Home Pose!");
       						toolAttachedToLBR.moveAsync(ptp(homePose).setJointVelocityRel(0.6));
       						key.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Green, UserKeyLEDSize.Normal);
       						isHome=true;return;
       					}
       					

       					
       					}
       					else if(event == UserKeyEvent.KeyUp){
       					}
       			}
               	};
               	
               IUserKey calibKey = calibrationBar.addUserKey(0, keyListener, true);
               calibKey.setEnabled(true);
               calibKey.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Yellow, UserKeyLEDSize.Normal);
               calibrationBar.publish();
               /*-----------------------------------------------------------------------------------------------------------------------*/
   
        
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is
     * collision free.
     */
    private void moveToInitialPosition()
    {
        toolAttachedToLBR.move(ptp(teleopPose).setJointVelocityRel(0.8));
        isHome=true;
        /*
         * Note: The Validation itself justifies, that in this very time
         * instance, the load parameter setting was sufficient. This does not
         * mean by far, that the parameter setting is valid in the sequel or
         * lifetime of this program
         */
        try
        {
            if (!ServoMotion.validateForImpedanceMode(toolAttachedToLBR))
            {
                getLogger().info("Validation of torque model failed - correct your mass property settings");
                getLogger().info("SmartServo will be available for position controlled mode only, until validation is performed");
            }
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }
 
    
/*****************************************************************************************************/
// Main Loop    
/*****************************************************************************************************/
    public void run() {
//back to home
        moveToInitialPosition();
        getLogger().info("Request position from ROS, moves there if possible and send the joints values");  
       
        /*Smart Servo*/
        
        SSMotion = new SmartServo(lbr.getCurrentJointPosition());
        SSMotion.useTrace(true);
        SSMotion.setMinimumTrajectoryExecutionTime(5e-2);
        SSMotion.setJointAccelerationRel(0.1);
        SSMotion.setJointVelocityRel(0.3);
      //   SSMotion.overrideJointAcceleration(5); 
               
        getLogger().info("Starting the SmartServo");
        toolAttachedToLBR.moveAsync(SSMotion);
        
        theServoRuntime = SSMotion.getRuntime();
        theServoRuntime.updateWithRealtimeSystem();
       // Transformation t = lbr.getFlange().getTransformationFromParent();
        startFrame = theServoRuntime.getCurrentCartesianDestination(toolAttachedToLBR.getDefaultMotionFrame());
        Transformation t = startFrame.getTransformationFromParent();
        
        System.out.println("-------------------");
        System.out.println(t);
       for(int r=0;r<3;r++)
       {
    	   for(int c=0;c<3;c++)
    	   {
    		   System.out.print(String.format( "%.4f", t.getRotation().getMatrix().get(r, c))+", ");
    	   }
           System.out.println("");
       }
       System.out.println("Red:"+startFrame.getRedundancyInformation());

       quat=matrixToQuat(t.getRotationMatrix());
        System.out.println("Current Tool Position [From Parent]: [ <X ^Z xY ]");
        System.out.println("Cartesian: " + "[" +String.format( "%.2f",  t.getX() )+ "," +String.format( "%.2f",  t.getY() )+ ","  + String.format( "%.2f",  t.getZ() )+ "]");
        System.out.println("ABC Orientation: " + "[" +String.format( "%.2f", t.getAlphaRad())+ "," +String.format( "%.2f",  t.getBetaRad())+ ","  +String.format( "%.2f",  t.getGammaRad())+ "]");
        System.out.println("Quaternion: " + "[" +String.format( "%.2f",  quat[0])+ "," +String.format( "%.2f",  quat[1])+ "," +String.format( "%.2f", quat[2])+ "," + String.format( "%.2f", quat[3])+"]");
        System.out.println("==============================");
    testping();//ok ubuntu
    listenAndSpeak(); //zeromq in PUB-SUB mode
    }
    /*****************************************************************************************************/
 
    private void listenAndSpeak() {
  
        String ADDRESS_S = "tcp://172.31.1.1:5555";  /// subscriber
        String ADDRESS_P = "tcp://172.31.1.1:5558"; // publisher
        //  Socket to subscribe to publisher
        System.out.println("Connecting to KUKA ROS Node...");
        subscriber.connect(ADDRESS_S);
        publisher.connect(ADDRESS_P);
        DelayMs(1000);
        String filter = "";
        subscriber.subscribe(filter.getBytes());
        System.out.println("Subscriber: "+ADDRESS_S);
        System.out.println("Publisher: "+ADDRESS_P);
        System.out.println("-----KUKA READY!-----: ");
        try{
            long startTime = System.currentTimeMillis();
            updateCounter= System.currentTimeMillis();
// MAIN LOOP -----------------------------------------------------------------------------------------------            
            while ((System.currentTimeMillis() - startTime) < 600000) {
                recieveAndAct();
              if((System.currentTimeMillis() - updateCounter) >= UPDATERATE){publishRobotState();updateCounter = System.currentTimeMillis();}	 
 
            }//while
        }
        catch (Exception e){ e.printStackTrace();} 
        finally{
            System.out.println("Exiting.");
            subscriber.close();
            publisher.close();
            context.term();
            theServoRuntime.stopMotion();

        }
     return;
    }
    /*****************************************************************************************************/
private void recieveAndAct(){
    /*------------------------------------------------------------------*/
    // Receive response
    /*------------------------------------------------------------------*/
    byte[] data;
    try{data = subscriber.recv(ZMQ.NOBLOCK); }// non blocking receive
     
    catch (Exception e){return;}      // Repeat if something goes wrong.
    if (data != null) { // Repeat if no response (e.g. server not yet started) // in non blocking process if valid
 
    ByteBuffer bb = ByteBuffer.wrap(data);
    kukaDesPose posemsg = kukaDesPose.getRootAskukaDesPose(bb);
	byte mode = posemsg.control(); // get the control mode
 
switch (mode) {
case ros_kuka.flatbuffer.ControlMode.Joints:
    //TODO 
	jointMove(posemsg);
    break;
     
case ros_kuka.flatbuffer.ControlMode.CartesianPTP:
    cartPTPMove(posemsg);
    break;
     
case ros_kuka.flatbuffer.ControlMode.CartesianSS:
  cartSSMove(posemsg);
    break;
     
case ros_kuka.flatbuffer.ControlMode.CartesianDS:
    //TODO 
    break;
 
case ros_kuka.flatbuffer.ControlMode.BackHome: // back to home
	System.out.println("Back to home!.");
    toolAttachedToLBR.move(ptp(homePose).setJointVelocityRel(0.8));
    break;
    
    default:
    	System.out.println("Are you kidding !!");
    	break;
}
 
    }//data-!null
    return;
}
/*****************************************************************************************************/
private void publishRobotState(){
    /*------------------------------------------------------------------*/
    // Send response
    /*------------------------------------------------------------------*/
    try{
        jointValues=  lbr.getCurrentJointPosition().get();
        theServoRuntime.updateWithRealtimeSystem();
        Transformation t = theServoRuntime.getCurrentCartesianPosition(toolAttachedToLBR.getDefaultMotionFrame()).getTransformationFromParent();
        quat=matrixToQuat(t.getRotationMatrix());
        FlatBufferBuilder fbb = new FlatBufferBuilder(1);
        int kJ=kukaJoints.createAngleValueVector(fbb,jointValues); // it should be before startkukaJoints https://github.com/google/flatbuffers/issues/19
        int name=fbb.createString("KUKA LBR iiwa 820"); // just for testing
        kukaJoints.startkukaJoints(fbb);
        kukaJoints.addPosValue(fbb, Vector3.createVector3(fbb, t.getX(),t.getY(), t.getZ()));//https://google.github.io/flatbuffers/md__java_usage.html
        kukaJoints.addRotValue(fbb, Quaternion.createQuaternion(fbb, quat[0],quat[1],quat[2], quat[3]));//https://google.github.io/flatbuffers/md__java_usage.html
         kukaJoints.addAngleValue(fbb, kJ); // angles values
        kukaJoints.addRobotName(fbb, name); // robot name for testing
        int kj=kukaJoints.endkukaJoints(fbb);
        kukaJoints.finishkukaJointsBuffer(fbb, kj);
     // Output Flat Buffer for Testing
      //  https://github.com/ahundt/grl/blob/master/src/java/grl/src/grl/flatBufferTesting/FlatBuffer_Send.java
        byte [] sendByteArray = fbb.sizedByteArray();
     //   System.out.println("ARRAY:"+sendByteArray);
        publisher.send(sendByteArray, 0);
    //  for debugging
          //  ByteBuffer bb = ByteBuffer.wrap(sendByteArray);
 
        //kukaJoints sendmsg=kukaJoints.getRootAskukaJoints(bb);
      //    System.out.println("sent:"+sendmsg.angleValue(0)+","+sendmsg.angleValue(1)+","+sendmsg.robotName());
    }
    catch (Exception e){return;}        // Repeat if something goes wrong.
}
 
    /*****************************************************************************************************/
    /**
     * Auto-generated method stub. Do not modify the contents of this method.
     */
    public static void main(String[] args) {
        ROSComm app = new ROSComm();
        app.runApplication();   
    }
    /*****************************************************************************************************/
    public void testping() {
        // Pings IP for debugging.
        System.out.println("Pinging 172.31.1.1 (just to be sure)");
        String pingResult = "";
        try{
            Runtime r = Runtime.getRuntime();
            Process p = r.exec("ping 172.31.1.1");
            BufferedReader inbuf = new BufferedReader(new
            InputStreamReader(p.getInputStream()));
            String inputLine;
            while ((inputLine = inbuf.readLine()) != null) {
                System.out.println(inputLine);
                pingResult =pingResult+ inputLine;
            }
            inbuf.close();
            System.out.println("Pinging done.");
        } catch (IOException e) {
            System.out.println(e);
        }
    }
    /*****************************************************************************************************/
 
    public void DelayMs(double delay_time){
        long startTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startTime) < delay_time);
    }
    /*****************************************************************************************************/
    public final static MatrixRotation quatToMatrix(double d, double e, double f, double g) {
        double sqw = g*g;
        double sqx = d*d;
        double sqy = e*e;
        double sqz = f*f;
 
        MatrixBuilder mb = new MatrixBuilder();
 
        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        mb.setElement00(( sqx - sqy - sqz + sqw)*invs) ; // since sqw + sqx + sqy + sqz =1/invs*invs
        mb.setElement11((-sqx + sqy - sqz + sqw)*invs);
        mb.setElement22((-sqx - sqy + sqz + sqw)*invs);
 
        double tmp1 = d*e;
        double tmp2 = f*g;
        mb.setElement10(2.0 * (tmp1 + tmp2)*invs);
        mb.setElement01(2.0 * (tmp1 - tmp2)*invs);
 
        tmp1 = d*f;
        tmp2 = e*g;
        mb.setElement20(2.0 * (tmp1 - tmp2)*invs);
        mb.setElement02(2.0 * (tmp1 + tmp2)*invs);
 
        tmp1 = e*f;
        tmp2 = d*g;
        mb.setElement21(2.0 * (tmp1 + tmp2)*invs);
        mb.setElement12(2.0 * (tmp1 - tmp2)*invs);
 
        return MatrixRotation.of(mb.toMatrix());
    }
     
    /*****************************************************************************************************/
      public  double[] matrixToQuat(Matrix matrix) {
 
        // mercilessly copied from https://github.com/libgdx/libgdx/blob/master/gdx/src/com/badlogic/gdx/math/Quaternion.java
        double xx = matrix.getElement00();
        double xy = matrix.getElement01();
        double xz = matrix.getElement02();
        double yx = matrix.getElement10();
        double yy = matrix.getElement11();
        double yz = matrix.getElement12();
        double zx = matrix.getElement20();
        double zy = matrix.getElement21();
        double zz = matrix.getElement22();
        double quat[]=new double[]{0,0,0,1};
        double x,y,z,w; // return
 
        final double t = xx + yy + zz;
 
        // we protect the division by s by ensuring that s>=1
        if (t >= 0) { // |w| >= .5
            float s = (float)Math.sqrt(t + 1); // |s|>=1 ...
            w = 0.5f * s;
            s = 0.5f / s; // so this division isn't bad
            x = (zy - yz) * s;
            y = (xz - zx) * s;
            z = (yx - xy) * s;
        } else if ((xx > yy) && (xx > zz)) {
            float s = (float)Math.sqrt(1.0 + xx - yy - zz); // |s|>=1
            x = s * 0.5f; // |x| >= .5
            s = 0.5f / s;
            y = (yx + xy) * s;
            z = (xz + zx) * s;
            w = (zy - yz) * s;
        } else if (yy > zz) {
            float s = (float)Math.sqrt(1.0 + yy - xx - zz); // |s|>=1
            y = s * 0.5f; // |y| >= .5
            s = 0.5f / s;
            x = (yx + xy) * s;
            z = (zy + yz) * s;
            w = (xz - zx) * s;
        } else {
            float s = (float)Math.sqrt(1.0 + zz - xx - yy); // |s|>=1
            z = s * 0.5f; // |z| >= .5
            s = 0.5f / s;
            x = (xz + zx) * s;
            y = (zy + yz) * s;
            w = (yx - xy) * s;
        }
 
        quat[0]=x;quat[1]=y;quat[2]=z;quat[3]=w;
        return quat;
    }
  
    /*****************************************************************************************************/
      // --------------------------------------------------------------------------------------
      // PTP Mode (frames)
      // --------------------------------------------------------------------------------------
     public void cartPTPMove(kukaDesPose pose){
            try
            {  // move 
            	startFrame= lbr.getCurrentCartesianPosition(toolAttachedToLBR.getDefaultMotionFrame());
            	destFrame = startFrame.copyWithRedundancy();
            	
            	System.out.println("Received PTP Position:" + "[" + pose.position().x()+ "," + pose.position().y() + "," + pose.position().z()+ "]");
                System.out.println("Revieved PTP qaternion:" + "[" + pose.orientation().x()+ "," + pose.orientation().y() + "," + pose.orientation().z() + ","+ pose.orientation().w()+"]");
                 
                Vector transl=Vector.of(pose.position().x(),pose.position().y(),pose.position().z());
                MatrixRotation rot = quatToMatrix(pose.orientation().x(), pose.orientation().y(), pose.orientation().z(), pose.orientation().w());
                destFrame.setTransformationFromParent(Transformation.of(transl,rot));
//                destFrame.setX(pose.position().x());destFrame.setY(pose.position().y());destFrame.setZ(pose.position().z());
//                destFrame.setAlphaRad(Math.toRadians(pose.rotation().alpha()));
//                destFrame.setBetaRad(Math.toRadians(pose.rotation().beta()));
//                destFrame.setGammaRad(Math.toRadians(pose.rotation().gamma()));

                CartesianPTP ptpMove = ptp(destFrame);
                ptpMove.setJointJerkRel(0.1).setJointVelocityRel(0.5);
                toolAttachedToLBR.move(ptpMove); //Asynchronous move
                 
            }
            catch(Exception e){
                System.out.println("Error PTP"+e.toString());
            }
      }
      
   // --------------------------------------------------------------------------------------
   // Smart Mode (frames)
   // --------------------------------------------------------------------------------------
      
      public void cartSSMove(kukaDesPose pose){
            try
            { 
            	startFrame= theServoRuntime.getCurrentCartesianDestination(toolAttachedToLBR.getDefaultMotionFrame());
            	
            	Frame destServoFrame = startFrame.copyWithRedundancy();
         // Synchronize with the real-time system
      //          System.out.println("Revieved SS Position:" + "[" + pose.position().x()+ "," + pose.position().y() + "," + pose.position().z()+ "]");
      //          System.out.println("Revieved SS qaternion:" + "[" + pose.orientation().x()+ "," + pose.orientation().y() + "," + pose.orientation().z() + ","+ pose.orientation().w()+"]");
           //     destServoFrame.setX(pose.position().x());destServoFrame.setY(pose.position().y());destServoFrame.setZ(pose.position().z());

              Vector transl = Vector.of(pose.position().x(),pose.position().y(),pose.position().z());
              MatrixRotation rot = quatToMatrix(pose.orientation().x(), pose.orientation().y(), pose.orientation().z(), pose.orientation().w());
            	destServoFrame.setTransformationFromParent(Transformation.of(transl,rot));
            	// move 
        //        double distToBase=destServoFrame.distanceTo(lbr.getRootFrame());
        //        System.out.println("distance to base :" + String.format("%.2f",distToBase));
                
                theServoRuntime.setDestination(destServoFrame);
                       }
          
       	 catch (Exception e)
          {
       		 System.out.println("x"+e.toString());
       		 return;
         }
		
      }
      
      /*****************************************************************************************************/
     
      public void jointMove(kukaDesPose pose){
            try
            {  // move 
            	JointPosition jointCommand = new JointPosition(lbr.getJointCount());
for(int j=0;j<jointCommand.getAxisCount();j++) jointCommand.set(j,pose.joints(j));
//System.out.println("Joints: ["+Math.toDegrees(jointCommand.get(0))+", "+Math.toDegrees(jointCommand.get(1))+", "+Math.toDegrees(jointCommand.get(2))+", "
//		+Math.toDegrees(jointCommand.get(3))+", "+Math.toDegrees(jointCommand.get(4))+", "+Math.toDegrees(jointCommand.get(5))+", "
//		+Math.toDegrees(jointCommand.get(6))+"]");
theServoRuntime.setDestination(jointCommand);  
            }
            catch(Exception e){
                System.out.println("ERROR Joints"+e);
            }
      }
}