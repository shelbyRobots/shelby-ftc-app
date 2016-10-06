package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.EnumMap;
import java.util.Map;

@Autonomous(name = "RedOpIterative", group = "Auton")
//@Disabled
public class RedOpIterative extends OpMode
{
    @Override
    public void init() {
        robot.init(hardwareMap);
        Drivetrain.init(robot.leftMotor, robot.rightMotor);

        setup();
        for(int t = 0; t < turns.length; ++t)
        {
            angMap.put(State.valueOf("TRN" + t), turns[t]);
        }

        state = State.STRT;
    }

    @Override
    public void start()
    {
        timer.reset();
        currPoint = pathSegs[0].getStrtPt();
        Drivetrain.setCurrPt(currPoint);
        DbgLog.msg("SJH start: %s (%5.2f, %5.2f). Frame: %4d Time: %6.3f",
                state.toString(), currPoint.getX(), currPoint.getY(),
                frame, timer.time());
        state = State.DRV1;
    }

    @Override
    public void loop()
    {
        if (frame % 100 == 0)
        {
            DbgLog.msg("SJH Frame: %6d %10.4f", frame, timer.time());
        }

        Segment seg = getSegment(state.name());
        Drivetrain.Direction dir = Drivetrain.Direction.FORWARD;
        if(seg != null)
        {
            if(seg.getDir() == Segment.SegDir.REVERSE)
            {
                dir = Drivetrain.Direction.REVERSE;
            }
        }

        double ang = angMap.get(state);
        State  nxt = State.STOP;

        State[] states = State.values();
        int numStates = states.length;
        String[] stateNames = new String[numStates];
        for(int s = 0; s < numStates; ++s)
        {
            stateNames[s] = states[s].name();
        }

        for(int n = 0; n < numStates - 1; ++n)
        {
            nxt = State.valueOf(stateNames[n+1]);
        }

        switch(state) {
            case DRV1: procMoveStateFrame(seg, nxt, dir); break;
            case TRN1: procTurnStateFrame(ang, nxt);      break;
            case DRV2: procMoveStateFrame(seg, nxt, dir); break;
            case TRN2: procTurnStateFrame(ang, nxt);      break;
            case DRV3: procMoveStateFrame(seg, nxt, dir); break;
            case TRN3: procTurnStateFrame(ang, nxt);      break;
            case DRV4: procMoveStateFrame(seg, nxt, dir); break;
            case TRN4: procTurnStateFrame(ang, nxt);      break;
            case DRV5: procMoveStateFrame(seg, nxt, dir); break;
            case TRN5: procTurnStateFrame(ang, nxt);      break;
            case DRV6: procMoveStateFrame(seg, nxt, dir); break;
            case TRN6: procTurnStateFrame(ang, nxt);      break;
            case DRV7: procMoveStateFrame(seg, nxt, dir); break;
            case STOP: state = State.DONE;                break;
            case DONE: Drivetrain.stopAndReset();         break;
        }
        telemetry.addData("RunTime", "%6.3f", getRuntime());
        telemetry.addData("Frame_State", "%5d %s %s", frame, state, nxt);
        frame++;
    }

    private void setup()
    {
        robot.init(hardwareMap);
        Drivetrain.init(robot.leftMotor, robot.rightMotor);

        Points pts = new Points();
        pathSegs = pts.getSegments(Field.Alliance.RED);
        turns    = pts.getTurns(Field.Alliance.RED);

        Point2d currPoint = pathSegs[0].getStrtPt();
        Drivetrain.setCurrPt(currPoint);

        timer.reset();
        DbgLog.msg("SJH Start %s. Time: %6.3f", currPoint, timer.time());

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path", "Start at %s", currPoint);
        telemetry.update();
    }

    private void procMoveStateFrame(Segment seg, State nextState,
                                    Drivetrain.Direction dir)
    {
        if(frame % 100 == 0)
        {
            Drivetrain.logDriveState();
        }

        if (firstPassInState)
        {
            nextPoint = seg.getTgtPt();
            double dist = currPoint.distance(nextPoint);
            DbgLog.msg("SJH Drive: %s to (%5.2f, %5.2f). Frame: %4d Time: %6.3f, Dist: %5.2f",
                    seg.getName(), nextPoint.getX(),nextPoint.getY(),
                    frame, timer.time(), dist);
            firstPassInState = false;
            Drivetrain.driveToPoint(nextPoint, DEF_DRV_PWR, dir);
            timer.reset();
        }

        else if (!Drivetrain.isBusy())
        {
            DbgLog.msg("SJH Completed state %s. Time: %6.3f", seg.getName(), timer.time());
            Drivetrain.stopAndReset();
            firstPassInState = true;
            state = nextState;
            currPoint = nextPoint;
            Drivetrain.setCurrPt(currPoint);
        }
        else
        {
            Drivetrain.makeCorrections(DEF_DRV_PWR, dir);
        }
    }

    private void procTurnStateFrame(double angle, State nextState)
    {
        if(frame % 100 == 0)
        {
            Drivetrain.logDriveState();
        }

        Drivetrain.Direction dir = Drivetrain.Direction.FORWARD;

        if (firstPassInState)
        {
            firstPassInState = false;
            DbgLog.msg("SJH Start Turn: %s Angle: %5.2f Frame: %4d Time: %6.3f, Hdg: %5.2f",
                    state, angle, frame, timer.time(), currHdg);
            Drivetrain.ctrTurn(angle, DEF_DRV_PWR);
            currHdg += angle;
            while(currHdg >  360.0) currHdg -= 360.0;
            while(currHdg <    0.0) currHdg += 360.0;
            timer.reset();
        }
        else if(!Drivetrain.isBusy())
        {
            DbgLog.msg("SJH Completed turn %s. Time: %6.3f", state, timer.time());
            Drivetrain.stopAndReset();
            firstPassInState = true;
            state = nextState;
        }
        else
        {
            Drivetrain.makeCorrections(DEF_TRN_PWR, dir);
        }
    }

    private Segment getSegment(String name)
    {
        for (Segment pathSeg : pathSegs)
        {
            String n = pathSeg.getName();
            if (n.equals(name)) return pathSeg;
        }
        return null;
    }

    private final static double DEF_DRV_PWR = 0.7;
    private final static double DEF_TRN_PWR = 0.5;

    private enum State
    {
        STRT,
        DRV1,
        TRN1,
        DRV2,
        TRN2,
        DRV3,
        TRN3,
        DRV4,
        TRN4,
        DRV5,
        TRN5,
        DRV6,
        TRN6,
        DRV7,
        STOP,
        DONE
    }

    private boolean     firstPassInState = true;
    private State       state            = State.STRT;
    private ElapsedTime timer            = new ElapsedTime();
    private int         frame            = 0;

    private double      currHdg          = 90.0;
    private Point2d     currPoint;
    private Point2d     nextPoint;

    //List of points to move to - one per drive state.
    private Map<State, Double>  angMap = new EnumMap<>(State.class);

    private Segment[] pathSegs;
    private double[] turns;

    private ShelbyBot robot = new ShelbyBot();
}