/*
 * Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbController;
import com.qualcomm.hardware.modernrobotics.comm.ReadWriteRunnable;
import com.qualcomm.hardware.modernrobotics.comm.ReadWriteRunnableStandard;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsConstants;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Locale;

/**
 * Modern Robotics USB DC Motor Controller
 * <p>
 * This is an implementation of {@link DcMotorController}
 * <p>
 * Modern Robotics USB DC Motor Controllers have a Voltage Sensor that measures the
 * current voltage of the main robot battery.
 * <p>
 * Use {@link HardwareDeviceManager} to create an instance of this class
 */
@SuppressWarnings("unused")
public final class ModernRoboticsUsbGangedDcMotorController
        extends ModernRoboticsUsbController
        implements DcMotorController, VoltageSensor
{
    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    public static final String TAG = "MRMotorController";

    /**
     * Enable DEBUG_LOGGING logging
     */
    protected static final boolean DEBUG_LOGGING = false;

    /**
     * const values used by this class
     */
    protected static final int MONITOR_LENGTH = 0x1e;
    protected static final int MOTOR_FIRST = 1;                 // first valid motor number value
    protected static final int MOTOR_LAST  = 2;                 // last valid motor number value
    protected static final int MOTOR_MAX   = MOTOR_LAST + 1;    // first invalid motor number value

    /**
     * const values used by motor controller
     */
    protected static final byte bPowerMax   = (byte)100;
    protected static final byte bPowerBrake = 0;
    protected static final byte bPowerMin   = (byte)-100;
    protected static final byte bPowerFloat = (byte)-128;
    protected static final byte RATIO_MIN = -0x80;
    protected static final byte RATIO_MAX = 0x7f;

    protected static final double apiPowerMin = -1.0;
    protected static final double apiPowerMax = 1.0;

    protected static final int DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX = 0xff;
    protected static final int BATTERY_MAX_MEASURABLE_VOLTAGE_INT = 1023; // (2^10)-1
    protected static final double BATTERY_MAX_MEASURABLE_VOLTAGE = 20.4;

    protected static final byte DEFAULT_P_COEFFICIENT = (byte) 0x80;
    protected static final byte DEFAULT_I_COEFFICIENT = (byte) 0x40;
    protected static final byte DEFAULT_D_COEFFICIENT = (byte) 0xb8;

    protected static final byte START_ADDRESS = 0x40;

    /**
     * channel mode masks used by controller
     */
    protected static final int CHANNEL_MODE_MASK_SELECTION  = 0x03;
    protected static final int CHANNEL_MODE_MASK_LOCK       = 0x04;
    protected static final int CHANNEL_MODE_MASK_REVERSE    = 0x08;
    protected static final int CHANNEL_MODE_MASK_NO_TIMEOUT = 0x10;
    protected static final int CHANNEL_MODE_MASK_EMPTY_D5   = 0x20;
    protected static final int CHANNEL_MODE_MASK_ERROR      = 0x40;
    protected static final int CHANNEL_MODE_MASK_BUSY       = 0x80;

    /**
     * channel mode flags used by controller
     */
    protected static final byte CHANNEL_MODE_FLAG_SELECT_RUN_POWER_CONTROL_ONLY = (byte) 0x0;
    protected static final byte CHANNEL_MODE_FLAG_SELECT_RUN_CONSTANT_SPEED = (byte) 0x1;
    protected static final byte CHANNEL_MODE_FLAG_SELECT_RUN_TO_POSITION = (byte) 0x2;
    protected static final byte CHANNEL_MODE_FLAG_SELECT_RESET = (byte) 0x03;
    protected static final byte CHANNEL_MODE_FLAG_LOCK = (byte) 0x04;
    protected static final byte CHANNEL_MODE_FLAG_REVERSE = (byte) 0x8;
    protected static final byte CHANNEL_MODE_FLAG_NO_TIMEOUT = (byte) 0x10;
    protected static final byte CHANNEL_MODE_FLAG_UNUSED = (byte) 0x20;
    protected static final byte CHANNEL_MODE_FLAG_ERROR = (byte) 0x40;
    protected static final byte CHANNEL_MODE_FLAG_BUSY = (byte) 0x80;
    protected static final byte CHANNEL_MODE_UNKNOWN = (byte) 0xFF; // not a real mode

    /**
     * "I2c register addresses" used in the controller hardware
     */
    protected static final int ADDRESS_MOTOR1_TARGET_ENCODER_VALUE  = 0x40;
    protected static final int ADDRESS_MOTOR1_MODE                  = 0x44;
    protected static final int ADDRESS_MOTOR1_POWER                 = 0x45;
    protected static final int ADDRESS_MOTOR2_POWER                 = 0x46;
    protected static final int ADDRESS_MOTOR2_MODE                  = 0x47;
    protected static final int ADDRESS_MOTOR2_TARGET_ENCODER_VALUE  = 0x48;
    protected static final int ADDRESS_MOTOR1_CURRENT_ENCODER_VALUE = 0x4c;
    protected static final int ADDRESS_MOTOR2_CURRENT_ENCODER_VALUE = 0x50;
    protected static final int ADDRESS_BATTERY_VOLTAGE              = 0x54;
    protected static final int ADDRESS_MOTOR1_GEAR_RATIO            = 0x56;
    protected static final int ADDRESS_MOTOR1_P_COEFFICIENT         = 0x57;
    protected static final int ADDRESS_MOTOR1_I_COEFFICIENT         = 0x58;
    protected static final int ADDRESS_MOTOR1_D_COEFFICIENT         = 0x59;
    protected static final int ADDRESS_MOTOR2_GEAR_RATIO            = 0x5a;
    protected static final int ADDRESS_MOTOR2_P_COEFFICIENT         = 0x5b;
    protected static final int ADDRESS_MOTOR2_I_COEFFICIENT         = 0x5c;
    protected static final int ADDRESS_MOTOR2_D_COEFFICIENT         = 0x5d;

    protected static final int ADDRESS_UNUSED = 0xff;

    /**
     * map of motors to memory addresses
     */
    protected static final int[] ADDRESS_MOTOR_POWER_MAP                               = { ADDRESS_UNUSED, ADDRESS_MOTOR1_POWER, ADDRESS_MOTOR2_POWER};
    protected static final int[] ADDRESS_MOTOR_MODE_MAP                                = { ADDRESS_UNUSED, ADDRESS_MOTOR1_MODE, ADDRESS_MOTOR2_MODE};
    protected static final int[] ADDRESS_MOTOR_TARGET_ENCODER_VALUE_MAP                = { ADDRESS_UNUSED, ADDRESS_MOTOR1_TARGET_ENCODER_VALUE, ADDRESS_MOTOR2_TARGET_ENCODER_VALUE};
    protected static final int[] ADDRESS_MOTOR_CURRENT_ENCODER_VALUE_MAP               = { ADDRESS_UNUSED, ADDRESS_MOTOR1_CURRENT_ENCODER_VALUE, ADDRESS_MOTOR2_CURRENT_ENCODER_VALUE};
    protected static final int[] ADDRESS_MOTOR_GEAR_RATIO_MAP                          = { ADDRESS_UNUSED, ADDRESS_MOTOR1_GEAR_RATIO, ADDRESS_MOTOR2_GEAR_RATIO};
    protected static final int[] ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP = { ADDRESS_UNUSED, ADDRESS_MOTOR1_P_COEFFICIENT, ADDRESS_MOTOR2_P_COEFFICIENT};

    public final static int BUSY_THRESHOLD = 5;
    protected static final byte cbEncoder  = 4;

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    static class MotorProperties
    {
        // We have caches of values that we *could* read from the controller, and need to
        // do so if the cache is invalid
        LastKnown<Byte> lastKnownPowerByte          = new LastKnown<>();
        LastKnown<Integer>          lastKnownTargetPosition     = new LastKnown<>();
        LastKnown<DcMotor.RunMode>  lastKnownMode               = new LastKnown<>();

        // The remainder of the data is authoritative, here
        DcMotor.ZeroPowerBehavior   zeroPowerBehavior          = DcMotor.ZeroPowerBehavior.BRAKE;
        boolean                     modeSwitchCompletionNeeded = false;
        int                         modeSwitchWaitCount        = 0;
        int                         modeSwitchWaitCountMax     = 4;
        DcMotor.RunMode             prevRunMode                = null;
        double                      prevPower;
        int                         maxSpeed;               // in encoder ticks / second
    }

    MotorProperties[] motors = new MotorProperties[MOTOR_MAX];

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Use HardwareDeviceManager to create an instance of this class
     */
    public ModernRoboticsUsbGangedDcMotorController(final Context context, final SerialNumber serialNumber, final OpenRobotUsbDevice openRobotUsbDevice, EventLoopManager manager)
            throws RobotCoreException, InterruptedException
    {
        super(context, serialNumber, manager, openRobotUsbDevice, new CreateReadWriteRunnable()
        {
            @Override public ReadWriteRunnable create(RobotUsbDevice device)
            {
                return new ReadWriteRunnableStandard(context, serialNumber, device, MONITOR_LENGTH, START_ADDRESS, DEBUG_LOGGING);
            }
        });
        for (int motor = 0; motor < motors.length; motor++)
        {
            motors[motor] = new MotorProperties();
        }
        resetMaxMotorSpeeds();
    }

    @Override
    public void initializeHardware()
    {
        // set all motors to float for safety reasons
        floatHardware();
        setDifferentialControlLoopCoefficientsToDefault();
    }

    //----------------------------------------------------------------------------------------------
    // Arming and disarming
    //----------------------------------------------------------------------------------------------

    void resetMaxMotorSpeeds()
    {
        for (int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            motors[motor].maxSpeed = getDefaultMaxMotorSpeed(motor);
        }
    }

    void brakeAllAtZero()
    {
        for (int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            motors[motor].zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        }
    }

    void forgetLastKnown()
    {
        for (int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            motors[motor].lastKnownPowerByte.invalidate();
            motors[motor].lastKnownMode.invalidate();
            motors[motor].lastKnownTargetPosition.invalidate();
        }
    }

    void forgetLastKnownPowers()
    {
        for (int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            motors[motor].lastKnownPowerByte.invalidate();
        }
    }

    @Override protected void doArm() throws RobotCoreException, InterruptedException
    {
        doArmOrPretend(true);
    }
    @Override protected void doPretend() throws RobotCoreException, InterruptedException
    {
        doArmOrPretend(false);
    }

    private void doArmOrPretend(boolean isArm) throws RobotCoreException, InterruptedException
    {
        RobotLog.d("arming modern motor controller %s%s...", HardwareFactory.getDeviceDisplayName(context, this.serialNumber), (isArm ? "" : " (pretend)"));

        forgetLastKnown();
        if (isArm)
            this.armDevice();
        else
            this.pretendDevice();

        RobotLog.d("...arming modern motor controller %s complete", HardwareFactory.getDeviceDisplayName(context, this.serialNumber));
    }

    @Override protected void doDisarm() throws RobotCoreException, InterruptedException
    {
        RobotLog.d("disarming modern motor controller %s...", HardwareFactory.getDeviceDisplayName(context, this.serialNumber));

        this.disarmDevice();
        forgetLastKnown();  // perhaps unneeded, but harmless

        RobotLog.d("...disarming modern motor controller %s complete", HardwareFactory.getDeviceDisplayName(context, this.serialNumber));
    }

    @Override protected void doCloseFromArmed()
    {
        floatHardware();
        doCloseFromOther();
    }

    @Override protected void doCloseFromOther()
    {
        try {
            this.doDisarm();
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
        catch (RobotCoreException ignore)
        {
            // ignore, won't actually happen
        }
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice interface
    //----------------------------------------------------------------------------------------------

    @Override public Manufacturer getManufacturer()
    {
        return Manufacturer.ModernRobotics;
    }

    /**
     * Device Name
     *
     * @return device name
     */
    @Override
    public String getDeviceName()
    {
        RobotUsbDevice.FirmwareVersion version = this.robotUsbDevice.getFirmwareVersion();
        if (version != null)
        {
            return String.format(Locale.US,
               "%s v%d.%d",
               context.getString(com.qualcomm.hardware.R.string.moduleDisplayNameMotorController),
               version.majorVersion, version.minorVersion);
        }
        else
        {
            return context.getString(com.qualcomm.hardware.R.string.moduleDisplayNameMotorController);
        }
    }

    @Override
    public String getConnectionInfo()
    {
        return "USB " + getSerialNumber();
    }

    @Override
    public void resetDeviceConfigurationForOpMode()
    {
        resetMaxMotorSpeeds();
        floatHardware();
        runWithoutEncoders();
        brakeAllAtZero();
        forgetLastKnown();
    }

    /**
     * Close this device
     */
    public void close()
    {
        floatHardware();
        super.close();
    }

    //------------------------------------------------------------------------------------------------
    // DcMotorController interface
    //------------------------------------------------------------------------------------------------

    protected int getDefaultMaxMotorSpeed(int motor)
    {
        final int encoderTicksPerRevolution = ModernRoboticsConstants.TETRIX_MOTOR_TICKS_PER_REVOLUTION; // We assume a Tetrix motor. For v2 firmware and above, we could instead query.
        final int maxDegreesPerSecond       = ModernRoboticsConstants.MAX_PID_DEGREES_PER_SECOND;
        final int degreesPerRevolution      = 360;

        return encoderTicksPerRevolution * maxDegreesPerSecond / degreesPerRevolution;
    }

    @Override public synchronized int getMotorMaxSpeed(int motor)
    {
        this.validateMotor(motor);
        return motors[motor].maxSpeed;
    }

    @Override public synchronized void setMotorMaxSpeed(int motor, int encoderTicksPerSecond)
    {
        this.validateMotor(motor);
        encoderTicksPerSecond = this.validateEncoderTicksPerSecond(motor, encoderTicksPerSecond);
        if (motors[motor].maxSpeed != encoderTicksPerSecond)
        {
            // Preserve the commanded power across the change
            DcMotor.RunMode mode = internalGetCachedOrQueriedRunMode(motor);
            if (mode.isPIDMode())
            {
                double power = internalGetCachedOrQueriedMotorPower(motor);
                motors[motor].maxSpeed = encoderTicksPerSecond;
                internalSetMotorPower(motor, power);
            }
            else
            {
                motors[motor].maxSpeed = encoderTicksPerSecond;
            }
        }
    }

    @Override public synchronized void setMotorMode(int motor, DcMotor.RunMode mode)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);

        DcMotor.RunMode prevMode = motors[motor].lastKnownMode.getNonTimedValue();
        if (motors[motor].lastKnownMode.updateValue(mode))
        {
            // Set us up so that we'll await the completion of the mode switch before doing
            // anything else with this motor. We just won't take that time *right*now*.
            motors[motor].modeSwitchCompletionNeeded = true;
            motors[motor].modeSwitchWaitCount = 0;
            motors[motor].prevRunMode = prevMode;
            motors[motor].prevPower = internalGetCachedOrQueriedMotorPower(motor);

            byte bNewMode = modeToByte(mode);
            this.write8(ADDRESS_MOTOR_MODE_MAP[motor], bNewMode);
        }
    }

    void finishModeSwitchIfNecessary(int motor)
    // Here, we implement the logic that completes the mode switch of the motor. We separate
    // that out from setMotorChannelMode itself so as to allow parallel mode switching of motors.
    // A common paradigm where this happens is that of resetting the encoders across all the
    // motors on one's bot. Having this separate like this speeds that up, somewhat.
    {
        // If there's nothing we need to do, then get out
        if (!motors[motor].modeSwitchCompletionNeeded)
        {
            return;
        }

        try {
            DcMotor.RunMode mode = internalGetCachedOrQueriedRunMode(motor);
            DcMotor.RunMode prevMode = motors[motor].prevRunMode;
            byte bNewMode = modeToByte(mode);
            byte bRunWithoutEncoderMode = modeToByte(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // The mode switch doesn't happen instantaneously. Wait for it,
            // so that the programmer's model is that he just needs to set the
            // mode and be done.
            for (;;)
            {
                if (!this.isArmed()) break;

                byte bCurrentMode = (byte)(this.read8(ADDRESS_MOTOR_MODE_MAP[motor]) & CHANNEL_MODE_MASK_SELECTION);
                if (bCurrentMode == bNewMode)
                    break;

                // Modern Robotics USB DC motor controllers with firmware >= 2.0 (note: only 2.0 exists
                // as of this writing) have a different behavior when switching to STOP_AND_RESET_ENCODER
                // mode: they only transiently enter that mode, then auto-switch to RUN_WITHOUT_ENCODER.
                // Note that the manufacturer informs us that this >= 2.0 behavior is in fact the behavior
                // on all firmware versions, but that's not consistent with our observations. For
                // robustness, however, we allow that transition on all firmware versions.
                if (mode==DcMotor.RunMode.STOP_AND_RESET_ENCODER && bCurrentMode==bRunWithoutEncoderMode)
                    break;

                // If we're waiting too long, then resend the mode switch. The theory is that
                // if switches are happening too quickly then one switch might have been missed.
                // Not a perfectly-understood theory, mind you, but there you go.
                if (motors[motor].modeSwitchWaitCount++ >= motors[motor].modeSwitchWaitCountMax)
                {
                    RobotLog.dd(TAG, "mode resend: motor=[%s,%d] wait=%d from=%d to=%d cur=%d",
                            getSerialNumber(), motor, motors[motor].modeSwitchWaitCount-1,
                            (prevMode==null ? CHANNEL_MODE_UNKNOWN : modeToByte(prevMode)), bNewMode, bCurrentMode);
                    this.write8(ADDRESS_MOTOR_MODE_MAP[motor], bNewMode);
                    motors[motor].modeSwitchWaitCount = 0;
                }

                // The above read8() reads from cache. To avoid flooding the system,
                // we wait for the next read cycle before we try again: the cache
                // isn't going to change until then.
                if (!waitForNextReadComplete()) break;
            }

            if (mode.isPIDMode() && (prevMode== null || !prevMode.isPIDMode()))
            {
                double power = motors[motor].prevPower;
                if (mode == DcMotor.RunMode.RUN_TO_POSITION)
                {
                    // Enforce that in RUN_TO_POSITION, we always need *positive* power. DCMotor will
                    // take care of that if we set power *after* we set the mode, but not the other way
                    // around. So we handle that here.
                    //
                    // Unclear that this is needed. The motor controller might take the absolute value
                    // automatically. But harmless in that uncertainty.
                    power = Math.abs(power);
                }
                internalSetMotorPower(motor, power);
            }

            else if (mode == DcMotor.RunMode.RUN_TO_POSITION)
            {
                double power = internalQueryMotorPower(motor);
                if (power < 0)
                    internalSetMotorPower(motor, Math.abs(power));
            }

            if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            {
                // If the mode is 'reset encoders', we don't want to return until the encoders have actually reset
                //      http://ftcforum.usfirst.org/showthread.php?4924-Use-of-RUN_TO_POSITION-in-LineraOpMode&highlight=reset+encoders
                //      http://ftcforum.usfirst.org/showthread.php?4567-Using-and-resetting-encoders-in-MIT-AI&p=19303&viewfull=1#post19303
                // For us, here, we believe we'll always *immediately* have that be true, as our writes
                // to the USB device actually happen when we issue them.
                // Unclear if this is needed, but anecdotes from (e.g.) Dryw Wade seem to indicate that it is

                long nsSendInterval = 100 * ElapsedTime.MILLIS_IN_NANO;
                long nsResendDeadline = System.nanoTime() + nsSendInterval;
                while (this.internalQueryMotorCurrentPosition(motor) != 0)
                {
                    // Robustness: resend mode if we can't see zero'd encoders after basically forever
                    long nsNow = System.nanoTime();
                    if (nsNow > nsResendDeadline)
                    {
                        RobotLog.dd(TAG, "mode resend: motor=[%s,%d] mode=%s", getSerialNumber(), motor, mode);
                        this.write8(ADDRESS_MOTOR_MODE_MAP[motor], bNewMode);
                        nsResendDeadline = nsNow + nsSendInterval;
                    }

                    if (!this.isArmed()) break;
                    if (!waitForNextReadComplete()) break;
                }
            }
        }
        finally
        {
            // On the legacy controller (at least, not sure about the modern one), when in RESET_ENCODERs
            // writes to power are ignored: the byte power value is always zero (off, braked) while in that
            // mode. So, transitioning either into or out of that mode, what we last thought we knew about
            // the power level is perhaps wrong, so we just forget what we thought we knew.
            //
            // Moreover, with the post-v2-firmware spontaneous mode switching, our cache of the modes
            // is suspect too. So, for robustness, we flush all our caches.
            forgetLastKnown();

            // Ok, this mode switch is done!
            motors[motor].modeSwitchCompletionNeeded = false;
        }
    }

    boolean waitForCallback()
    {
        this.callbackWaiterCount.incrementAndGet();
        boolean interrupted = false;
        if (this.readWriteRunnableIsRunning)
        {
            try {
                callbackLock.wait();
            }
            catch (InterruptedException e)
            {
                interrupted = true;
                Thread.currentThread().interrupt();
            }
        }
        boolean result = !interrupted && this.readWriteRunnableIsRunning;
        this.callbackWaiterCount.decrementAndGet();
        return result;
    }

    boolean waitForNextReadComplete()
    {
        synchronized (this.concurrentClientLock)
        {
            synchronized (this.callbackLock)
            {
                long cur = this.readCompletionCount.get();
                long target = cur + 1;
                while (this.readCompletionCount.get() < target)
                {
                    if (!this.isArmed())
                        return false;
                    if (!waitForCallback())
                        return false;     // interrupted or readWriteRunnable is dead, deem us to have completed
                }
            }
        }
        return true;
    }

    @Override public synchronized DcMotor.RunMode getMotorMode(int motor)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);
        return internalQueryRunMode(motor);
    }

    DcMotor.RunMode internalQueryRunMode(int motor)
    {
        byte b = this.read8(ADDRESS_MOTOR_MODE_MAP[motor]);
        DcMotor.RunMode result = modeFromByte(b);
        motors[motor].lastKnownMode.setValue(result);
        return result;
    }

    DcMotor.RunMode internalGetCachedOrQueriedRunMode(int motor)
    {
        DcMotor.RunMode mode = motors[motor].lastKnownMode.getNonTimedValue();
        if (mode == null)
        {
            mode = internalQueryRunMode(motor);
        }
        return mode;
    }

    @Override public synchronized void setMotorPower(int motor, double power)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);
        internalSetMotorPower(motor, power);
    }

    public synchronized void setMotorsPower(double[] powers)
    {
        for(int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            this.validateMotor(motor);
            finishModeSwitchIfNecessary(motor);
        }
        internalSetMotorsPower(powers);
    }

    void internalSetMotorPower(int motor, double power)
    {
        power = Range.clip(power, apiPowerMin, apiPowerMax);
        this.validateApiMotorPower(power);  // may catch NaNs, for example

        DcMotor.RunMode mode = internalGetCachedOrQueriedRunMode(motor);
        if (mode.isPIDMode())
        {
            double defMaxSpeed = getDefaultMaxMotorSpeed(motor);
            power = Math.signum(power) * Range.scale(Math.abs(power), 0, apiPowerMax, 0, motors[motor].maxSpeed / defMaxSpeed);
            power = Range.clip(power, apiPowerMin, apiPowerMax);
        }

        byte bPower = (power == 0.0 && motors[motor].zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT)
                              ? bPowerFloat
                              : (byte)Range.scale(power, apiPowerMin, apiPowerMax, bPowerMin, bPowerMax);
        internalSetMotorPower(motor, bPower);
    }

    void internalSetMotorsPower(double[] powers)
    {
        byte[] bPowers = new byte[powers.length];
        for(int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            double power = powers[motor];
            power = Range.clip(power, apiPowerMin, apiPowerMax);
            this.validateApiMotorPower(power);  // may catch NaNs, for example

            DcMotor.RunMode mode = internalGetCachedOrQueriedRunMode(motor);
            if (mode.isPIDMode())
            {
                double defMaxSpeed = getDefaultMaxMotorSpeed(motor);
                power = Math.signum(power) *
                        Range.scale(Math.abs(power),
                                    0, apiPowerMax, 0, motors[motor].maxSpeed / defMaxSpeed);
                power = Range.clip(power, apiPowerMin, apiPowerMax);
            }

            byte bPower = (power == 0.0 &&
                           motors[motor].zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT)
                              ? bPowerFloat
                              : (byte) Range.scale(power, apiPowerMin, apiPowerMax,
                                                   bPowerMin, bPowerMax);
            bPowers[motor] = bPower;
        }
        internalSetMotorsPower(bPowers);
    }

    void internalSetMotorPower(int motor, byte bPower)
    {
        if (motors[motor].lastKnownPowerByte.updateValue(bPower))
        {
            this.write8(ADDRESS_MOTOR_POWER_MAP[motor], bPower);
        }
    }

    public void writeByteArray(int address, byte[] data) {
        write(address, data);
    }

    void internalSetMotorsPower(byte[] bPowers)
    {
        boolean powersFresh = true;
        for(int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            powersFresh = powersFresh &&
                          motors[motor].lastKnownPowerByte.updateValue(bPowers[motor]);
        }
        if (powersFresh)
        {
            this.writeByteArray(ADDRESS_MOTOR_POWER_MAP[MOTOR_FIRST], bPowers);
        }
    }

    double internalQueryMotorPower(int motor)
    {
        byte bPower = this.read8(ADDRESS_MOTOR_POWER_MAP[motor]);
        motors[motor].lastKnownPowerByte.setValue(bPower);
        return internalMotorPowerFromByte(motor, bPower);
    }

    double internalGetCachedOrQueriedMotorPower(int motor)
    {
        Byte bPower = motors[motor].lastKnownPowerByte.getNonTimedValue();
        if (bPower != null)
            return internalMotorPowerFromByte(motor, bPower);
        else
            return internalQueryMotorPower(motor);
    }

    double internalMotorPowerFromByte(int motor, byte bPower)
    {
        if (bPower == bPowerFloat)
            return 0.0; // Float counts as zero power
        else
        {
            double power = Range.scale(bPower, bPowerMin, bPowerMax, apiPowerMin, apiPowerMax);
            DcMotor.RunMode mode = internalGetCachedOrQueriedRunMode(motor);
            if (mode.isPIDMode())
            {
                // undo the scaling that setMotorPower() did
                double defMaxSpeed = getDefaultMaxMotorSpeed(motor);
                power = Math.signum(power) * Range.scale(Math.abs(power), 0, motors[motor].maxSpeed / defMaxSpeed, 0, apiPowerMax);
            }
            return Range.clip(power, apiPowerMin, apiPowerMax);
        }
    }

    @Override public synchronized double getMotorPower(int motor)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);
        return internalQueryMotorPower(motor);
    }

    // From the HiTechnic Motor Controller specification (the Modern Robotics motor controller is
    // understood to have the self-same issue):
    //
    //      The Run to position command will cause the firmware to run the motor to make the current encoder
    //      value to become equal to the target encoder value. It will do this using a maximum rotation rate
    //      as defined by the motor power byte. It will hold this position in a servo like mode until the Run
    //      to position command is changed or the target encoder value is changed. While the Run to position
    //      command is executing, the Busy bit will be set. Once the target position is achieved, the Busy bit
    //      will be cleared. There may be a delay of up to 50mS after a Run to position command is initiated
    //      before the Busy bit will be set.
    //
    // Our task here is to work around that 50ms issue.

    @Override
    public boolean isBusy(int motor)
    {
        validateMotor(motor);
        finishModeSwitchIfNecessary(motor);

        // Compare current and target positions to determine if RunToPosition is still busy.
        return (Math.abs(getMotorTargetPosition(motor) - getMotorCurrentPosition(motor)) > BUSY_THRESHOLD);
    }

    @Override public synchronized void setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        this.validateMotor(motor);
        if (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.UNKNOWN) throw new IllegalArgumentException("zeroPowerBehavior may not be UNKNOWN");
        finishModeSwitchIfNecessary(motor);

        if (motors[motor].zeroPowerBehavior != zeroPowerBehavior)
        {
            motors[motor].zeroPowerBehavior = zeroPowerBehavior;

            // If we're currently stopped, then reissue power to cause new zero behavior to take effect
            if (internalGetCachedOrQueriedMotorPower(motor) == 0.0)
            {
                motors[motor].lastKnownPowerByte.invalidate();  // be sure we reissue
                internalSetMotorPower(motor, 0.0);
            }
        }
    }

    @Override public synchronized DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int motor)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);

        return motors[motor].zeroPowerBehavior;
    }

    protected synchronized void setMotorPowerFloat(int motor)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);

        this.write8(ADDRESS_MOTOR_POWER_MAP[motor], bPowerFloat);
    }

    @Override public synchronized boolean getMotorPowerFloat(int motor)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);

        byte bPower = this.read8(ADDRESS_MOTOR_POWER_MAP[motor]);
        return bPower == bPowerFloat;
    }

    @Override public synchronized void setMotorTargetPosition(int motor, int position)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);

        if (motors[motor].lastKnownTargetPosition.updateValue(position))
        {
            // We rely here on the fact that sizeof(int) == cbEncoder
            this.write(ADDRESS_MOTOR_TARGET_ENCODER_VALUE_MAP[motor], TypeConversion.intToByteArray(position, ByteOrder.BIG_ENDIAN));
        }
    }

    @Override public synchronized int getMotorTargetPosition(int motor)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);
        return internalQueryMotorTargetPosition(motor);
    }

    int internalQueryMotorTargetPosition(int motor)
    {
        byte[] rgbPosition = this.read(ADDRESS_MOTOR_TARGET_ENCODER_VALUE_MAP[motor], cbEncoder);
        int result = TypeConversion.byteArrayToInt(rgbPosition, ByteOrder.BIG_ENDIAN);
        motors[motor].lastKnownTargetPosition.setValue(result);
        return result;
    }

    @Override public synchronized int getMotorCurrentPosition(int motor)
    {
        this.validateMotor(motor);
        finishModeSwitchIfNecessary(motor);
        return internalQueryMotorCurrentPosition(motor);
    }

    public synchronized int[] getMotorsCurrentPositions()
    {
        for(int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            this.validateMotor(motor);
            finishModeSwitchIfNecessary(motor);
        }
        return internalQueryMotorsCurrentPositions();
    }

    int internalQueryMotorCurrentPosition(int motor)
    {
        byte[] bytes = this.read(ADDRESS_MOTOR_CURRENT_ENCODER_VALUE_MAP[motor], cbEncoder);
        return TypeConversion.byteArrayToInt(bytes, ByteOrder.BIG_ENDIAN);
    }

    int[] internalQueryMotorsCurrentPositions()
    {
        byte[] bytes = this.read(ADDRESS_MOTOR_CURRENT_ENCODER_VALUE_MAP[MOTOR_FIRST],
                MOTOR_LAST*cbEncoder);
        int[] cnts = new int[MOTOR_LAST];
        for(int b = 0; b<MOTOR_LAST; b++)
        {
            int start = b*cbEncoder;
            int end   = start + cbEncoder;
            cnts[b] =
                    TypeConversion.byteArrayToInt(Arrays.copyOfRange(bytes, start, end)
                    , ByteOrder.BIG_ENDIAN);
        }
        return cnts;
    }

    //----------------------------------------------------------------------------------------------
    // VoltageSensor
    //----------------------------------------------------------------------------------------------

    /**
     * Get battery voltage. Measurements range from 0 to BATTERY_MAX_VOLTAGE. Measurement resolution
     * is 20mV.
     *
     * @return voltage
     */
    @Override
    public double getVoltage()
    {
        byte[] data = read(ADDRESS_BATTERY_VOLTAGE, 2);

        // data is in an unusual format, only the top 8 bits and bottom 2 bits count { XXXXXXXX, 000000XX }
        int voltage = TypeConversion.unsignedByteToInt(data[0]);
        voltage = voltage << 2;
        voltage += TypeConversion.unsignedByteToInt(data[1]) & 0x03;
        voltage = voltage & BATTERY_MAX_MEASURABLE_VOLTAGE_INT;

        // now calculate the percentage, relative to the max reading
        double percent = (double) (voltage) / (double) (BATTERY_MAX_MEASURABLE_VOLTAGE_INT);

        // scale to max value and return
        return percent * BATTERY_MAX_MEASURABLE_VOLTAGE;
    }

    public void setGearRatio(int motor, double ratio)
    {
        validateMotor(motor);
        Range.throwIfRangeIsInvalid(ratio, -1, 1);

        write(ADDRESS_MOTOR_GEAR_RATIO_MAP[motor], new byte[]{(byte) (ratio * RATIO_MAX)});
    }

    public double getGearRatio(int motor)
    {
        validateMotor(motor);

        byte[] data = read(ADDRESS_MOTOR_GEAR_RATIO_MAP[motor], 1);
        return (double) data[0] / (double) RATIO_MAX;
    }

    public void setDifferentialControlLoopCoefficients(int motor,
                                                       DifferentialControlLoopCoefficients pid)
    {
        validateMotor(motor);

        if (pid.p > DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX)
        {
            pid.p = DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX;
        }

        if (pid.i > DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX)
        {
            pid.i = DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX;
        }

        if (pid.d > DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX)
        {
            pid.d = DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAX;
        }

        write(ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP[motor],
                new byte[]{(byte) pid.p, (byte) pid.i, (byte) pid.d});
    }

    public DifferentialControlLoopCoefficients getDifferentialControlLoopCoefficients(int motor)
    {
        validateMotor(motor);

        DifferentialControlLoopCoefficients pid = new DifferentialControlLoopCoefficients();
        byte[] data = read(ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP[motor], 3);
        pid.p = data[0];
        pid.i = data[1];
        pid.d = data[2];

        return pid;
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public static byte modeToByte(DcMotor.RunMode mode)
    {
        switch (mode)
        {
            case RUN_USING_ENCODER:         return CHANNEL_MODE_FLAG_SELECT_RUN_CONSTANT_SPEED;
            case RUN_WITHOUT_ENCODER:       return CHANNEL_MODE_FLAG_SELECT_RUN_POWER_CONTROL_ONLY;
            case RUN_TO_POSITION:           return CHANNEL_MODE_FLAG_SELECT_RUN_TO_POSITION;
            case STOP_AND_RESET_ENCODER:    return CHANNEL_MODE_FLAG_SELECT_RESET;
        }
        return CHANNEL_MODE_FLAG_SELECT_RUN_CONSTANT_SPEED;
    }

    public static DcMotor.RunMode modeFromByte(byte flag)
    {
        switch (flag & CHANNEL_MODE_MASK_SELECTION)
        {
            case CHANNEL_MODE_FLAG_SELECT_RUN_CONSTANT_SPEED:       return DcMotor.RunMode.RUN_USING_ENCODER;
            case CHANNEL_MODE_FLAG_SELECT_RUN_POWER_CONTROL_ONLY:   return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            case CHANNEL_MODE_FLAG_SELECT_RUN_TO_POSITION:          return DcMotor.RunMode.RUN_TO_POSITION;
            case CHANNEL_MODE_FLAG_SELECT_RESET:                    return DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        }
        return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }

    private void floatHardware()
    {
        for (int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            this.setMotorPowerFloat(motor);
        }
    }

    private void runWithoutEncoders()
    {
        for (int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            this.setMotorMode(motor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void setDifferentialControlLoopCoefficientsToDefault()
    {
        for (int motor = MOTOR_FIRST; motor <= MOTOR_LAST; motor++)
        {
            write(ADDRESS_MAX_DIFFERENTIAL_CONTROL_LOOP_COEFFICIENT_MAP[motor],
                    new byte[]{DEFAULT_P_COEFFICIENT, DEFAULT_I_COEFFICIENT, DEFAULT_D_COEFFICIENT});
        }
    }

    private void validateMotor(int motor)
    {
        if (motor < MOTOR_FIRST || motor > MOTOR_LAST)
        {
            throw new IllegalArgumentException(
               String.format(Locale.US,
                       "Motor %d is invalid; valid motors are %d..%d",
                       motor, MOTOR_FIRST, MOTOR_LAST));
        }
    }

    private int validateEncoderTicksPerSecond(int motor, int encoderTicksPerSecond)
    {
        // We enforce clipping against the max since we only talk to the controller in terms
        // relative to that; we don't speak 'encoder ticks / s' directly
        encoderTicksPerSecond = Range.clip(encoderTicksPerSecond, 1, getDefaultMaxMotorSpeed(motor));
        return encoderTicksPerSecond;
    }

    private void validateApiMotorPower(double power)
    {
        if (!(apiPowerMin <= power && power <= apiPowerMax))
        {
            throw new IllegalArgumentException(String.format(Locale.US,
                    "illegal motor power %f; must be in interval [%f,%f]",
                    power, apiPowerMin, apiPowerMax));
        }
    }
}
