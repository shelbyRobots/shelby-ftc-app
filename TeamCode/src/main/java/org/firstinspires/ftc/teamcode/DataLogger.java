package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.ftccommon.DbgLog;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Locale;

public class DataLogger {
    private Writer writer;
    private StringBuffer lineBuffer;
    private long msBase;
    private long nsBase;

    public DataLogger(String fileName) {
        //String directoryPath  = "/sdcard/FIRST/DataLogger";
        String directoryPath  = Environment.getExternalStorageDirectory().getPath() +
                                        "/FIRST/DataLogger";
        String filePath       = directoryPath + "/" + fileName + ".csv";

        File fDir = new File(directoryPath);
        // Create directory if it does not exist
        if(!fDir.isDirectory())
        {
            if(!fDir.mkdir())
            {
                DbgLog.error("SJH: Could not create directory " + directoryPath);
            }
        }

        try {
            writer = new FileWriter(filePath);
            lineBuffer = new StringBuffer(128);
        } catch (IOException e) {
            DbgLog.error("SJH: Could not create file " + filePath);
        }
        msBase = System.currentTimeMillis();
        nsBase = System.nanoTime();
        addField("sec");
        addField("d ms");
    }

    private void flushLineBuffer(){
        long milliTime,nanoTime;

        try {
            lineBuffer.append('\n');
            writer.write(lineBuffer.toString());
            lineBuffer.setLength(0);
        }
        catch (IOException e){
            DbgLog.error("SJH: Could not write in flushLineBuffer()");
        }
        milliTime   = System.currentTimeMillis();
        nanoTime    = System.nanoTime();
        addField(String.format(Locale.US, "%.3f",(milliTime - msBase) / 1.0E3));
        addField(String.format(Locale.US, "%.3f",(nanoTime - nsBase) / 1.0E6));
        nsBase      = nanoTime;
    }

    public void closeDataLogger() {
        try {
            writer.close();
        }
        catch (IOException e) {
            DbgLog.error("SJH: IoException in closeDataLogger()");
        }
    }

    public void addField(String s) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(s);
    }

    public void addField(char c) {
        if (lineBuffer.length()>0) {
            lineBuffer.append(',');
        }
        lineBuffer.append(c);
    }

    public void addField(boolean b) {
        addField(b ? '1' : '0');
    }

    public void addField(byte b) {
        addField(Byte.toString(b));
    }

    public void addField(short s) {
        addField(Short.toString(s));
    }

    public void addField(long l) {
        addField(Long.toString(l));
    }

    public void addField(float f) {
        addField(Float.toString(f));
    }

    public void addField(String light, double d) {
        addField(Double.toString(d));
    }

    public void newLine() {
        flushLineBuffer();
    }

    @Override
    protected void finalize() throws Throwable {
        closeDataLogger();
        super.finalize();
    }
}