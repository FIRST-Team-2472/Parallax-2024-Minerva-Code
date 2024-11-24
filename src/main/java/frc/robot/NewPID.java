package frc.robot;



public class NewPID {
    
    private double Kp, Ki,I, Kd, error, lasterror, timeIncrement, time;

    public NewPID(double Kp, double Ki, double Kd, double time){
    this.Kd = Kd;
    this.Ki = Ki;
    this.Kp = Kp;
    this.time = time;
    I = 0;
    timeIncrement = 1/time/50; // 50 = 1/code refresh rate
}
    public double calculatePID(double setpoint, double sensorRead){
        lasterror = error;
        error = setpoint - sensorRead;
        if (error != 0 && I<1 && I>-1){
            if (error < 0) 
            I += -timeIncrement;
            else 
            I += timeIncrement;
        };
        return Math.min(1, Math.max(-1, (error * Kp) + Ki*I + ((error - lasterror)*Kd)));
    }
}
