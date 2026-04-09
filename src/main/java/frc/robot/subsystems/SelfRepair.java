package frc.robot.subsystems;
import java.util.ArrayList;

public class SelfRepair {
    //mech fix the robot please
    //apply mending enchant
    public ArrayList <String> strategies = new ArrayList<>();

    public SelfRepair(){
        strategies.add("Mech fix the robot pretty please");
        strategies.add("Apply mending enchant?");
        strategies.add("What if we tried getting a mending villager");
        strategies.add("no thanks joseph");
        strategies.add("albert and einstein in the pit");
        strategies.add("zero tolerance confirmed");
        strategies.add("I wish we had a robot to work with 👉👈");
        // strategies.add("W programming and Electrical");
        // strategies.add("have you tried turning it on and off again?");

    }

    public String getAdvice(){
        int guess = (int)Math.random() * 10;
        return strategies.get(guess);
    }
}
