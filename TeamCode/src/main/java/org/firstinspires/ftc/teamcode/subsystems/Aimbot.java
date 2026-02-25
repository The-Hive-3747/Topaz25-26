package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;
import org.firstinspires.ftc.teamcode.utilities.AimbotValues;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;


public class Aimbot implements Component{
    Pose currentPose = OpModeTransfer.currentPose;
    Vector currentVelocity;
    double velocity;
    double percentage;
    double hoodPos;
    double botDistance;
    double goalX, goalY, fieldGoalX, fieldGoalY;
    Alliance alliance;
    AimbotValues currentAimValues;

    public void preInit() {

    }
    public void postInit() {
        AimbotValues currentAimValues = getAimValues(botDistance);
    }
    public void update() {
        botDistance = this.getBotDistance();
        currentAimValues = this.getAimValues(botDistance);
        ActiveOpMode.telemetry().addData("Bot Distance", botDistance);
        ActiveOpMode.telemetry().addData("CURRENT AIM VALUES", currentAimValues.velocity);
        ActiveOpMode.telemetry().addData("CURRENT AIM VALUES", currentAimValues.hoodPos);
    }
    
    public void setCurrentPose(Pose pose, Vector velocity) {
        this.currentPose = pose;
        this.currentVelocity = velocity;
    }
    public Pose getCurrentPose() {
        return this.currentPose;
    }

    AimbotValues[] aimbotValues = { //These are all the actual values from tests on field, must be
                                    // in order of distance from least to greatest
            new AimbotValues(16.5, 2057, 0), //vel: 850//
            new AimbotValues(21.5, 2057, 0), //vel: 950//
            new AimbotValues(26.5, 2057, 0), //vel: 950//v: 950
            new AimbotValues(31.5, 2057, 0), //v: 950 h:727//v:1000
            new AimbotValues(36.5, 2157, 0), //v: 950 h: 1003//v:1050, h:1005
            new AimbotValues(41.5, 2242, 0), //v: 1000 h: 1243//v:
            new AimbotValues(46.5, 2285, 500), //v: 1050 h: 1237//v:
            new AimbotValues(51.5, 2322, 1000), //v: 1000 h: 1250//v:1140
            new AimbotValues(56.5, 2371, 1500), //v: 1050 h: 1243//v:1140
            new AimbotValues(61.5, 2456, 1750), //v: 1050 h: 1264//v:1180, h:1466
            new AimbotValues(69.5, 2542, 2100), //v: 1200 h: 1762//v:1240, h:1538
            new AimbotValues(76.5, 2542, 2200), //v: 1250 h: 1903//v:1280, h:1580
            new AimbotValues(111.5, 2985, 2250), //v: 1300 h: 1250//v: 1440, h:1835//h:1900//v:1410//v:1430, h:1945
            new AimbotValues(116.5, 3014, 2500), //v: 1350 h: 1400//v: 1440, h:1835//h:1940//v:1440, h:1965
            new AimbotValues(121.5, 3057, 2500), //v: 1400 h: 1500//v: 1440, h:1835//h:1940//v:1460, h:1965
            new AimbotValues(126.5, 3057, 2000), //v: 1450 h: 2000//v: 1440, h:1835//h:1950//v:1460, h:1975
            new AimbotValues(131.5, 3142, 1750), //v: 1500 h: 2000//v: 1440, h:1835//v: 1480, h:1960

            /*new AimbotValues(16.5, 850, 0), //vel: 850//
            new AimbotValues(21.5, 1050, 518), //vel: 950//
            new AimbotValues(26.5, 1050, 300), //vel: 950//v: 950
            new AimbotValues(31.5, 1050, 796), //v: 950 h:727//v:1000
            new AimbotValues(36.5, 1100, 1227), //v: 950 h: 1003//v:1050, h:1005
            new AimbotValues(41.5, 1100, 1992), //v: 1000 h: 1243//v:
            new AimbotValues(46.5, 1200, 817), //v: 1050 h: 1237//v:
            new AimbotValues(51.5, 1200, 728), //v: 1000 h: 1250//v:1140
            new AimbotValues(56.5, 1200, 2259), //v: 1050 h: 1243//v:1140
            new AimbotValues(61.5, 1140,1500), //v: 1050 h: 1264//v:1180, h:1466
            new AimbotValues(69.5, 1200, 1560), //v: 1200 h: 1762//v:1240, h:1538
            new AimbotValues(76.5, 1260, 1600), //v: 1250 h: 1903//v:1280, h:1580
            new AimbotValues(111.5, 1427, 3057), //v: 1300 h: 1250//v: 1440, h:1835//h:1900//v:1410//v:1430, h:1945
            new AimbotValues(116.5, 1439, 2880), //v: 1350 h: 1400//v: 1440, h:1835//h:1940//v:1440, h:1965
            new AimbotValues(121.5, 1455, 2826), //v: 1400 h: 1500//v: 1440, h:1835//h:1940//v:1460, h:1965
            new AimbotValues(126.5, 1483, 2688), //v: 1450 h: 2000//v: 1440, h:1835//h:1950//v:1460, h:1975
            new AimbotValues(131.5, 1505, 1985), //v: 1500 h: 2000//v: 1440, h:1835//v: 1480, h:1960
            /*new AimbotValues(16.5, 1000, 0), //vel: 850
            new AimbotValues(21.5, 1000, 0), //vel: 950
            new AimbotValues(26.5, 1000, 0), //vel: 950
            new AimbotValues(31.5, 1050, 726), //v: 950 h:727
            new AimbotValues(36.5, 1050, 1005), //v: 950 h: 1003
            new AimbotValues(41.5, 1050, 1248), //v: 1000 h: 1243
            new AimbotValues(46.5, 1100, 1236), //v: 1050 h: 1237
            new AimbotValues(51.5, 1150, 1236), //v: 1000 h: 1250
            new AimbotValues(56.5, 1150, 1457), //v: 1050 h: 1243
            new AimbotValues(61.5, 1200,1486), //v: 1050 h: 1264
            new AimbotValues(69.5, 1250, 1558), //v: 1200 h: 1762
            new AimbotValues(76.5, 1300, 1600), //v: 1250 h: 1903
            new AimbotValues(111.5, 1450, 1835), //v: 1300 h: 1250
            new AimbotValues(116.5, 1450, 1858), //v: 1350 h: 1400
            new AimbotValues(121.5, 1450, 1858), //v: 1400 h: 1500
            new AimbotValues(126.5, 1450, 1858), //v: 1450 h: 2000
            new AimbotValues(131.5, 1450, 1858), //v: 1500 h: 2000*/
    };
    public double getAimVelocity() { return currentAimValues.velocity;}
    public double getAimHoodPos() { return currentAimValues.hoodPos;}

    public AimbotValues getAimValues(double distance){
        AimbotValues next;
        AimbotValues last;
        last = null;

        percentage = 0;
        velocity = aimbotValues[0].velocity;
        hoodPos = aimbotValues[0].hoodPos;

        for(AimbotValues value : aimbotValues) {
            next = value;
            if (next != null && last != null && distance >= last.distance && distance < next.distance) {
                //math for linear interpolation code
                percentage = (distance - last.distance)/(next.distance-last.distance);
                velocity = (next.velocity - last.velocity) * percentage + last.velocity;
                hoodPos = (next.hoodPos - last.hoodPos) * percentage + last.hoodPos;
            }else if(next != null && last != null && distance > last.distance && distance >= next.distance){
                percentage = 0;
                velocity = next.velocity;
                hoodPos = next.hoodPos;
            }
            last = next;
        }
        return new AimbotValues(distance, velocity, hoodPos);
    }
    public void setAlliance(Alliance all) {
        this.alliance = all;
        if (this.alliance == Alliance.RED) {
            goalX = 120; //this is the point of the middle of the front panel of the goal
            fieldGoalX = 120;
            goalY = 129;
            fieldGoalY = 129;
        } else {
            goalX = 20;
            fieldGoalX = 20;
            goalY = 129;
            fieldGoalY = 129;
        }
    }

    public double getBotDistance() {
        //this is math for the distance from bot to goal using hypotenuse of x and y
        botDistance = Math.sqrt(Math.pow(goalX - currentPose.getX(), 2) + Math.pow(goalY - currentPose.getY(), 2));
        return botDistance;
    }

    public AimbotValues getAimbotValues() {
        return currentAimValues;
    }
}

