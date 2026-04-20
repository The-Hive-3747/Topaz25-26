package org.firstinspires.ftc.teamcode.utilities;

import java.util.ArrayList;
import java.util.List;

public class Motif {
    public Artifact left, front, right;
    public Motif autoMotif = null;
    private List<Shoot> shotOrder;
    public enum Shoot {
        LEFT,
        RIGHT,
        LEFT_ALL,
        RIGHT_ALL
    }
    public Motif(Artifact left, Artifact front, Artifact right) {
        this.left = normalize(left);
        this.front = normalize(front);
        this.right = normalize(right);
    }

    private static Artifact normalize(Artifact a) {
        if (a == Artifact.MAYBE_GREEN) return Artifact.GREEN;
        if (a == Artifact.MAYBE_PURPLE) return Artifact.PURPLE;
        return a;
    }

    public void setAutoMotif(Motif autoMotif) {
        this.autoMotif = autoMotif;
    }

    public List<Shoot> getShotOrder(Motif possessed, Motif autoMotif) {
        if (possessed.left.equals(autoMotif.left)) {
            if (possessed.right.equals(autoMotif.front) && !possessed.right.equals(autoMotif.right)) {
                return shotOrder = List.of(Shoot.LEFT, Shoot.RIGHT_ALL);
            }
            return shotOrder = List.of(Shoot.LEFT_ALL);
        } else if (possessed.right.equals(autoMotif.left)) {
            if (possessed.left.equals(autoMotif.front) && !possessed.left.equals(autoMotif.right)) {
                return shotOrder = List.of(Shoot.RIGHT, Shoot.LEFT_ALL);
            }
            return shotOrder = List.of(Shoot.RIGHT_ALL);
        } else if (possessed.left.equals(autoMotif.front) || possessed.front.equals(autoMotif.right)) {
            return shotOrder = List.of(Shoot.RIGHT, Shoot.LEFT_ALL);
        }
        return shotOrder = List.of(Shoot.RIGHT_ALL);
    }

    public List<Shoot> getShotOrder(Motif possessed) {
        if (autoMotif == null) {
            return shotOrder = List.of(Shoot.RIGHT_ALL);
        }
        return getShotOrder(possessed, autoMotif);
    }

    public static Motif getMotif(int aprilTagID) {
        if (aprilTagID == 21) {
            return new Motif(Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE);
        } else if (aprilTagID == 22) {
            return new Motif(Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE);
        } else if (aprilTagID == 23) {
            return new Motif(Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN);
        }
        return null;
    }
}
