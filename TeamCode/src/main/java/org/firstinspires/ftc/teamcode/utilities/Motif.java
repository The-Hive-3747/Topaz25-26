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

    /**
     * Creates a new motif (set of 3 artifacts)
     * @param left left on the robot, 1 on the obelisk
     * @param front front on the robot, 2 on the obelisk
     * @param right right on the robot, 3 on the obelisk
     */
    public Motif(Artifact left, Artifact front, Artifact right) {
        this.left = normalize(left);
        this.front = normalize(front);
        this.right = normalize(right);
    }

    /**
     * normalizes to assume that maybes are the real deal
     * @param a artifact to normalize
     * @return normalized artifact
     */
    private static Artifact normalize(Artifact a) {
        if (a == Artifact.MAYBE_GREEN) return Artifact.GREEN;
        if (a == Artifact.MAYBE_PURPLE) return Artifact.PURPLE;
        return a;
    }

    /**
     * Sets the motif for this autonomous
     * @param autoMotif motif to save
     */
    public void setAutoMotif(Motif autoMotif) {
        this.autoMotif = autoMotif;
    }

    /**
     * Returns best sorted shot order
     * @param possessed Artifacts you have
     * @param autoMotif Motif to match
     * @return Shot order
     */
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

    /**
     * Returns best sorted shot order
     * @param possessed Artifacts you have
     * @return Shot order
     */
    public List<Shoot> getShotOrder(Motif possessed) {
        if (autoMotif == null) {
            return shotOrder = List.of(Shoot.RIGHT_ALL);
        }
        return getShotOrder(possessed, autoMotif);
    }

    /**
     * Gets motif based on obelisk
     * @param aprilTagID integer
     * @return Obelisk motif
     */
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

    public static Motif pPG = new Motif(Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN);
    public static Motif pGP = new Motif(Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE);
    public static Motif gPP = new Motif(Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE);

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Motif)) return false;
        Motif other = (Motif) o;
        return this.left == other.left && this.front == other.front && this.right == other.right;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(left, front, right);
    }
}