package org.firstinspires.ftc.teamcode;

import java.lang.Math;

public final class Vector {

    public static final Vector zero = new Vector(0, 0);

    public float x;
    public float y;

    public Vector(final float x, final float y) {
        this.x = x;
        this.y = y;
    }

    public float norm() {
        return (float) Math.sqrt(x * x + y * y);
    }

    public float getAngle() {
        if (x == 0 && y == 0) {
            return 0;
        } else {
            return (float) Math.atan2(y, x);
        }
    }

    public Vector scale(final float scalingFactor) {
        return new Vector(x * scalingFactor, y * scalingFactor);
    }

    public Vector unit() {
        if (x == 0 && y == 0) {
            return this;
        } else {
            return scale(1 / norm());
        }
    }

    public static Vector unit(final float angle) {
        return new Vector((float) Math.cos(angle), (float) Math.sin(angle));
    }

    public static Vector fromAngleAndRadius(final float theta, final float radius) {
        final float vectorX = (float) Math.cos(theta) * radius;
        final float vectorY = (float) Math.sin(theta) * radius;
        return new Vector(vectorX, vectorY);
    }

    public float dotProduct(final Vector other) {
        return x * other.x + y * other.y;
    }

    public Vector rotate(final float theta) {
        final float newX = (float) (x * Math.cos(theta) - y * Math.sin(theta));
        final float newY = (float) (x * Math.sin(theta) + y * Math.cos(theta));
        return new Vector(newX, newY);
    }

    public Vector rotate90(final boolean rotateCounterClockwise) {
        if (rotateCounterClockwise) {
            return new Vector(-y, x);
        } else {
            return new Vector(y, -x);
        }
    }

    public Vector abs() {
        return new Vector(Math.abs(x), Math.abs(y));
    }

    public Vector add(final Vector other) {
        return new Vector(x + other.x, y + other.y);
    }

    public Vector subtract(final Vector other) {
        return new Vector(x - other.x, y - other.y);
    }

    public float project(final float angle) {
        final Vector unitVector = unit(angle);
        return dotProduct(unitVector);
    }

    public static Vector longest(final Vector a, final Vector b) {
        return a.norm() > b.norm() ? a : b;
    }

    public static Vector shortest(final Vector a, final Vector b) {
        return a.norm() < b.norm() ? a : b;
    }

    public static Vector max(final Vector a, final Vector b) {
        return new Vector (Math.max(a.x, b.x), Math.max(a.y, b.y));
    }

    public static Vector min(final Vector a, final Vector b) {
        return new Vector (Math.min(a.x, b.x), Math.min(a.y, b.y));
    }
}