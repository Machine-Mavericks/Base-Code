package frc.robot;

public class Utils {
    
    /**
     * Function determines difference between angle1 and angle 2
     * accounting for -180/180 boundary
     * @param angle1 The first angle to compare, in degrees
     * @param angle2 The second angle to compare, in degrees
     * @return Smallest difference between angles, in degrees
     * @apiNote This works by noticing that we want to take the difference between the two angles, angle1 and angle2 and wrap that result to the range [-180, 179). The mod operator allows us to wrap something to the range [0, n). I.e. x % n "wraps" (with a caveat for x < 0) x to the range [0, n).Our range starts at -180 rather than 0, so we shift it over by adding 180. Then we wrap to 360, then we shift back. That's what the first line of the method does.The second line takes care of that little wrinkle with negative numbers. If angle2 - angle1 + 180 happened to be less than 0, then diff will be less than -180. In that case we just wrap it back into range by adding 360 to it. Otherwise we do nothing.As an added bonus, the input angles are completely unconstrained. They need not be between -360 and 360. They can be anything.
     */
     public static double AngleDifference( double angle1, double angle2 )
    {
        double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }
}
